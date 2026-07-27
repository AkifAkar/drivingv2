// ==========================================================
//  MASTER PICO  -  Ethernet (W5500) <-> UART link to slave
//
//  Sends to the slave on UART0 (GP12 TX / GP13 RX), plain ASCII:
//      motor1:<angle 90..270>\n      steering target, degrees
//      motor2:<speed -255..255>\n    drive command, raw PWM units
//
//  Receives from the slave:
//      fb:<current_angle>,<homed 0|1>,<raw_angle>\n
// ==========================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pt.h"
// NOTE: roboclaw_bridge.h is no longer needed on this Pico.

// Wiznet Libraries
#include "port_common.h"
#include "wizchip_conf.h"
#include "w5x00_spi.h"
#include "socket.h"

// --- Configuration ---
#define PLL_SYS_KHZ (133 * 1000)
#define SOCKET_UDP        0
#define PORT_UDP          5000
#define ETHERNET_BUF_SIZE 2048

#ifndef LAST_IP_OCTET
    #define LAST_IP_OCTET 32
#endif

#ifndef MAC_LAST_BYTE
    #define MAC_LAST_BYTE 0x32
#endif

// IP
static wiz_NetInfo g_net_info = {
    .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, MAC_LAST_BYTE},
    .ip = {10, 42, 0, LAST_IP_OCTET},
    .sn = {255, 255, 255, 0},
    .gw = {10, 42, 0, 254},
    .dns = {8, 8, 8, 8},
    .dhcp = NETINFO_STATIC
};

// Buffers
static uint8_t g_rx_buf[ETHERNET_BUF_SIZE] = {0,};
static uint8_t g_tx_buf[ETHERNET_BUF_SIZE] = {0,};

// Host Info
static uint8_t g_dest_ip[4] = {0, 0, 0, 0};
static uint16_t g_dest_port = 0;
static bool g_has_connection = false;

// ----------------------------------------------------------
// --- Motor Link Config (same physical pins as the old RoboClaw) ---
// ----------------------------------------------------------
#define LINK_UART_ID      uart0
#define LINK_TX_PIN       12     // -> slave Pico RX
#define LINK_RX_PIN       13     // <- slave Pico TX
#define LINK_BAUD         115200

// Command pair is repeated at this rate even when nothing changed.
// Doubles as the heartbeat for the slave's own watchdog.
#define LINK_PERIOD_MS    50     // 20 Hz
#define LINK_MIN_GAP_MS   10     // rate limit for change-triggered sends

// UDP telemetry rate. 20 Hz so a step response / oscillation is actually
// resolvable on the host; at the old 5 Hz you could not see a 2 Hz hunt.
#define UDP_TX_PERIOD_MS  50

// --- Angle limits (degrees, sent on the wire) ---
#define ANGLE_MIN 90
#define ANGLE_MAX 270
#define ANGLE_CENTER ((ANGLE_MIN + ANGLE_MAX) / 2)

// --- If the steering direction is mechanically inverted ---
// Applied here so the slave only ever sees a true mechanical angle.
#ifndef INVERTED
    #define INVERTED 1
#endif

// --- Speed scaling ---
// The host still sends -127..127 in its JSON. The wire (and the BTS7960)
// wants -255..255, so we scale by 2 exactly like the old abs()*2 did.
// If you switch the host to sending -255..255 directly, set SPEED_SCALE to 1
// and SPEED_INPUT_MAX to 255.
#define SPEED_INPUT_MAX   127
#define SPEED_SCALE       2
#define SPEED_WIRE_MAX    255

// --- Globals ---
volatile int32_t g_desired_angle_val = ANGLE_CENTER;  // motor1 target, DEGREES
volatile int32_t g_desired_speed_val = 0;             // motor2 command, WIRE units -255..255
volatile float   g_current_angle_val = ANGLE_CENTER;  // reported by the slave
volatile float   g_raw_angle_val = ANGLE_CENTER;      // slave's raw AS5600, before its STEERING_OFFSET_DEG
volatile bool    g_slave_homed = false;               // reported by the slave
volatile bool    g_link_alive = false;                // have we heard from the slave lately?
volatile bool    g_network_ok = false;                 // W5500 responded and socket opened at boot
// Live PID tuning passthrough. NaN-free sentinel: gains are only forwarded to
// the slave once the host has actually sent some, so normal operation never
// overrides the slave's own compiled-in defaults.
volatile float   g_kp = 0.0f, g_ki = 0.0f, g_kd = 0.0f;
volatile bool    g_pid_dirty = false;   // host sent new gains, not yet forwarded
volatile bool    g_pid_set = false;     // host has sent gains at least once
volatile int32_t g_steer_raw = 0;       // open-loop steering test PWM
volatile bool    g_steer_raw_dirty = false;
static   uint32_t g_link_last_rx_ms = 0;

// --- Threads ---
static struct pt pt_link_tx;
static struct pt pt_link_rx;
static struct pt pt_udp_rx;
static struct pt pt_udp_tx;

// --- Helper Functions ---
static void set_clock_khz(void) {
    set_sys_clock_khz(PLL_SYS_KHZ, true);
    clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                    PLL_SYS_KHZ * 1000, PLL_SYS_KHZ * 1000);
}

void update_angle_target(int angle_input) {
    if (angle_input < ANGLE_MIN) angle_input = ANGLE_MIN;
    if (angle_input > ANGLE_MAX) angle_input = ANGLE_MAX;
    if (INVERTED) angle_input = 360 - angle_input;
    g_desired_angle_val = angle_input;
}

void update_speed_target(int speed_input) {
    if (speed_input >  SPEED_INPUT_MAX) speed_input =  SPEED_INPUT_MAX;
    if (speed_input < -SPEED_INPUT_MAX) speed_input = -SPEED_INPUT_MAX;

    int wire = speed_input * SPEED_SCALE;
    if (wire >  SPEED_WIRE_MAX) wire =  SPEED_WIRE_MAX;
    if (wire < -SPEED_WIRE_MAX) wire = -SPEED_WIRE_MAX;

    g_desired_speed_val = wire;
}

// --- JSON Parser (unchanged behaviour) ---
// Format: {"command_type": "control", "action": "led", "value": 0, "speed": 100, "angle": 180, "light": 1}
void parse_and_process_json(char* json_str) {
    // 1. Parse ANGLE
    char* key_angle = strstr(json_str, "\"angle\"");
    if (key_angle) {
        char* val_str = strstr(key_angle, ":");
        if (val_str) {
            val_str++;
            int angle = atoi(val_str);
            if (angle > 0) update_angle_target(angle);
        }
    }

    // 2. Parse SPEED
    char* key_speed = strstr(json_str, "\"speed\"");
    if (key_speed) {
        char* val_str = strstr(key_speed, ":");
        if (val_str) {
            val_str++;
            int speed = atoi(val_str);
            update_speed_target(speed);
        }
    }

    char* key_light = strstr(json_str, "\"light\"");
    if (key_light) {
        // Not included yet
    }

    // 3. Parse live PID gains (optional -- only present while tuning).
    //    All three must be supplied together.
    char* key_kp = strstr(json_str, "\"kp\"");
    char* key_ki = strstr(json_str, "\"ki\"");
    char* key_kd = strstr(json_str, "\"kd\"");
    if (key_kp && key_ki && key_kd) {
        char* vp = strstr(key_kp, ":");
        char* vi = strstr(key_ki, ":");
        char* vd = strstr(key_kd, ":");
        if (vp && vi && vd) {
            g_kp = (float)atof(vp + 1);
            g_ki = (float)atof(vi + 1);
            g_kd = (float)atof(vd + 1);
            g_pid_set = true;
            g_pid_dirty = true;
        }
    }

    // 4. Parse open-loop steering test PWM (tuning only).
    //    Re-send continuously to hold it; the slave expires it after ~1s.
    char* key_sraw = strstr(json_str, "\"steer_raw\"");
    if (key_sraw) {
        char* vs = strstr(key_sraw, ":");
        if (vs) {
            g_steer_raw = atoi(vs + 1);
            g_steer_raw_dirty = true;
        }
    }
}

// ----------------------------------------------------------
// Motor link helpers
// ----------------------------------------------------------
static void link_init(void) {
    uart_init(LINK_UART_ID, LINK_BAUD);
    gpio_set_function(LINK_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LINK_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(LINK_UART_ID, false, false);
    uart_set_format(LINK_UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(LINK_UART_ID, true);
}

// One newline-terminated ASCII line to the slave.
// Lines are <16 B and the TX FIFO is 32 B deep, so this practically
// never blocks at 115200 baud.
static void link_send(const char *fmt, ...) {
    char buf[48];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        if (n > (int)sizeof(buf) - 1) n = (int)sizeof(buf) - 1;
        uart_write_blocking(LINK_UART_ID, (const uint8_t *)buf, (size_t)n);
    }
}

// ----------------------------------------------------------
// THREAD: Motor Link TX
// ----------------------------------------------------------
static int thread_link_tx(struct pt *pt) {
    static uint32_t last_tx_time = 0;
    static int32_t  last_sent_angle = ANGLE_CENTER;
    static int32_t  last_sent_speed = 0;
    static bool     changed;
    static uint32_t now;

    PT_BEGIN(pt);

    while (1) {
        now = to_ms_since_boot(get_absolute_time());

        // Forward new PID gains immediately when the host sends them.
        if (g_pid_dirty) {
            link_send("pid:%.4f,%.4f,%.4f\n", g_kp, g_ki, g_kd);
            printf("[LINK] pid:%.4f,%.4f,%.4f\n", g_kp, g_ki, g_kd);
            g_pid_dirty = false;
        }

        // Forward open-loop steering test PWM (tuning only).
        if (g_steer_raw_dirty) {
            link_send("sraw:%ld\n", (long)g_steer_raw);
            g_steer_raw_dirty = false;
        }

        changed = (g_desired_angle_val != last_sent_angle) ||
                  (g_desired_speed_val != last_sent_speed);

        if ((changed && (now - last_tx_time >= LINK_MIN_GAP_MS)) ||
            (now - last_tx_time >= LINK_PERIOD_MS)) {

            link_send("motor1:%ld\n", (long)g_desired_angle_val);
            link_send("motor2:%ld\n", (long)g_desired_speed_val);

            if (changed) {
                printf("[LINK] motor1:%ld motor2:%ld\n",
                       (long)g_desired_angle_val, (long)g_desired_speed_val);
            }

            last_sent_angle = g_desired_angle_val;
            last_sent_speed = g_desired_speed_val;
            last_tx_time    = now;
        }
        PT_YIELD(pt);
    }
    PT_END(pt);
}

// ----------------------------------------------------------
// THREAD: Motor Link RX  ->  "fb:<current_angle>,<homed>\n"
// ----------------------------------------------------------
static int thread_link_rx(struct pt *pt) {
    static char    line[64];
    static size_t  idx = 0;
    static int     c;
    static uint32_t now;

    PT_BEGIN(pt);

    while (1) {
        while (uart_is_readable(LINK_UART_ID)) {
            c = uart_getc(LINK_UART_ID);

            if (c == '\n' || c == '\r') {
                if (idx > 0) {
                    line[idx] = '\0';
                    char *p = strstr(line, "fb:");
                    if (p) {
                        p += 3;
                        g_current_angle_val = (float)atof(p);
                        char *comma1 = strchr(p, ',');
                        if (comma1) {
                            g_slave_homed = (atoi(comma1 + 1) != 0);
                            char *comma2 = strchr(comma1 + 1, ',');
                            if (comma2) g_raw_angle_val = (float)atof(comma2 + 1);
                        }
                        if (!g_link_alive) {
                            printf("[LINK] Slave connected.\n");
                            gpio_put(25, 1);
                        }
                        g_link_alive = true;
                        g_link_last_rx_ms = to_ms_since_boot(get_absolute_time());
                    }
                    idx = 0;
                }
            } else if (idx < sizeof(line) - 1) {
                line[idx++] = (char)c;
            } else {
                idx = 0; // overlong garbage, resync
            }
        }

        // Link watchdog: slave silent for 500 ms -> flag it to the host
        now = to_ms_since_boot(get_absolute_time());
        if (g_link_alive && (now - g_link_last_rx_ms > 500)) {
            g_link_alive = false;
            gpio_put(25, 0);
            printf("[LINK] Slave silent! (no fb: line in >500ms)\n");
        }

        PT_YIELD(pt);
    }
    PT_END(pt);
}

// ----------------------------------------------------------
// THREAD: UDP Receive (Burst Mode / Low Latency)
// ----------------------------------------------------------
static int thread_udp_rx(struct pt *pt) {
    static int32_t recv_len;
    static int32_t last_good_len;
    static uint8_t remote_ip[4];
    static uint16_t remote_port;
    static bool has_new_command;
    static uint32_t last_command_time = 0;
    static uint32_t timestamp = 0;

    PT_BEGIN(pt);
    while (1) {
        has_new_command = false;
        last_good_len = 0;

        // Drain the buffer entirely. Only keep the latest packet.
        while (getSn_RX_RSR(SOCKET_UDP) > 0) {
            recv_len = recvfrom(SOCKET_UDP, g_rx_buf, ETHERNET_BUF_SIZE - 1, remote_ip, &remote_port);
            if (recv_len > 0) {
                has_new_command = true;
                last_good_len = recv_len;   // guard: a later recvfrom may fail
                memcpy(g_dest_ip, remote_ip, 4);
                g_dest_port = remote_port;
                g_has_connection = true;
            }
        }

        if (has_new_command) {
            g_rx_buf[last_good_len] = '\0';
            parse_and_process_json((char*)g_rx_buf);
            last_command_time = to_ms_since_boot(get_absolute_time());
        } else if (g_has_connection) {
            // Watchdog: stop the rover if no command received for 500 ms
            if (to_ms_since_boot(get_absolute_time()) - last_command_time > 500) {
                if (g_desired_speed_val != 0) {
                    g_desired_speed_val = 0;
                    printf("[Watchdog] Connection lost! Stopping motors.\n");
                }
            }
        }

        timestamp = to_ms_since_boot(get_absolute_time());
        PT_WAIT_UNTIL(pt, to_ms_since_boot(get_absolute_time()) - timestamp >= 10);
    }
    PT_END(pt);
}

// ----------------------------------------------------------
// THREAD: UDP Transmit
// ----------------------------------------------------------
static int thread_udp_tx(struct pt *pt) {
    static uint32_t timestamp;
    static int len;

    PT_BEGIN(pt);
    while (1) {
        if (g_has_connection) {
            len = snprintf((char*)g_tx_buf, ETHERNET_BUF_SIZE,
                "{\"current_angle\": %.1f, \"raw_angle\": %.1f, \"target_angle\": %ld, "
                "\"target_speed\": %ld, \"homed\": %d, \"link\": %d, "
                "\"kp\": %.4f, \"ki\": %.4f, \"kd\": %.4f, \"pid_set\": %d}",
                g_current_angle_val, g_raw_angle_val,
                (long)g_desired_angle_val, (long)g_desired_speed_val,
                g_slave_homed ? 1 : 0, g_link_alive ? 1 : 0,
                g_kp, g_ki, g_kd, g_pid_set ? 1 : 0
            );
            sendto(SOCKET_UDP, g_tx_buf, len, g_dest_ip, g_dest_port);
        }
        timestamp = to_ms_since_boot(get_absolute_time());
        PT_WAIT_UNTIL(pt, to_ms_since_boot(get_absolute_time()) - timestamp >= UDP_TX_PERIOD_MS);
    }
    PT_END(pt);
}

// ----------------------------------------------------------
// MAIN
// ----------------------------------------------------------
// Bounded replacement for the library's wizchip_check(), which is an
// infinite while(1) loop if the W5500 doesn't respond over SPI. That hang
// used to happen before the UART link to the slave was ever started, so a
// disconnected/unresponsive Ethernet chip took the whole motor link down
// with it. This gives up after a timeout instead.
static bool network_chip_present(uint32_t timeout_ms) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    do {
        if (getVERSIONR() == 0x04) return true;
        sleep_ms(50);
    } while (to_ms_since_boot(get_absolute_time()) - start < timeout_ms);
    return false;
}

void setup() {
    set_clock_khz();
    stdio_init_all();
    sleep_ms(2000);
    printf("--- Pico Rover Control (UART link to BTS7960 slave) ---\n");

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    // Motor link comes up FIRST and unconditionally. Steering/drive control
    // between the two Picos must not depend on Ethernet being present.
    link_init();
    PT_INIT(&pt_link_tx);
    PT_INIT(&pt_link_rx);

    // Network bring-up is now best-effort: if the W5500 doesn't answer
    // within the timeout (unplugged, unpowered, miswired SPI), log it and
    // move on. The rover still drives from UDP commands once you fix
    // the Ethernet side and power-cycle -- but the motor link, and this
    // firmware, no longer hang waiting for it.
    wizchip_spi_initialize();
    wizchip_cris_initialize();
    wizchip_reset();
    wizchip_initialize();

    g_network_ok = network_chip_present(3000);
    if (!g_network_ok) {
        printf("[NET] W5500 not responding after 3s -- check SPI wiring/power "
               "and that the chip itself is seated. Continuing WITHOUT "
               "network; motor link is still active.\n");
        return;
    }

    network_initialize(g_net_info);
    printf("--- Network: IP %d.%d.%d.%d  GW %d.%d.%d.%d  SN %d.%d.%d.%d  UDP port %d ---\n",
           g_net_info.ip[0], g_net_info.ip[1], g_net_info.ip[2], g_net_info.ip[3],
           g_net_info.gw[0], g_net_info.gw[1], g_net_info.gw[2], g_net_info.gw[3],
           g_net_info.sn[0], g_net_info.sn[1], g_net_info.sn[2], g_net_info.sn[3],
           PORT_UDP);

    if (socket(SOCKET_UDP, Sn_MR_UDP, PORT_UDP, 0) != SOCKET_UDP) {
        printf("[NET] UDP socket open failed -- continuing WITHOUT network; "
               "motor link is still active.\n");
        g_network_ok = false;
        return;
    }

    PT_INIT(&pt_udp_rx);
    PT_INIT(&pt_udp_tx);
    g_network_ok = true;
}

int main() {
    setup();
    while (true) {
        thread_link_tx(&pt_link_tx);
        thread_link_rx(&pt_link_rx);
        if (g_network_ok) {
            thread_udp_rx(&pt_udp_rx);
            thread_udp_tx(&pt_udp_tx);
        }
    }
}