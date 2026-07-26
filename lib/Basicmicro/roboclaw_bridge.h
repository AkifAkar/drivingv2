#ifndef ROBOCLAW_BRIDGE_H
#define ROBOCLAW_BRIDGE_H

#include "pico/stdlib.h"
#include "hardware/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Roboclaw_Handle Roboclaw_Handle;

// --- Setup ---
Roboclaw_Handle* roboclaw_create(uart_inst_t *uart, uint tx_pin, uint rx_pin, uint32_t timeout_us);
void roboclaw_begin(Roboclaw_Handle* handle, uint32_t baudrate);
void roboclaw_destroy(Roboclaw_Handle* handle); // Good practice to have

// --- Basic Motor Control (Duty Cycle) ---
void roboclaw_forward_m1(Roboclaw_Handle* handle, uint8_t address, uint8_t speed);
void roboclaw_backward_m1(Roboclaw_Handle* handle, uint8_t address, uint8_t speed);
void roboclaw_forward_m2(Roboclaw_Handle* handle, uint8_t address, uint8_t speed);
void roboclaw_backward_m2(Roboclaw_Handle* handle, uint8_t address, uint8_t speed);
void roboclaw_stop(Roboclaw_Handle* handle, uint8_t address); // Stops both

// --- Encoder & Speed Reading ---
uint32_t roboclaw_read_enc_m1(Roboclaw_Handle* handle, uint8_t address, uint8_t *status, bool *valid);
uint32_t roboclaw_read_enc_m2(Roboclaw_Handle* handle, uint8_t address, uint8_t *status, bool *valid);
uint32_t roboclaw_read_speed_m1(Roboclaw_Handle* handle, uint8_t address, uint8_t *status, bool *valid);
uint32_t roboclaw_read_speed_m2(Roboclaw_Handle* handle, uint8_t address, uint8_t *status, bool *valid);
void roboclaw_reset_encoders(Roboclaw_Handle* handle, uint8_t address);

// --- PID Speed Control (Closed Loop) ---
// Speed is in encoder counts per second (qp_ps)
bool roboclaw_speed_m1(Roboclaw_Handle* handle, uint8_t address, uint32_t speed);
bool roboclaw_speed_m2(Roboclaw_Handle* handle, uint8_t address, uint32_t speed);
bool roboclaw_speed_m1m2(Roboclaw_Handle* handle, uint8_t address, uint32_t speed1, uint32_t speed2);

// --- PID Position Control ---
// Moves to specific encoder count
bool roboclaw_speed_accel_deccel_pos_m1(Roboclaw_Handle* handle, uint8_t address, uint32_t accel, uint32_t speed, uint32_t deccel, uint32_t position, uint8_t flag);
bool roboclaw_speed_accel_deccel_pos_m2(Roboclaw_Handle* handle, uint8_t address, uint32_t accel, uint32_t speed, uint32_t deccel, uint32_t position, uint8_t flag);

// --- System Info ---
uint16_t roboclaw_read_main_battery(Roboclaw_Handle* handle, uint8_t address, bool *valid);
uint16_t roboclaw_read_temp(Roboclaw_Handle* handle, uint8_t address, bool *valid);
uint32_t roboclaw_read_error(Roboclaw_Handle* handle, uint8_t address, bool *valid);

#ifdef __cplusplus
}
#endif

#endif // ROBOCLAW_BRIDGE_H