#include "roboclaw_bridge.h"
#include "Basicmicro.h"
#include <stdlib.h>

extern "C" {

    struct Roboclaw_Handle {
        Basicmicro* obj;
    };

    // --- Setup ---
    Roboclaw_Handle* roboclaw_create(uart_inst_t *uart, uint tx_pin, uint rx_pin, uint32_t timeout_us) {
        Roboclaw_Handle* handle = (Roboclaw_Handle*)malloc(sizeof(Roboclaw_Handle));
        handle->obj = new Basicmicro(uart, tx_pin, rx_pin, timeout_us);
        return handle;
    }

    void roboclaw_begin(Roboclaw_Handle* handle, uint32_t baudrate) {
        if (handle && handle->obj) handle->obj->begin(baudrate);
    }

    void roboclaw_destroy(Roboclaw_Handle* handle) {
        if (handle) {
            delete handle->obj;
            free(handle);
        }
    }

    // --- Basic Motor Control ---
    void roboclaw_forward_m1(Roboclaw_Handle* handle, uint8_t address, uint8_t speed) {
        if (handle && handle->obj) handle->obj->ForwardM1(address, speed);
    }
    void roboclaw_backward_m1(Roboclaw_Handle* handle, uint8_t address, uint8_t speed) {
        if (handle && handle->obj) handle->obj->BackwardM1(address, speed);
    }
    void roboclaw_forward_m2(Roboclaw_Handle* handle, uint8_t address, uint8_t speed) {
        if (handle && handle->obj) handle->obj->ForwardM2(address, speed);
    }
    void roboclaw_backward_m2(Roboclaw_Handle* handle, uint8_t address, uint8_t speed) {
        if (handle && handle->obj) handle->obj->BackwardM2(address, speed);
    }
    void roboclaw_stop(Roboclaw_Handle* handle, uint8_t address) {
        if (handle && handle->obj) {
            handle->obj->ForwardM1(address, 0);
            handle->obj->ForwardM2(address, 0);
        }
    }

    // --- Encoder & Speed Reading ---
    uint32_t roboclaw_read_enc_m1(Roboclaw_Handle* handle, uint8_t address, uint8_t *status, bool *valid) {
        if (handle && handle->obj) return handle->obj->ReadEncM1(address, status, valid);
        if (valid) *valid = false; return 0;
    }
    uint32_t roboclaw_read_enc_m2(Roboclaw_Handle* handle, uint8_t address, uint8_t *status, bool *valid) {
        if (handle && handle->obj) return handle->obj->ReadEncM2(address, status, valid);
        if (valid) *valid = false; return 0;
    }
    uint32_t roboclaw_read_speed_m1(Roboclaw_Handle* handle, uint8_t address, uint8_t *status, bool *valid) {
        if (handle && handle->obj) return handle->obj->ReadSpeedM1(address, status, valid);
        if (valid) *valid = false; return 0;
    }
    uint32_t roboclaw_read_speed_m2(Roboclaw_Handle* handle, uint8_t address, uint8_t *status, bool *valid) {
        if (handle && handle->obj) return handle->obj->ReadSpeedM2(address, status, valid);
        if (valid) *valid = false; return 0;
    }
    void roboclaw_reset_encoders(Roboclaw_Handle* handle, uint8_t address) {
        if (handle && handle->obj) handle->obj->ResetEncoders(address);
    }

    // --- PID Speed Control ---
    bool roboclaw_speed_m1(Roboclaw_Handle* handle, uint8_t address, uint32_t speed) {
        if (handle && handle->obj) return handle->obj->SpeedM1(address, speed);
        return false;
    }
    bool roboclaw_speed_m2(Roboclaw_Handle* handle, uint8_t address, uint32_t speed) {
        if (handle && handle->obj) return handle->obj->SpeedM2(address, speed);
        return false;
    }
    bool roboclaw_speed_m1m2(Roboclaw_Handle* handle, uint8_t address, uint32_t speed1, uint32_t speed2) {
        if (handle && handle->obj) return handle->obj->SpeedM1M2(address, speed1, speed2);
        return false;
    }

    // --- PID Position Control ---
    bool roboclaw_speed_accel_deccel_pos_m1(Roboclaw_Handle* handle, uint8_t address, uint32_t accel, uint32_t speed, uint32_t deccel, uint32_t position, uint8_t flag) {
        if (handle && handle->obj) return handle->obj->SpeedAccelDeccelPositionM1(address, accel, speed, deccel, position, flag);
        return false;
    }
    bool roboclaw_speed_accel_deccel_pos_m2(Roboclaw_Handle* handle, uint8_t address, uint32_t accel, uint32_t speed, uint32_t deccel, uint32_t position, uint8_t flag) {
        if (handle && handle->obj) return handle->obj->SpeedAccelDeccelPositionM2(address, accel, speed, deccel, position, flag);
        return false;
    }

    // --- System Info ---
    uint16_t roboclaw_read_main_battery(Roboclaw_Handle* handle, uint8_t address, bool *valid) {
        if (handle && handle->obj) return handle->obj->ReadMainBatteryVoltage(address, valid);
        if (valid) *valid = false; return 0;
    }
    uint16_t roboclaw_read_temp(Roboclaw_Handle* handle, uint8_t address, bool *valid) {
        uint16_t temp = 0;
        if (handle && handle->obj) *valid = handle->obj->ReadTemp(address, temp);
        else if (valid) *valid = false;
        return temp;
    }
    uint32_t roboclaw_read_error(Roboclaw_Handle* handle, uint8_t address, bool *valid) {
        if (handle && handle->obj) return handle->obj->ReadError(address, valid);
        if (valid) *valid = false; return 0;
    }
}