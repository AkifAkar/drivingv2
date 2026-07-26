#include "Basicmicro.h"
#include "pico/stdlib.h"
#include <stdarg.h>
#include <cstring>

#define MAXRETRY 3

// Helper macros
#define SetDWORDval(arg) (uint8_t)(((uint32_t)arg)>>24),(uint8_t)(((uint32_t)arg)>>16),(uint8_t)(((uint32_t)arg)>>8),(uint8_t)arg
#define SetWORDval(arg) (uint8_t)(((uint16_t)arg)>>8),(uint8_t)arg

// Arduino "constrain" macro implementation for standard C++
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

const int16_t ST_NOT_SET = -256;

/**
 * @brief Constructor for Pico SDK
 */
Basicmicro::Basicmicro(uart_inst_t *uart, uint tx_pin, uint rx_pin, uint32_t tout)
{
	ST_Power = ST_Turn = ST_NOT_SET;
	timeout = tout;
    _uart = uart;
    _tx_pin = tx_pin;
    _rx_pin = rx_pin;
}

Basicmicro::~Basicmicro()
{
}

/**
 * @brief Initialize the serial connection
 */
void Basicmicro::begin(uint32_t baudrate)
{
    uart_init(_uart, baudrate);
    gpio_set_function(_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(_rx_pin, GPIO_FUNC_UART);
    
    // Enable FIFO (CRITICAL for high reliability on RX)
    uart_set_fifo_enabled(_uart, true);
    
    // Default format 8N1
    uart_set_format(_uart, 8, 1, UART_PARITY_NONE);
}

// --- IO Primitives for Pico ---

int Basicmicro::available()
{
    return uart_is_readable(_uart);
}

size_t Basicmicro::write(uint8_t byte)
{
    // Blocking write to ensure command integrity
    uart_putc_raw(_uart, (char)byte);
    return 1;
}

int Basicmicro::read()
{
    if (uart_is_readable(_uart))
        return uart_getc(_uart);
    return -1;
}

void Basicmicro::flush()
{
    uart_tx_wait_blocking(_uart);
}

/**
 * @brief Read data from serial with timeout (Pico implementation)
 */
int Basicmicro::read(uint32_t timeout_us)
{
    uint32_t start = time_us_32();
    // Empty buffer?
    while(!uart_is_readable(_uart)){
       if((time_us_32() - start) >= timeout_us)
          return -1;
    }
    return uart_getc(_uart);
}

void Basicmicro::clear()
{
    while(uart_is_readable(_uart))
        uart_getc(_uart);
}

// --- Logic Below is Standard C++ (Mostly Unchanged) ---

void Basicmicro::crc_clear()
{
	crc = 0;
}

void Basicmicro::crc_update (uint8_t data)
{
	int i;
	crc = crc ^ ((uint16_t)data << 8);
	for (i=0; i<8; i++)
	{
		if (crc & 0x8000)
			crc = (crc << 1) ^ 0x1021;
		else
			crc <<= 1;
	}
}

uint16_t Basicmicro::crc_get()
{
	return crc;
}

int16_t Basicmicro::_read(void)
{
    int16_t data = read(timeout);
    if (data != -1) {
        crc_update((uint8_t)data);
    }
    return data;
}

bool Basicmicro::_checkcrc(void)
{
    int16_t data;

    // Read MSB of CRC
    data = _read();
    if (data == -1) {
        return false;
    }

    // Read LSB of CRC
    data = _read();
    if (data == -1) {
        return false;
    }

    return crc_get() == 0;
}

bool Basicmicro::write_n(uint8_t cnt, ... )
{
	uint8_t trys=MAXRETRY;
	do{
		clear();
		crc_clear();
		//send data with crc
		va_list marker;
		va_start( marker, cnt );
		for(uint8_t index=0;index<cnt;index++){
			uint8_t data = va_arg(marker, int);
			_write(data);
		}
		va_end( marker );
		if(_writechecksum()){
			return true;
		}
	}while(trys--);
	return false;
}

void Basicmicro::write_address_cmd(uint8_t address,uint8_t cmd)
{
	clear();
	crc_clear();
	_write(address);
	_write(cmd);
}

void Basicmicro::_write(uint8_t val)
{
    write(val);
    crc_update(val);
}

void Basicmicro::_writeword(uint16_t val)
{
    _write((uint8_t)(val >> 8));
    _write((uint8_t)(val & 0xFF));
}

void Basicmicro::_writelong(uint32_t val)
{
    _writeword((uint16_t)(val >> 16));
    _writeword((uint16_t)(val & 0xFFFF));
}

bool Basicmicro::_writechecksum()
{
    uint16_t checksum = crc_get();
    write(checksum >> 8);
    write(checksum & 0xFF);

    // Wait for ACK (0xFF)
    return read(timeout) == 0xFF;
}

bool Basicmicro::ReadByte(uint8_t &value)
{
    int16_t data;

    data = _read(); 
    if(data == -1) return false;

    value = (uint8_t)data;
    return true;
}

bool Basicmicro::ReadWord(uint16_t &value)
{
    int16_t data;
    uint16_t val = 0;

    data = _read(); 
    if(data == -1) return false;
    val = (uint16_t)data << 8;

    data = _read();
    if(data == -1) return false;
    val |= (uint16_t)data;

    value = val;
    return true;
}

bool Basicmicro::ReadLong(uint32_t &value)
{
    int16_t data;
    uint32_t val = 0;

    data = _read(); 
    if(data == -1) return false;
    val = (uint32_t)data << 24;

    data = _read();
    if(data == -1) return false;
    val |= (uint32_t)data << 16;

    data = _read(); 
    if(data == -1) return false;
    val |= (uint32_t)data << 8;

    data = _read(); 
    if(data == -1) return false;
    val |= (uint32_t)data;

    value = val;
    return true;
}

bool Basicmicro::read_n(uint8_t cnt,uint8_t address,uint8_t cmd,...)
{
	uint8_t trys=MAXRETRY;
	do{
		write_address_cmd(address,cmd);

		va_list marker;
		va_start( marker, cmd );

		bool read_ok = true;
		for(uint8_t index=0;index<cnt;index++){
			uint32_t *ptr = va_arg(marker, uint32_t *);
            uint32_t value;

			if (!ReadLong(value)) {
                read_ok = false;
                break;
            }
            *ptr = value;
		}
		va_end( marker ); 

		if(read_ok && _checkcrc()) {
            return true;
		}

	}while(trys--); 

	return false; 
}

bool Basicmicro::read_n_words(uint8_t cnt,uint8_t address,uint8_t cmd,...)
{
	uint8_t trys=MAXRETRY;
	do{
		write_address_cmd(address,cmd); 

		va_list marker;
		va_start( marker, cmd );

		bool read_ok = true;
		for(uint8_t index=0;index<cnt;index++){
			uint16_t *ptr = va_arg(marker, uint16_t *);
            uint16_t value;

			if (!ReadWord(value)) {
                read_ok = false; 
                break;
            }
            *ptr = value; 
		}
		va_end( marker );  

		if(read_ok && _checkcrc()) {
            return true; 
		}

	}while(trys--); 

	return false; 
}

bool Basicmicro::read_n_bytes(uint8_t cnt, uint8_t address, uint8_t cmd, ...)
{
    uint8_t trys = MAXRETRY;
    do {
        write_address_cmd(address, cmd); 

        va_list marker;
        va_start(marker, cmd);   

        bool read_ok = true;
        for (uint8_t index = 0; index < cnt; index++) {
            uint8_t *ptr = va_arg(marker, uint8_t *);
            uint8_t value;

            if (!ReadByte(value)) {
                read_ok = false;
                break;
            }
            *ptr = value;
        }
        va_end(marker); 

        if (read_ok && _checkcrc()) {
            return true; 
        }

    } while (trys--); 

    return false;
}

uint8_t Basicmicro::Read1(uint8_t address,uint8_t cmd,bool *valid)
{
	if(valid)
		*valid = false;

	uint8_t value=0;
	uint8_t trys=MAXRETRY;
	do{
		write_address_cmd(address,cmd); 

		if (!ReadByte(value)) {
            continue; 
        }

		if(_checkcrc()){
			if(valid)
				*valid = true;
			return value;
		}

	}while(trys--); 

	return 0; 
}

uint16_t Basicmicro::Read2(uint8_t address,uint8_t cmd,bool *valid)
{
	if(valid)
		*valid = false;

	uint16_t value=0;
	uint8_t trys=MAXRETRY;
	do{
		write_address_cmd(address,cmd);

		if (!ReadWord(value)) {
            continue;
        }

		if(_checkcrc()){
			if(valid)
				*valid = true;
			return value;
		}

	}while(trys--); 

	return 0; 
}

uint32_t Basicmicro::Read4(uint8_t address, uint8_t cmd, bool *valid)
{
	if(valid)
		*valid = false;

	uint32_t value=0;
	uint8_t trys=MAXRETRY;
	do{
		write_address_cmd(address,cmd); 

		if (!ReadLong(value)) {
            continue; 
        }

		if(_checkcrc()){
			if(valid)
				*valid = true;
			return value;
		}

	}while(trys--);

	return 0; 
}

uint32_t Basicmicro::Read4_1(uint8_t address, uint8_t cmd, uint8_t *status, bool *valid)
{
	if(valid)
		*valid = false;

	uint32_t value=0;
	uint8_t trys=MAXRETRY;
	uint8_t temp_status = 0;

	do{
		write_address_cmd(address,cmd);

		if (!ReadLong(value)) {
            continue; 
        }

		if (!ReadByte(temp_status)) {
            continue; 
        }

		if(_checkcrc()){
			if(status)
				*status = temp_status;
			if(valid)
				*valid = true;
			return value;
		}

	}while(trys--); 

	return 0; 
}

// --- Compatibility & Command Functions ---

bool Basicmicro::SetPWM(uint8_t address, uint8_t motor, int16_t value, int32_t accel, uint16_t range)
{
	if (range == 0) return false;

    if (value > (int16_t)range) value = (int16_t)range;
    if (value < -(int16_t)range) value = -(int16_t)range;

    int32_t duty_32 = ((int32_t)value * 0x7FFF) / range;

	if(!motor)
		return DutyAccelM1(address, duty_32, accel); 
	else
		return DutyAccelM2(address, duty_32, accel); 
}

bool Basicmicro::ST_Single(uint8_t cmd,uint8_t address, uint8_t speed)
{
	ST_Power = ST_NOT_SET;
	ST_Turn = ST_NOT_SET;

	uint8_t power = speed & 0x7F;

    int16_t duty_value;
    uint16_t duty_range;
    uint8_t motor;

    switch(cmd) {
        case M1FORWARD: motor = 0; duty_value = power; duty_range = 127; break;
        case M1BACKWARD: motor = 0; duty_value = -power; duty_range = 127; break;
        case M2FORWARD: motor = 1; duty_value = power; duty_range = 127; break;
        case M2BACKWARD: motor = 1; duty_value = -power; duty_range = 127; break;
        case M17BIT: motor = 0; duty_value = power - 64; duty_range = 63; break; 
        case M27BIT: motor = 1; duty_value = power - 64; duty_range = 63; break;
        default: return false; 
    }

	return SetPWM(address, motor, duty_value, 0, duty_range);
}

bool Basicmicro::ForwardM1(uint8_t address, uint8_t speed)
{
	return ST_Single(M1FORWARD,address,speed);
}
bool Basicmicro::BackwardM1(uint8_t address, uint8_t speed)
{
	return ST_Single(M1BACKWARD,address,speed);
}
bool Basicmicro::ForwardM2(uint8_t address, uint8_t speed)
{
	return ST_Single(M2FORWARD,address,speed);
}
bool Basicmicro::BackwardM2(uint8_t address, uint8_t speed)
{
	return ST_Single(M2BACKWARD,address,speed);
}
bool Basicmicro::ForwardBackwardM1(uint8_t address, uint8_t speed)
{
	return ST_Single(M17BIT,address,speed);
}
bool Basicmicro::ForwardBackwardM2(uint8_t address, uint8_t speed)
{
	return ST_Single(M27BIT,address,speed);
}

bool Basicmicro::SetPWM2(uint8_t address, int16_t value1, int32_t accel1, int16_t value2, int32_t accel2, uint16_t range)
{
	if (range == 0) return false;

    if (value1 > (int16_t)range) value1 = (int16_t)range;
    if (value1 < -(int16_t)range) value1 = -(int16_t)range;

    if (value2 > (int16_t)range) value2 = (int16_t)range;
    if (value2 < -(int16_t)range) value2 = -(int16_t)range;

    int32_t duty1_32 = ((int32_t)value1 * 0x7FFF) / range;
    int32_t duty2_32 = ((int32_t)value2 * 0x7FFF) / range;

	return DutyAccelM1M2(address, duty1_32, accel1, duty2_32, accel2);
}

bool Basicmicro::ST_Mixed(uint8_t cmd, uint8_t address, uint8_t speed)
{
	uint8_t raw_speed = speed & 0x7F;
    int16_t current_signed_value;

    switch (cmd) {
        case MIXEDFORWARD:
            current_signed_value = raw_speed;
            ST_Power = current_signed_value;
            break;
        case MIXEDBACKWARD:
            current_signed_value = -raw_speed;
            ST_Power = current_signed_value;
            break;
        case MIXEDRIGHT:
            current_signed_value = raw_speed;
            ST_Turn = current_signed_value; 
            break;
        case MIXEDLEFT:
            current_signed_value = -raw_speed;
            ST_Turn = current_signed_value; 
            break;
        case MIXEDFB:
            current_signed_value = raw_speed - 64;
             ST_Power = current_signed_value; 
            break;
        case MIXEDLR: 
            current_signed_value = raw_speed - 64;
             ST_Turn = current_signed_value; 
            break;
        default:
            ST_Power = ST_NOT_SET;
            ST_Turn = ST_NOT_SET;
            return false;
    }

	if(ST_Power != ST_NOT_SET && ST_Turn != ST_NOT_SET){
		int16_t motor1_duty = ST_Power + ST_Turn;
		int16_t motor2_duty = ST_Power - ST_Turn;

        motor1_duty = constrain(motor1_duty, -127, 127);
        motor2_duty = constrain(motor2_duty, -127, 127);

		return SetPWM2(address, motor1_duty, 0, motor2_duty, 0, 127);
	}

	return true;
}

bool Basicmicro::ForwardMixed(uint8_t address, uint8_t speed)
{
	ST_Power = speed & 0x7F;
	return ST_Mixed(MIXEDFORWARD, address, speed); 
}
bool Basicmicro::BackwardMixed(uint8_t address, uint8_t speed)
{
	ST_Power = speed & 0x7F;
	return ST_Mixed(MIXEDBACKWARD, address, speed);
}
bool Basicmicro::TurnRightMixed(uint8_t address, uint8_t speed)
{
	ST_Turn = speed & 0x7F;
	return ST_Mixed(MIXEDRIGHT, address, speed);
}
bool Basicmicro::TurnLeftMixed(uint8_t address, uint8_t speed)
{
	ST_Turn = speed & 0x7F;
	return ST_Mixed(MIXEDLEFT, address, speed);
}
bool Basicmicro::ForwardBackwardMixed(uint8_t address, uint8_t speed)
{
	ST_Power = speed & 0x7F;
	return ST_Mixed(MIXEDFB, address, speed);
}
bool Basicmicro::LeftRightMixed(uint8_t address, uint8_t speed)
{
	ST_Turn = speed & 0x7F;
	return ST_Mixed(MIXEDLR, address, speed);
}

// --- Basicmicro Commands (Direct Copy) ---

uint32_t Basicmicro::ReadEncM1(uint8_t address, uint8_t *status,bool *valid)
{
	return Read4_1(address,GETM1ENC,status,valid);
}

uint32_t Basicmicro::ReadEncM2(uint8_t address, uint8_t *status,bool *valid)
{
	return Read4_1(address,GETM2ENC,status,valid);
}

uint32_t Basicmicro::ReadSpeedM1(uint8_t address, uint8_t *status,bool *valid)
{
	return Read4_1(address,GETM1SPEED,status,valid);
}

uint32_t Basicmicro::ReadSpeedM2(uint8_t address, uint8_t *status,bool *valid)
{
	return Read4_1(address,GETM2SPEED,status,valid);
}

bool Basicmicro::ResetEncoders(uint8_t address)
{
	return write_n(2,address,RESETENC);
}

bool Basicmicro::ReadVersion(uint8_t address,char *version)
{
	uint8_t trys=MAXRETRY;

	if(!version) return false;

	do{
		write_address_cmd(address, GETVERSION);

		bool read_ok = true;
		uint8_t i;
		for(i=0;i<48;i++){
            int16_t data = _read();
			if(data != -1){
				version[i] = (uint8_t)data;
				if(version[i]==0){ 
					i++;
					break;
				}
			}
			else{
				read_ok = false; 
				break;
			}
		}
        if (i < 48) version[i] = 0;
        else version[47] = 0; 

		if(read_ok && _checkcrc()){
			return true;
		}

	}while(trys--); 

	version[0] = 0;
	return false; 
}

bool Basicmicro::SetEncM1(uint8_t address, int32_t val)
{
	return write_n(6,address,SETM1ENCCOUNT,SetDWORDval(val));
}

bool Basicmicro::SetEncM2(uint8_t address, int32_t val)
{
	return write_n(6,address,SETM2ENCCOUNT,SetDWORDval(val));
}

uint16_t Basicmicro::ReadMainBatteryVoltage(uint8_t address,bool *valid)
{
	return Read2(address,GETMBATT,valid);
}

uint16_t Basicmicro::ReadLogicBatteryVoltage(uint8_t address,bool *valid)
{
	return Read2(address,GETLBATT,valid);
}

bool Basicmicro::SetM1VelocityPID(uint8_t address, float kp_fp, float ki_fp, float kd_fp, uint32_t qpps)
{
	uint32_t kp = (uint32_t)(kp_fp * 65536.0f);
	uint32_t ki = (uint32_t)(ki_fp * 65536.0f);
	uint32_t kd = (uint32_t)(kd_fp * 65536.0f);
	return write_n(18,address,SETM1PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}

bool Basicmicro::SetM2VelocityPID(uint8_t address, float kp_fp, float ki_fp, float kd_fp, uint32_t qpps)
{
	uint32_t kp = (uint32_t)(kp_fp * 65536.0f);
	uint32_t ki = (uint32_t)(ki_fp * 65536.0f);
	uint32_t kd = (uint32_t)(kd_fp * 65536.0f);
	return write_n(18,address,SETM2PID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(qpps));
}

uint32_t Basicmicro::ReadISpeedM1(uint8_t address,uint8_t *status,bool *valid)
{
	return Read4_1(address,GETM1ISPEED,status,valid);
}

uint32_t Basicmicro::ReadISpeedM2(uint8_t address,uint8_t *status,bool *valid)
{
	return Read4_1(address,GETM2ISPEED,status,valid);
}

bool Basicmicro::DutyM1(uint8_t address, uint16_t duty) 
{
	return write_n(4,address,M1DUTY,SetWORDval(duty)); 
}

bool Basicmicro::DutyM2(uint8_t address, uint16_t duty) 
{
	return write_n(4,address,M2DUTY,SetWORDval(duty));
}

bool Basicmicro::DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2) 
{
	return write_n(6,address,MIXEDDUTY,SetWORDval(duty1),SetWORDval(duty2)); 
}

bool Basicmicro::SpeedM1(uint8_t address, uint32_t speed) 
{
	return write_n(6,address,M1SPEED,SetDWORDval(speed)); 
}

bool Basicmicro::SpeedM2(uint8_t address, uint32_t speed) 
{
	return write_n(6,address,M2SPEED,SetDWORDval(speed)); 
}

bool Basicmicro::SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2) 
{
	return write_n(10,address,MIXEDSPEED,SetDWORDval(speed1),SetDWORDval(speed2)); 
}

bool Basicmicro::SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed) 
{
	return write_n(10,address,M1SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed)); 
}

bool Basicmicro::SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed)
{
	return write_n(10,address,M2SPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed)); 
}

bool Basicmicro::SpeedAccelM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2) 
{
	return write_n(14,address,MIXEDSPEEDACCEL,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(speed2)); 
}

bool Basicmicro::SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag) 
{
	return write_n(11,address,M1SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag);
}

bool Basicmicro::SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag) 
{
	return write_n(11,address,M2SPEEDDIST,SetDWORDval(speed),SetDWORDval(distance),flag); 
}

bool Basicmicro::SpeedDistanceM1M2(uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag) 
{
	return write_n(19,address,MIXEDSPEEDDIST,SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag); 
}

bool Basicmicro::SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag)
{
	return write_n(15,address,M1SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag); 
}

bool Basicmicro::SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag) 
{
	return write_n(15,address,M2SPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(distance),flag); 
}

bool Basicmicro::SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag) 
{
	return write_n(23,address,MIXEDSPEEDACCELDIST,SetDWORDval(accel),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(speed2),SetDWORDval(distance2),flag); 
}

bool Basicmicro::ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2)
{
	return read_n_bytes(2, address, GETBUFFERS, &depth1, &depth2);
}

bool Basicmicro::ReadPWMs(uint8_t address, int16_t &pwm1, int16_t &pwm2)
{
	bool valid = false;
	uint32_t value = Read4(address,GETPWMS,&valid);
	if(valid){
		pwm1 = (int16_t)(value>>16);
		pwm2 = (int16_t)(value&0xFFFF);
	}
	return valid;
}

bool Basicmicro::ReadCurrents(uint8_t address, int16_t &current1, int16_t &current2)
{
	bool valid = false;
	uint32_t value = Read4(address,GETCURRENTS,&valid);
	if(valid){
		current1 = (int16_t)(value>>16);
		current2 = (int16_t)(value&0xFFFF);
	}
	return valid;
}

bool Basicmicro::SpeedAccelM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2) 
{
	return write_n(18,address,MIXEDSPEED2ACCEL,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(accel2),SetDWORDval(speed2)); 
}

bool Basicmicro::SpeedAccelDistanceM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag) 
{
	return write_n(27,address,MIXEDSPEED2ACCELDIST,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(distance1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(distance2),flag); 
}

bool Basicmicro::DutyAccelM1(uint8_t address, uint16_t duty, uint32_t accel) 
{
	return write_n(8,address,M1DUTYACCEL,SetWORDval(duty),SetDWORDval(accel)); 
}

bool Basicmicro::DutyAccelM2(uint8_t address, uint16_t duty, uint32_t accel) 
{
	return write_n(8,address,M2DUTYACCEL,SetWORDval(duty),SetDWORDval(accel));
}

bool Basicmicro::DutyAccelM1M2(uint8_t address, uint16_t duty1, uint32_t accel1, uint16_t duty2, uint32_t accel2) 
{
	return write_n(14,address,MIXEDDUTYACCEL,SetWORDval(duty1),SetDWORDval(accel1),SetWORDval(duty2),SetDWORDval(accel2)); 
}

bool Basicmicro::ReadM1VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps)
{
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(4,address,READM1PID,&Kp,&Ki,&Kd,&qpps); 
	if(valid){
		Kp_fp = ((float)Kp)/65536.0f;
		Ki_fp = ((float)Ki)/65536.0f;
		Kd_fp = ((float)Kd)/65536.0f;
	}
	return valid;
}

bool Basicmicro::ReadM2VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps)
{
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(4,address,READM2PID,&Kp,&Ki,&Kd,&qpps); 
	if(valid){
		Kp_fp = ((float)Kp)/65536.0f;
		Ki_fp = ((float)Ki)/65536.0f;
		Kd_fp = ((float)Kd)/65536.0f;
	}
	return valid;
}

bool Basicmicro::SetMainVoltages(uint8_t address,uint16_t min,uint16_t max,uint8_t autoMax)
{
	return write_n(7,address,SETMAINVOLTAGES,SetWORDval(min),SetWORDval(max),autoMax);
}

bool Basicmicro::SetLogicVoltages(uint8_t address,uint16_t min,uint16_t max){
	return write_n(6,address,SETLOGICVOLTAGES,SetWORDval(min),SetWORDval(max));
}

bool Basicmicro::ReadMinMaxMainVoltages(uint8_t address,uint16_t &min,uint16_t &max,uint8_t& autoMax)
{
	bool valid = false;
	uint8_t temp_autoMax; 
	uint32_t value =  Read4_1(address,GETMINMAXMAINVOLTAGES,&temp_autoMax,&valid);
	if(valid){
		min = (uint16_t)(value>>16);
		max = (uint16_t)(value&0xFFFF);
		autoMax = temp_autoMax; 
	}
	return valid;
}

bool Basicmicro::ReadMinMaxLogicVoltages(uint8_t address,uint16_t &min,uint16_t &max)
{
	bool valid = false;
	uint32_t value = Read4(address,GETMINMAXLOGICVOLTAGES,&valid);
	if(valid){
		min = (uint16_t)(value>>16);
		max = (uint16_t)(value&0xFFFF);
	}
	return valid;
}

bool Basicmicro::SetM1PositionPID(uint8_t address,float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max)
{
	uint32_t kp=(uint32_t)(kp_fp*1024.0f);
	uint32_t ki=(uint32_t)(ki_fp*1024.0f);
	uint32_t kd=(uint32_t)(kd_fp*1024.0f);
	return write_n(30,address,SETM1POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max)); 
}

bool Basicmicro::SetM2PositionPID(uint8_t address,float kp_fp,float ki_fp,float kd_fp,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max)
{
	uint32_t kp=(uint32_t)(kp_fp*1024.0f);
	uint32_t ki=(uint32_t)(ki_fp*1024.0f);
	uint32_t kd=(uint32_t)(kd_fp*1024.0f);
	return write_n(30,address,SETM2POSPID,SetDWORDval(kd),SetDWORDval(kp),SetDWORDval(ki),SetDWORDval(kiMax),SetDWORDval(deadzone),SetDWORDval(min),SetDWORDval(max)); 
}

bool Basicmicro::ReadM1PositionPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max)
{
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(7,address,READM1POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max); 
	if(valid){
		Kp_fp = ((float)Kp)/1024.0f;
		Ki_fp = ((float)Ki)/1024.0f;
		Kd_fp = ((float)Kd)/1024.0f;
	}
	return valid;
}

bool Basicmicro::ReadM2PositionPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max)
{
	uint32_t Kp,Ki,Kd;
	bool valid = read_n(7,address,READM2POSPID,&Kp,&Ki,&Kd,&KiMax,&DeadZone,&Min,&Max); 
	if(valid){
		Kp_fp = ((float)Kp)/1024.0f;
		Ki_fp = ((float)Ki)/1024.0f;
		Kd_fp = ((float)Kd)/1024.0f;
	}
	return valid;
}

bool Basicmicro::SpeedAccelDeccelPositionM1(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag) 
{
	return write_n(19,address,M1SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag); 
}

bool Basicmicro::SpeedAccelDeccelPositionM2(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag) 
{
	return write_n(19,address,M2SPEEDACCELDECCELPOS,SetDWORDval(accel),SetDWORDval(speed),SetDWORDval(deccel),SetDWORDval(position),flag);
}

bool Basicmicro::SpeedAccelDeccelPositionM1M2(uint8_t address,uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag)
{
	return write_n(35,address,MIXEDSPEEDACCELDECCELPOS,SetDWORDval(accel1),SetDWORDval(speed1),SetDWORDval(deccel1),SetDWORDval(position1),SetDWORDval(accel2),SetDWORDval(speed2),SetDWORDval(deccel2),SetDWORDval(position2),flag); 
}

bool Basicmicro::SetM1DefaultAccel(uint8_t address, uint32_t accel, uint32_t decel)
{
	return write_n(10,address,SETM1DEFAULTACCEL,SetDWORDval(accel),SetDWORDval(decel)); 
}

bool Basicmicro::SetM2DefaultAccel(uint8_t address, uint32_t accel, uint32_t decel)
{
	return write_n(10,address,SETM2DEFAULTACCEL,SetDWORDval(accel),SetDWORDval(decel));
}

bool Basicmicro::GetDefaultAccels(uint8_t address, uint32_t &accelM1, uint32_t &decelM1, uint32_t &accelM2, uint32_t &decelM2)
{
	return read_n(4,address,GETDEFAULTACCELS,&accelM1,&decelM1,&accelM2,&decelM2); 
}

bool Basicmicro::SetPinFunctions(uint8_t address, uint8_t S3mode, uint8_t S4mode, uint8_t S5mode, uint8_t D1mode, uint8_t D2mode)
{
	return write_n(7,address,SETPINFUNCTIONS,S3mode,S4mode,S5mode,D1mode,D2mode); 
}

bool Basicmicro::GetPinFunctions(uint8_t address, uint8_t &S3mode, uint8_t &S4mode, uint8_t &S5mode, uint8_t &D1mode, uint8_t &D2mode)
{
    return read_n_bytes(5, address, GETPINFUNCTIONS, &S3mode, &S4mode, &S5mode, &D1mode, &D2mode); 
}

bool Basicmicro::SetCtrlSettings(uint8_t address,
                              uint8_t minDBM1, uint8_t maxDBM1,
                              uint16_t minLimitsM1, uint16_t maxLimitsM1,
                              uint16_t centerM1, uint16_t minM1, uint16_t maxM1,
                              uint8_t minDBM2, uint8_t maxDBM2,
                              uint16_t minLimitsM2, uint16_t maxLimitsM2,
                              uint16_t centerM2, uint16_t minM2, uint16_t maxM2)
{
    return write_n(26, address, SETCTRLSETTINGS,
                 minDBM1, maxDBM1, 
                 SetWORDval(minLimitsM1), SetWORDval(maxLimitsM1), 
                 SetWORDval(centerM1), SetWORDval(minM1), SetWORDval(maxM1), 
                 minDBM2, maxDBM2, 
                 SetWORDval(minLimitsM2), SetWORDval(maxLimitsM2), 
                 SetWORDval(centerM2), SetWORDval(minM2), SetWORDval(maxM2)); 
}

bool Basicmicro::GetCtrlSettings(uint8_t address,
                              uint8_t &minDBM1, uint8_t &maxDBM1,
                              uint16_t &minLimitsM1, uint16_t &maxLimitsM1,
                              uint16_t &centerM1, uint16_t &minM1, uint16_t &maxM1,
                              uint8_t &minDBM2, uint8_t &maxDBM2,
                              uint16_t &minLimitsM2, uint16_t &maxLimitsM2,
                              uint16_t &centerM2, uint16_t &minM2, uint16_t &maxM2)
{
    uint8_t trys = MAXRETRY;

    do {
        write_address_cmd(address, GETCTRLSETTINGS);

        bool read_ok = true;

        uint8_t temp_minDBM1, temp_maxDBM1;
        if (!ReadByte(temp_minDBM1)) { read_ok = false; }
        if (read_ok && !ReadByte(temp_maxDBM1)) { read_ok = false; }
        if (!read_ok) continue;

        uint16_t temp_minLimitsM1, temp_maxLimitsM1, temp_centerM1, temp_minM1, temp_maxM1;
        if (read_ok && !ReadWord(temp_minLimitsM1)) { read_ok = false; }
        if (read_ok && !ReadWord(temp_maxLimitsM1)) { read_ok = false; }
        if (read_ok && !ReadWord(temp_centerM1)) { read_ok = false; }
        if (read_ok && !ReadWord(temp_minM1)) { read_ok = false; }
        if (read_ok && !ReadWord(temp_maxM1)) { read_ok = false; }
        if (!read_ok) continue;

        uint8_t temp_minDBM2, temp_maxDBM2;
        if (read_ok && !ReadByte(temp_minDBM2)) { read_ok = false; }
        if (read_ok && !ReadByte(temp_maxDBM2)) { read_ok = false; }
        if (!read_ok) continue;

        uint16_t temp_minLimitsM2, temp_maxLimitsM2, temp_centerM2, temp_minM2, temp_maxM2;
        if (read_ok && !ReadWord(temp_minLimitsM2)) { read_ok = false; }
        if (read_ok && !ReadWord(temp_maxLimitsM2)) { read_ok = false; }
        if (read_ok && !ReadWord(temp_centerM2)) { read_ok = false; }
        if (read_ok && !ReadWord(temp_minM2)) { read_ok = false; }
        if (read_ok && !ReadWord(temp_maxM2)) { read_ok = false; }
        if (!read_ok) continue;

        if (read_ok && _checkcrc()) {
            minDBM1 = temp_minDBM1;
            maxDBM1 = temp_maxDBM1;
            minLimitsM1 = temp_minLimitsM1;
            maxLimitsM1 = temp_maxLimitsM1;
            centerM1 = temp_centerM1;
            minM1 = temp_minM1;
            maxM1 = temp_maxM1;
            minDBM2 = temp_minDBM2;
            maxDBM2 = temp_maxDBM2;
            minLimitsM2 = temp_minLimitsM2;
            maxLimitsM2 = temp_maxLimitsM2;
            centerM2 = temp_centerM2;
            minM2 = temp_minM2;
            maxM2 = temp_maxM2;
            return true;
        }

    } while(trys--); 

    return false;
}

bool Basicmicro::ReadEncoders(uint8_t address,uint32_t &enc1,uint32_t &enc2)
{
	return read_n(2,address,GETENCODERS,&enc1,&enc2); 
}

bool Basicmicro::ReadISpeeds(uint8_t address,uint32_t &ispeed1,uint32_t &ispeed2)
{
	return read_n(2,address,GETISPEEDS,&ispeed1,&ispeed2);
}

bool Basicmicro::RestoreDefaults(uint8_t address)
{
	return write_n(6,address,RESTOREDEFAULTS,SetDWORDval(0xE22EAB7A));
}

bool Basicmicro::ReadTemp(uint8_t address, uint16_t &temp)
{
	bool valid = false;
	uint16_t value = Read2(address,GETTEMP,&valid);
	if(valid){
		temp = value;
	}
	return valid;
}

bool Basicmicro::ReadTemp2(uint8_t address, uint16_t &temp)
{
	bool valid = false;
	uint16_t value = Read2(address,GETTEMP2,&valid);
	if(valid){
		temp = value;
	}
	return valid;
}

uint32_t Basicmicro::ReadError(uint8_t address,bool *valid)
{
	return Read4(address,GETERROR,valid);
}

bool Basicmicro::ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode)
{
	return read_n_bytes(2, address, GETENCODERMODE, &M1mode, &M2mode);
}

bool Basicmicro::SetM1EncoderMode(uint8_t address,uint8_t mode)
{
	return write_n(3,address,SETM1ENCODERMODE,mode);
}

bool Basicmicro::SetM2EncoderMode(uint8_t address,uint8_t mode)
{
	return write_n(3,address,SETM2ENCODERMODE,mode); 
}

bool Basicmicro::WriteNVM(uint8_t address)
{
	return write_n(6,address,WRITENVM, SetDWORDval(0xE22EAB7A) );
}

bool Basicmicro::ReadNVM(uint8_t address)
{
	return write_n(2,address,READNVM); 
}

bool Basicmicro::SetConfig(uint8_t address, uint16_t config)
{
	return write_n(4,address,SETCONFIG,SetWORDval(config));
}

bool Basicmicro::GetConfig(uint8_t address, uint16_t &config)
{
	bool valid = false;
	uint16_t value = Read2(address,GETCONFIG,&valid);
	if(valid){
		config = value;
	}
	return valid;
}

bool Basicmicro::SetM1MaxCurrent(uint8_t address,uint32_t max,uint32_t min)
{
	return write_n(10,address,SETM1MAXCURRENT,SetDWORDval(max),SetDWORDval(min)); 
}

bool Basicmicro::SetM2MaxCurrent(uint8_t address,uint32_t max,uint32_t min)
{
	return write_n(10,address,SETM2MAXCURRENT,SetDWORDval(max),SetDWORDval(min)); 
}

bool Basicmicro::ReadM1MaxCurrent(uint8_t address,uint32_t &max,uint32_t&min)
{
	uint32_t tmax,tmin;
	bool valid = read_n(2,address,GETM1MAXCURRENT,&tmax,&tmin); 
	if(valid){
		max = tmax;
		min = tmin;
	}
	return valid;
}

bool Basicmicro::ReadM2MaxCurrent(uint8_t address,uint32_t &max,uint32_t&min)
{
	uint32_t tmax,tmin;
	bool valid = read_n(2,address,GETM2MAXCURRENT,&tmax,&tmin); 
	if(valid){
		max = tmax;
		min = tmin;
	}
	return valid;
}

bool Basicmicro::SetPWMMode(uint8_t address, uint8_t modeM1, uint8_t modeM2)
{
	return write_n(4,address,SETPWMMODE,modeM1,modeM2); 
}

bool Basicmicro::GetPWMMode(uint8_t address, uint8_t &modeM1, uint8_t &modeM2)
{
	bool valid = false;
	uint16_t value = Read2(address,GETPWMMODE,&valid); 
	if(valid){
		modeM1 = value>>8;
		modeM2 = value&0xFF; 
	}
	return valid;
}

bool Basicmicro::SetAUXDutys(uint8_t address, uint16_t S3duty, uint16_t S4duty, uint16_t S5duty, uint16_t CTRL1duty, uint16_t CTRL2duty)
{
	return write_n(12,address,SETAUXDUTYS,SetWORDval(S3duty),SetWORDval(S4duty),SetWORDval(S5duty),SetWORDval(CTRL1duty),SetWORDval(CTRL2duty)); 
}

bool Basicmicro::GetAUXDutys(uint8_t address, uint16_t &S3duty, uint16_t &S4duty, uint16_t &S5duty, uint16_t &CTRL1duty, uint16_t &CTRL2duty)
{
    return read_n_words(5, address, GETAUXDUTYS, &S3duty, &S4duty, &S5duty, &CTRL1duty, &CTRL2duty); 
}

bool Basicmicro::SetTimeout(uint8_t address, float timeout)
{
    if (timeout < 0.0f) timeout = 0.0f;
    if (timeout > 25.5f) timeout = 25.5f;
    uint8_t value = (uint8_t)(timeout * 10.0f);
    return write_n(3, address, SETTIMEOUT, value); 
}

bool Basicmicro::GetTimeout(uint8_t address, float &timeout)
{
    bool valid = false;
    uint8_t value = Read1(address, GETTIMEOUT, &valid);
    if(valid) {
        timeout = value / 10.0f;
        return true;
    }
    return false;
}

bool Basicmicro::SetM1DefaultSpeed(uint8_t address, uint16_t speed) 
{
    return write_n(4, address, SETM1DEFAULTSPEED, SetWORDval(speed));
}

bool Basicmicro::SetM2DefaultSpeed(uint8_t address, uint16_t speed)
{
    return write_n(4, address, SETM2DEFAULTSPEED, SetWORDval(speed));
}

bool Basicmicro::GetDefaultSpeeds(uint8_t address, uint16_t &speed1, uint16_t &speed2)
{
    return read_n_words(2, address, GETDEFAULTSPEEDS, &speed1, &speed2);
}

bool Basicmicro::GetStatus(uint8_t address, uint32_t &tick, uint32_t &state,
                          uint16_t &temp1, uint16_t &temp2,
                          uint16_t &mainBattVoltage, uint16_t &logicBattVoltage,
                          int16_t &pwm1, int16_t &pwm2, int16_t &current1, int16_t &current2,
                          uint32_t &enc1, uint32_t &enc2, uint32_t &speed1, uint32_t &speed2,
                          uint32_t &ispeed1, uint32_t &ispeed2, uint16_t &speedError1, uint16_t &speedError2,
                          uint16_t &posError1, uint16_t &posError2)
{
    uint8_t trys = MAXRETRY;
    uint16_t rawPwm1, rawPwm2, rawCurrent1, rawCurrent2; 

    do {
        write_address_cmd(address, GETSTATUS); 

        bool read_ok = true;

        if(!ReadLong(tick)) read_ok = false;
        if(read_ok && !ReadLong(state)) read_ok = false;

        if(read_ok && !ReadWord(temp1)) read_ok = false;
        if(read_ok && !ReadWord(temp2)) read_ok = false;
        if(read_ok && !ReadWord(mainBattVoltage)) read_ok = false;
        if(read_ok && !ReadWord(logicBattVoltage)) read_ok = false;

        if(read_ok && !ReadWord(rawPwm1)) read_ok = false; 
        if(read_ok && !ReadWord(rawPwm2)) read_ok = false; 
        if(read_ok && !ReadWord(rawCurrent1)) read_ok = false; 
        if(read_ok && !ReadWord(rawCurrent2)) read_ok = false; 

        if(read_ok && !ReadLong(enc1)) read_ok = false;
        if(read_ok && !ReadLong(enc2)) read_ok = false;
        if(read_ok && !ReadLong(speed1)) read_ok = false;
        if(read_ok && !ReadLong(speed2)) read_ok = false;
        if(read_ok && !ReadLong(ispeed1)) read_ok = false;
        if(read_ok && !ReadLong(ispeed2)) read_ok = false;

        if(read_ok && !ReadWord(speedError1)) read_ok = false;
        if(read_ok && !ReadWord(speedError2)) read_ok = false;
        if(read_ok && !ReadWord(posError1)) read_ok = false;
        if(read_ok && !ReadWord(posError2)) read_ok = false;

        if(!read_ok) continue;

        if(_checkcrc()) {
            pwm1 = (int16_t)rawPwm1;
            pwm2 = (int16_t)rawPwm2;
            current1 = (int16_t)rawCurrent1;
            current2 = (int16_t)rawCurrent2;
            return true;
        }

    } while(trys--); 

    return false;
}

bool Basicmicro::SetSerialNumber(uint8_t address, const char* serialNumber)
{
    uint8_t trys = MAXRETRY;
    uint8_t length = serialNumber ? strlen(serialNumber) : 0; 

    if(length > 36)
        length = 36;

    do {
        write_address_cmd(address, SETSERIALNUMBER);

        write(length);
        crc_update(length);

        for(uint8_t i = 0; i < 36; i++) {
            uint8_t c = (serialNumber && i < length) ? serialNumber[i] : 0; 
            write(c);
            crc_update(c);
        }

        if(_writechecksum()) {
            return true;
        }
    } while(trys--);

    return false;
}

bool Basicmicro::GetSerialNumber(uint8_t address, char* serialNumber)
{
    uint8_t trys = MAXRETRY;

    if(!serialNumber) return false;

    do {
        write_address_cmd(address, GETSERIALNUMBER); 

        bool read_ok = true;

        uint8_t length;
        if (!ReadByte(length)) { read_ok = false; }
        if (!read_ok) continue;

        uint8_t buffer[36];
        for(uint8_t i = 0; i < 36; i++) {
            int16_t data = _read();
            if(data == -1) {
                read_ok = false; 
                break;
            }
            buffer[i] = (uint8_t)data;
        }
        if (!read_ok) continue;


        if(_checkcrc()) {
            uint8_t copy_len = (length <= 36) ? length : 36;
            for(uint8_t i = 0; i < copy_len; i++) {
                serialNumber[i] = buffer[i];
            }
            serialNumber[copy_len] = 0; 

            return true;
        }
    } while(trys--); 

    if(serialNumber)
        serialNumber[0] = 0;

    return false; 
}

bool Basicmicro::GetVolts(uint8_t address, uint16_t &mainBattVoltage, uint16_t &logicBattVoltage)
{
	return read_n_words(2, address, GETVOLTS, &mainBattVoltage, &logicBattVoltage);
}

bool Basicmicro::GetTemps(uint8_t address, uint16_t &temp1, uint16_t &temp2)
{
	return read_n_words(2, address, GETTEMPS, &temp1, &temp2);
}

bool Basicmicro::GetEncStatus(uint8_t address, uint8_t &enc1Status, uint8_t &enc2Status)
{
    return read_n_bytes(2, address, GETENCSTATUS, &enc1Status, &enc2Status); 
}

bool Basicmicro::SetAuto1(uint8_t address, uint32_t value)
{
    return write_n(6, address, SETAUTO1, SetDWORDval(value)); 
}

bool Basicmicro::SetAuto2(uint8_t address, uint32_t value)
{
    return write_n(6, address, SETAUTO2, SetDWORDval(value));
}

bool Basicmicro::GetAutos(uint8_t address, uint32_t &auto1, uint32_t &auto2)
{
    return read_n(2, address, GETAUTOS, &auto1, &auto2);
}

bool Basicmicro::GetSpeeds(uint8_t address, uint32_t &speed1, uint32_t &speed2)
{
    return read_n(2, address, GETSPEEDS, &speed1, &speed2);
}

bool Basicmicro::SetSpeedErrorLimit(uint8_t address, uint16_t limit1, uint16_t limit2)
{
    return write_n(6, address, SETSPEEDERRORLIMIT, SetWORDval(limit1), SetWORDval(limit2)); 
}

bool Basicmicro::GetSpeedErrorLimit(uint8_t address, uint16_t &limit1, uint16_t &limit2)
{
    return read_n_words(2, address, GETSPEEDERRORLIMIT, &limit1, &limit2); 
}

bool Basicmicro::GetSpeedErrors(uint8_t address, uint16_t &error1, uint16_t &error2)
{
	return read_n_words(2, address, GETSPEEDERRORS, &error1, &error2);
}

bool Basicmicro::SetPosErrorLimit(uint8_t address, uint16_t limit1, uint16_t limit2)
{
    return write_n(6, address, SETPOSERRORLIMIT, SetWORDval(limit1), SetWORDval(limit2)); 
}

bool Basicmicro::GetPosErrorLimit(uint8_t address, uint16_t &limit1, uint16_t &limit2)
{
    return read_n_words(2, address, GETPOSERRORLIMIT, &limit1, &limit2); 
}

bool Basicmicro::GetPosErrors(uint8_t address, uint16_t &error1, uint16_t &error2)
{
	return read_n_words(2, address, GETPOSERRORS, &error1, &error2);
}

bool Basicmicro::SetOffsets(uint8_t address, uint8_t offset1, uint8_t offset2)
{
    return write_n(4, address, SETOFFSETS, offset1, offset2); 
}

bool Basicmicro::GetOffsets(uint8_t address, uint8_t &offset1, uint8_t &offset2)
{
    return read_n_bytes(2, address, GETOFFSETS, &offset1, &offset2); 
}

bool Basicmicro::M1Position(uint8_t address, uint32_t position, uint8_t buffer) 
{
    return write_n(7, address, M1POS, SetDWORDval(position), buffer); 
}

bool Basicmicro::M2Position(uint8_t address, uint32_t position, uint8_t buffer)
{
    return write_n(7, address, M2POS, SetDWORDval(position), buffer); 
}

bool Basicmicro::MixedPosition(uint8_t address, uint32_t position1, uint32_t position2, uint8_t buffer)
{
    return write_n(11, address, MIXEDPOS, SetDWORDval(position1), SetDWORDval(position2), buffer); 
}

bool Basicmicro::M1SpeedPosition(uint8_t address, uint32_t speed, uint32_t position, uint8_t buffer) 
{
    return write_n(11, address, M1SPEEDPOS, SetDWORDval(speed), SetDWORDval(position), buffer);
}

bool Basicmicro::M2SpeedPosition(uint8_t address, uint32_t speed, uint32_t position, uint8_t buffer)
{
    return write_n(11, address, M2SPEEDPOS, SetDWORDval(speed), SetDWORDval(position), buffer);
}

bool Basicmicro::MixedSpeedPosition(uint8_t address, uint32_t speed1, uint32_t position1, uint32_t speed2, uint32_t position2, uint8_t buffer)
{
    return write_n(19, address, MIXEDSPEEDPOS, SetDWORDval(speed1), SetDWORDval(position1), SetDWORDval(speed2), SetDWORDval(position2), buffer);
}

bool Basicmicro::M1PercentPosition(uint8_t address, int16_t position, uint8_t buffer)
{
    return write_n(5, address, M1PPOS, SetWORDval(position), buffer);
}

bool Basicmicro::M2PercentPosition(uint8_t address, int16_t position, uint8_t buffer)
{
    return write_n(5, address, M2PPOS, SetWORDval(position), buffer); 
}

bool Basicmicro::MixedPercentPosition(uint8_t address, int16_t position1, int16_t position2, uint8_t buffer)
{
    return write_n(7, address, MIXEDPPOS, SetWORDval(position1), SetWORDval(position2), buffer); 
}

bool Basicmicro::SetM1LR(uint8_t address, float L, float R)
{
    union {
        float f;
        uint32_t u;
    } l_union, r_union;

    l_union.f = L;
    r_union.f = R;

    return write_n(10, address, SETM1LR, SetDWORDval(l_union.u), SetDWORDval(r_union.u)); 
}

bool Basicmicro::SetM2LR(uint8_t address, float L, float R)
{
    union {
        float f;
        uint32_t u;
    } l_union, r_union;

    l_union.f = L;
    r_union.f = R;

    return write_n(10, address, SETM2LR, SetDWORDval(l_union.u), SetDWORDval(r_union.u));
}

bool Basicmicro::GetM1LR(uint8_t address, float &L, float &R)
{
    uint32_t lval_u, rval_u;

    if(read_n(2, address, GETM1LR, &lval_u, &rval_u)) {
        union {
            float f;
            uint32_t u;
        } l_union, r_union;

        l_union.u = lval_u;
        r_union.u = rval_u;

        L = l_union.f;
        R = r_union.f;
        return true;
    }

    return false;
}

bool Basicmicro::GetM2LR(uint8_t address, float &L, float &R)
{
    uint32_t lval_u, rval_u;

    if(read_n(2, address, GETM2LR, &lval_u, &rval_u)) {
        union {
            float f;
            uint32_t u;
        } l_union, r_union;

        l_union.u = lval_u;
        r_union.u = rval_u;

        L = l_union.f;
        R = r_union.f;
        return true;
    }

    return false;
}

bool Basicmicro::SetDOUT(uint8_t address, uint8_t index, uint8_t action)
{
    return write_n(4, address, SETDOUT, index, action);
}

bool Basicmicro::GetDOUTS(uint8_t address, uint8_t &count, uint8_t *actions, uint8_t maxActions)
{
    uint8_t trys = MAXRETRY;
    count = 0; 

    if(maxActions > 0 && !actions) return false;

    do {
        write_address_cmd(address, GETDOUTS);

        bool read_ok = true;
        uint8_t actual_count;

        if (!ReadByte(actual_count)) { read_ok = false; }
        if (!read_ok) continue;

        count = actual_count;

        for(uint8_t i = 0; i < actual_count; i++) {
            int16_t data = _read();
            if(data == -1) {
                read_ok = false;
            }

            if(read_ok && actions && i < maxActions) { 
                actions[i] = (uint8_t)data;
            }
        }
        if (!read_ok) continue;

        if(_checkcrc()) {
            return true;
        }
    } while(trys--); 

    count = 0; 
    return false; 
}

bool Basicmicro::SetPriority(uint8_t address, uint8_t priority1, uint8_t priority2, uint8_t priority3)
{
    return write_n(5, address, SETPRIORITY, priority1, priority2, priority3); 
}

bool Basicmicro::GetPriority(uint8_t address, uint8_t &priority1, uint8_t &priority2, uint8_t &priority3)
{
    return read_n_bytes(3, address, GETPRIORITY, &priority1, &priority2, &priority3); 
}

bool Basicmicro::SetAddressMixed(uint8_t address, uint8_t newAddress, uint8_t enableMixing)
{
    return write_n(4, address, SETADDRESSMIXED, newAddress, enableMixing); 
}

bool Basicmicro::GetAddressMixed(uint8_t address, uint8_t &newAddress, uint8_t &mixingEnabled)
{
	return read_n_bytes(2,address, GETADDRESSMIXED, &newAddress, &mixingEnabled);
}

bool Basicmicro::SetSignal(uint8_t address, uint8_t index, uint8_t signalType, uint8_t mode,
                          uint8_t target, uint16_t minAction, uint16_t maxAction, uint8_t lowpass,
                          uint32_t timeout, int32_t loadhome, int32_t minVal, int32_t maxVal,
                          int32_t center, uint32_t deadband, uint32_t powerexp, uint32_t minout,
                          uint32_t maxout, uint32_t powermin, uint32_t potentiometer)
{
    uint8_t trys = MAXRETRY;

    do {
        write_address_cmd(address, SETSIGNAL); 

        write(index);
        crc_update(index);
        write(signalType);
        crc_update(signalType);

        write(mode);
        crc_update(mode);
        write(target);
        crc_update(target);

        _writeword(minAction);
        _writeword(maxAction);

        write(lowpass);
        crc_update(lowpass);

        _writelong(timeout);

        _writelong((uint32_t)loadhome);
        _writelong((uint32_t)minVal);
        _writelong((uint32_t)maxVal);
        _writelong((uint32_t)center);

        _writelong(deadband);
        _writelong(powerexp);
        _writelong(minout);
        _writelong(maxout);
        _writelong(powermin);
        _writelong(potentiometer);

        if(_writechecksum()) {
            return true; 
        }
    } while(trys--); 

    return false; 
}

bool Basicmicro::GetSignals(uint8_t address, uint8_t &count, SignalConfig *signals, uint8_t maxSignals)
{
    uint8_t trys = MAXRETRY;
    count = 0; 

    if(maxSignals > 0 && !signals) return false;

    do {
        write_address_cmd(address, GETSIGNALS);

        bool read_ok = true;
        uint8_t actual_count;

        if (!ReadByte(actual_count)) { read_ok = false; }
        if (!read_ok) continue;

        count = actual_count;

        for(uint8_t i = 0; i < actual_count; i++) {
            SignalConfig config;
            bool current_config_ok = true;

			uint32_t val;
            if(current_config_ok && !ReadByte(config.type)) { current_config_ok = false; }
            if(current_config_ok && !ReadByte(config.mode)) { current_config_ok = false; }
            if(current_config_ok && !ReadByte(config.target)) { current_config_ok = false; }
            if(current_config_ok && !ReadWord(config.minAction)) { current_config_ok = false; }
            if(current_config_ok && !ReadWord(config.maxAction)) { current_config_ok = false; }
            if(current_config_ok && !ReadByte(config.lowpass)) { current_config_ok = false; }
            if(current_config_ok && !ReadLong(val)) { current_config_ok = false; config.timeout = val; }
            if(current_config_ok && !ReadLong(val)) { current_config_ok = false; config.loadhome = val; }
            if(current_config_ok && !ReadLong(val)) { current_config_ok = false; config.minVal = val; }
            if(current_config_ok && !ReadLong(val)) { current_config_ok = false; config.maxVal = val; }
            if(current_config_ok && !ReadLong(val)) { current_config_ok = false; config.center = val; }
            if(current_config_ok && !ReadLong(val)) { current_config_ok = false; config.deadband = val; }
			if(current_config_ok && !ReadLong(val)) { current_config_ok = false; config.powerexp = val; }
            if(current_config_ok && !ReadLong(val)) { current_config_ok = false; config.minout = val; }
			if(current_config_ok && !ReadLong(val)) { current_config_ok = false; config.maxout = val; }
            if(current_config_ok && !ReadLong(val)) { current_config_ok = false; config.powermin = val; }
			if(current_config_ok && !ReadLong(val)) { current_config_ok = false; config.potentiometer = val; }

            if (!current_config_ok) {
                read_ok = false;
                break; 
            }

            if(i < maxSignals) {
                signals[i] = config;
            }
        } 

        if(read_ok) { 
            if(_checkcrc()) {
                return true; 
            }
        }

    } while(trys--); 

    count = 0;
    return false;
}

bool Basicmicro::SetStream(uint8_t address, uint8_t index, uint8_t streamType, uint32_t baudrate, uint32_t timeout)
{
    uint8_t trys = MAXRETRY;

    do {
        write_address_cmd(address, SETSTREAM);

        write(index);
        crc_update(index);
        write(streamType);
        crc_update(streamType);

        _writelong(baudrate);
        _writelong(timeout);

        if(_writechecksum()) {
            return true;
        }
    } while(trys--); 

    return false;
}

bool Basicmicro::GetStreams(uint8_t address, uint8_t &count, StreamConfig *streams, uint8_t maxStreams)
{
    uint8_t trys = MAXRETRY;
    count = 0;

    if(maxStreams > 0 && !streams) return false;

    do {
        write_address_cmd(address, GETSTREAMS);

        bool read_ok = true;
        uint8_t actual_count;

        if (!ReadByte(actual_count)) { read_ok = false; }
        if (!read_ok) continue;

        count = actual_count;

        for(uint8_t i = 0; i < actual_count; i++) {
            StreamConfig config;
            bool current_config_ok = true;

            if(current_config_ok && !ReadByte(config.type)) { current_config_ok = false; }
            if(current_config_ok && !ReadLong(config.baudrate)) { current_config_ok = false; }
			if(current_config_ok && !ReadLong(config.timeout)) { current_config_ok = false; }

            if (!current_config_ok) {
                 read_ok = false;
                 break; 
            }

            if(streams && i < maxStreams) {
                streams[i] = config;
            }
        }

        if(read_ok) { 
            if(_checkcrc()) {
                return true;
            }
        }

    } while(trys--); 

    count = 0;
    return false; 
}


bool Basicmicro::GetSignalsData(uint8_t address, uint8_t &count, SignalData *signalsData, uint8_t maxSignals)
{
    uint8_t trys = MAXRETRY;
    count = 0;

    if(maxSignals > 0 && !signalsData) return false;

    do {
        write_address_cmd(address, GETSIGNALSDATA);

        bool read_ok = true;
        uint8_t actual_count;

        if (!ReadByte(actual_count)) { read_ok = false; }
        if (!read_ok) continue;

        count = actual_count;

        for(uint8_t i = 0; i < actual_count; i++) {
            SignalData sData;
            bool current_data_ok = true;

            if(!ReadLong(sData.command)) { current_data_ok = false; }
            if(current_data_ok && !ReadLong(sData.position)) { current_data_ok = false; }
            if(current_data_ok && !ReadLong(sData.percent)) { current_data_ok = false; }
            if(current_data_ok && !ReadLong(sData.speed)) { current_data_ok = false; }
            if(current_data_ok && !ReadLong(sData.speeds)) { current_data_ok = false; }

            if (!current_data_ok) {
                 read_ok = false;
                 break;
            }

            if(signalsData && i < maxSignals) {
                signalsData[i] = sData;
            }
        } 

        if(read_ok) { 
            if(_checkcrc()) {
                return true;
            }
        }

    } while(trys--);

    count = 0;
    return false;
}

bool Basicmicro::SetNodeID(uint8_t address, uint8_t nodeID)
{
    return write_n(3, address, SETNODEID, nodeID); 
}

bool Basicmicro::GetNodeID(uint8_t address, uint8_t &nodeID)
{
	return read_n_bytes(1,address, GETNODEID, &nodeID);
}

bool Basicmicro::SetPWMIdle(uint8_t address, float idleDelay1, bool idleMode1, float idleDelay2, bool idleMode2)
{
    if (idleDelay1 < 0.0f) idleDelay1 = 0.0f;
    if (idleDelay1 > 12.7f) idleDelay1 = 12.7f;
    uint8_t byte1 = (uint8_t)(idleDelay1 * 10.0f) & 0x7F;
    if(idleMode1) byte1 |= 0x80; 

    if (idleDelay2 < 0.0f) idleDelay2 = 0.0f;
    if (idleDelay2 > 12.7f) idleDelay2 = 12.7f;
    uint8_t byte2 = (uint8_t)(idleDelay2 * 10.0f) & 0x7F;
    if(idleMode2) byte2 |= 0x80;  

    return write_n(4, address, SETPWMIDLE, byte1, byte2); 
}

bool Basicmicro::GetPWMIdle(uint8_t address, float &idleDelay1, bool &idleMode1, float &idleDelay2, bool &idleMode2)
{
    uint8_t byte1, byte2;
	if(read_n_bytes(2,address, GETPWMIDLE, &byte1, &byte2)){
		idleDelay1 = (byte1 & 0x7F) / 10.0f;
		idleDelay2 = (byte2 & 0x7F) / 10.0f;

		idleMode1 = (byte1 & 0x80) != 0;
		idleMode2 = (byte2 & 0x80) != 0;

        return true; 
	}
    return false;
}

bool Basicmicro::CANBufferState(uint8_t address, uint8_t &count)
{
	return read_n_bytes(1,address, CANBUFFERSTATE, &count);
}

bool Basicmicro::CANPutPacket(uint8_t address, uint16_t cobID, uint8_t rtr, uint8_t length, const uint8_t *data)
{
    uint8_t trys = MAXRETRY;

    if(length > 8) length = 8;
    if(length > 0 && !data) return false;

    do {
        write_address_cmd(address, CANPUTPACKET);

        _writeword(cobID); 

        write(rtr);
        crc_update(rtr);

        write(length);
        crc_update(length);

        for(uint8_t i = 0; i < 8; i++) {
            uint8_t byteVal = (i < length) ? data[i] : 0;
            write(byteVal);
            crc_update(byteVal);
        }

        if(_writechecksum()) {
            return true;
        }
    } while(trys--); 

    return false;
}

bool Basicmicro::CANGetPacket(uint8_t address, uint16_t &cobID, uint8_t &rtr, uint8_t &length, uint8_t *data)
{
    uint8_t trys = MAXRETRY;

    if(!data) return false;

    do {
        write_address_cmd(address, CANGETPACKET); 

        bool read_ok = true;
        uint8_t packet_status;

        if(!ReadByte(packet_status)) { read_ok = false; }
        if (!read_ok) continue;

        if(packet_status != 0xFF) {
            if (_checkcrc()) {
                return false; 
            } else {
                 continue; 
            }
        }

        uint16_t temp_cobID;
        uint8_t temp_rtr, temp_length;
        uint8_t temp_data[8];

        if(!ReadWord(temp_cobID)) { read_ok = false; }

        if(read_ok && !ReadByte(temp_rtr)) { read_ok = false; }

        if(read_ok && !ReadByte(temp_length)) { read_ok = false; }

        if (read_ok) {
            for(uint8_t i = 0; i < 8; i++) {
                int16_t byte_data = _read();
                if(byte_data == -1) {
                    read_ok = false; 
                }
                if(read_ok) temp_data[i] = (uint8_t)byte_data;
            }
        }
        if (!read_ok) continue; 

        if(_checkcrc()) {
            cobID = temp_cobID;
            rtr = temp_rtr;
            length = (temp_length <= 8) ? temp_length : 8; 
            for(uint8_t i = 0; i < length; i++) {
                data[i] = temp_data[i];
            }
            return true;
        }

    } while(trys--); 

    length = 0; 
    return false;
}

bool Basicmicro::CANOpenWriteDict(uint8_t address, uint8_t nodeID, uint16_t index, uint8_t subindex, uint32_t value, uint8_t size, uint32_t &result)
{
    uint8_t trys = MAXRETRY;
    result = 0;

    if (size != 1 && size != 2 && size != 4) {
        return false;
    }

    do {
        write_address_cmd(address, CANOPENWRITEDICT);

        _write(nodeID);

        _writeword(index);

        _write(subindex);

        _writelong(value); 

        write(size);
        crc_update(size);

        uint16_t calculated_crc = crc_get();
        write(calculated_crc >> 8); 
        write(calculated_crc & 0xFF); 

        crc_clear();

        bool read_ok = true;
        uint32_t temp_result;

        if(!ReadLong(temp_result)) { read_ok = false; }
        if (!read_ok) continue; 

        if(_checkcrc()) {
            result = temp_result;
            return true;
        }

    } while(trys--); 

    return false; 
}

bool Basicmicro::CANOpenReadDict(uint8_t address, uint8_t nodeID, uint16_t index, uint8_t subindex, uint32_t &value, uint8_t &size, uint8_t &type, uint32_t &result)
{
    uint8_t trys = MAXRETRY;
    value = 0; 
    size = 0;
    type = 0;
    result = 0;

    do {
        write_address_cmd(address, CANOPENREADDICT);

        _write(nodeID);
		
        _writeword(index);

        _write(subindex);

        uint16_t calculated_crc = crc_get();
        write(calculated_crc >> 8);
        write(calculated_crc & 0xFF); 

        crc_clear();

        bool read_ok = true;
        uint32_t temp_value, temp_result;
        uint8_t temp_size, temp_type;

        if(!ReadLong(temp_value)) { read_ok = false; }

        if(read_ok && !ReadByte(temp_size)) { read_ok = false; }

        if(read_ok && !ReadByte(temp_type)) { read_ok = false; }

        if(read_ok && !ReadLong(temp_result)) { read_ok = false; }

        if (!read_ok) continue; 

        if(_checkcrc()) {
            value = temp_value;
            size = temp_size;
            type = temp_type;
            result = temp_result;
            return true; 
        }

    } while(trys--); 

    return false; 
}

bool Basicmicro::ResetEStop(uint8_t address)
{
    return write_n(2, address, RESETESTOP); 
}

bool Basicmicro::SetEStopLock(uint8_t address, uint8_t lockState)
{
    return write_n(3, address, SETESTOPLOCK, lockState);
}

bool Basicmicro::GetEStopLock(uint8_t address, uint8_t &lockState)
{
	return read_n_bytes(1,address, GETESTOPLOCK, &lockState);
}

bool Basicmicro::SetScriptAutorun(uint8_t address, uint32_t scriptAutorunTime)
{
    return write_n(6, address, SETSCRIPTAUTORUN, SetDWORDval(scriptAutorunTime)); 
}

bool Basicmicro::GetScriptAutorun(uint8_t address, uint32_t &scriptAutorunTime)
{
    return read_n(1, address, GETSCRIPTAUTORUN, &scriptAutorunTime); 
}

bool Basicmicro::StartScript(uint8_t address)
{
    return write_n(2, address, STARTSCRIPT); 
}

bool Basicmicro::StopScript(uint8_t address)
{
    return write_n(2, address, STOPSCRIPT); 
}

bool Basicmicro::ReadEEPROM(uint8_t address, uint8_t eeAddress, uint16_t &value)
{
    uint8_t trys = MAXRETRY;

    do {
        write_address_cmd(address, READEEPROM);

        write(eeAddress);
        crc_update(eeAddress);

        bool read_ok = true;
        uint16_t temp_value;

        if(!ReadWord(temp_value)) { read_ok = false; }

        if (!read_ok) continue;

        if(_checkcrc()) {
            value = temp_value;
            return true;
        }
    } while(trys--);

    return false;
}

bool Basicmicro::WriteEEPROM(uint8_t address, uint8_t eeAddress, uint16_t value)
{
    return write_n(5, address, WRITEEEPROM, eeAddress, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)); 
}