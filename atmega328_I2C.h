/*
I2C library for the ATMEGA328 MCU
2024 Eric Chiang
*/

#ifndef atmega328_I2C_h
#define atemaga328_I2C_h
#include <stdint.h>

//Timer registers
#define PRR (*((volatile uint8_t *) 0x64)) //Enable/Disable timer
#define TCCR1A (*((volatile uint8_t *) 0x80)); //Timer1 control reg
#define TCCR1B (*((volatile uint8_t *) 0x81)); //Timer1 control reg

// i2c registers
#define TWB (*((volatile uint8_t *)0xB8))
#define TWC (*((volatile uint8_t *)0xBC))
#define TWS (*((volatile uint8_t *)0xB9))
#define TWD (*((volatile uint8_t *)0xBB))
#define TWA (*((volatile uint8_t *)0xBA))
#define TWAM (*((volatile uint8_t *)0xBD))

// Global interrupt register
#define SREG (*((volatile uint8_t *)0x5F))

// TWCR register bit masks
#define TWINT 7 // Interrupt flag
#define TWEA 6  // Enable Acknowledge
#define TWSTA 5 // Generate start bit
#define TWSTO 4 // Generate stop bit
#define TWWC 3  // Write collision flag
#define TWEN 2  // Enable i2c
#define TWIE 0  // Enable interrupt

// i2c status codes
//Master Start and Restart
#define M_START 0x08
#define M_RSTART 0x10
#define M_ARB_LOST 0x38

//Master Transmitter
#define MT_SLA_W_ACK 0x18
#define MT_SLA_W_NACK 0x20
#define MT_DATA_ACK 0x28
#define MT_DATA_NACK 0x30

//Master Receiver
#define MR_SLA_R_ACK 0x40
#define MR_SLA_R_NACK 0x48
#define MR_DATA_ACK 0x50
#define MR_DATA_NACK 0x58

//Slave Receiver
#define SR_SLA_W_ACK 0x60
#define SR_ARB_LOST_SLA_W_ACK 0x68
#define SR_GEN_CALL_ACK 0x70
#define SR_ARB_LOST_GEN_CALL_ACK 0x78
#define SR_DATA_ACK 0X80
#define SR_DATA_NACK 0x88
#define SR_PREV_GEN_CALL_DATA_ACK 0x90
#define SR_PREV_GEN_CALL_DATA_NACK 0x98
#define SR_STOP_RSTART 0xA0

//Slave Transmitter
#define ST_SLA_R_ACK 0xA8
#define ST_ARB_LOST_SLA_R_ACK 0xB0
#define ST_DATA_ACK 0xB8
#define ST_DATA_NACK 0xC0
#define ST_LAST_DATA_ACK 0xC8

#define BUS_ERROR 0x00

// i2c clock speed
#define CLOCK_SPEED 16000000UL

// Set the maximum required buffer size in bytes for i2c communication
#define TWI_TX_BUFFER_LEN 16
#define TWI_RX_BUFFER_LEN 16

// Function for TWCR Control
#define i2cSendStop() (TWC |= (1 << TWINT) | (1 << TWSTO) | (1 << TWEN) | (1 << TWIE))
#define i2cSendAck() (TWC = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE))
#define i2cSendNack() (TWC = (1 << TWINT) | (1 << TWEN) | (1 << TWIE))
#define i2cClearFlag() (TWC = (1 << TWINT) | (1 << TWEN) | (1 << TWIE))
#define i2cSendStart() (TWC = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE))

// Function prototypes
void i2cInit(uint8_t addr, uint8_t addrMask, uint32_t i2cSpeed);                            // Enables global interrupts, i2c interrupts, and sets i2c clock speed
void i2cDisable();
uint8_t i2cMasterTransmit(uint8_t numBytes, uint8_t *data, uint8_t addr, uint8_t sendStop); // As master send specified number of bytes from given i2c slave address
uint8_t i2cMasterReceive(uint8_t numBytes, uint8_t *data, uint8_t addr, uint8_t sendStop);  // As master read specified number of bytes from given i2c slave address
uint8_t i2cSetSlaveTransmission(uint8_t numBytes, uint8_t *data);                                  // As slave transfer number of bytes onto data line
// uint8_t i2cOnSlaveReceive(uint8_t numBytes, uint8_t *data); //Populates given data with received I2C data
uint8_t i2cOnSlaveReceive(uint8_t numBytes, uint8_t *data);

extern volatile uint8_t printVal;

#endif
