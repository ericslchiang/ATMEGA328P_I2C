#include "atmega328_I2C.h"
#include <stdint.h>
#include <avr/interrupt.h>

volatile uint8_t printVal = 0;
volatile uint8_t receiveDataReady = 0;

volatile uint8_t txBuffer[TWI_TX_BUFFER_LEN];
volatile uint8_t txBufferIndex = 0;
volatile uint8_t txBufferLen = 0;

volatile uint8_t rxBuffer[TWI_RX_BUFFER_LEN];
volatile uint8_t rxBufferIndex = 0;
volatile uint8_t rxBufferLen = 0;

volatile uint8_t slaRW;
volatile uint8_t stopStatus = 1; // 0 = repeated start, 1 = stop
volatile uint8_t i2cError = 0;

static volatile uint8_t testVal = 111;

/*
 * Function i2cInit
 * Desc     Initializes I2C functionality on the ATMEGA328p
 * Input    addr: 7bit i2c address the device is set as
 *          addrMask: bitmask to be applied to device i2c address
 *          sendStop: I2C SCL speed in Hz
*/
void i2cInit(uint8_t addr, uint8_t addrMask, uint32_t i2cSpeed)
{
  SREG |= 0b10000000;                                 // Enable global interrupts
  TWB = (uint8_t)((CLOCK_SPEED / i2cSpeed - 16) / 2); // Set SCL speed
  TWC = (1 << TWIE) | (1 << TWEN) | (1 << TWEA);      // Enable i2c and i2c interrupt
  TWA = addr << 1 | 0x01;                             // Set i2c address and enable i2c general call
  TWAM = addrMask << 1;                               // 7-bit MSB mask
  
  PRR &= ~(1 << 3);
  TCCR1A = 0;
  TCCR1B = ((1 << 2) | (1 << 0)); //Set timer prescaler to 1024
}

/*
 * Function i2cDisable
 * Desc     Disables I2C functionality on the ATMEGA328p
*/
void i2cDisable()
{
  TWC &= ~((1 << TWEN) | (1 << TWIE)); //Disable I2C
  PRR |= (1 << 3); //Disable timer1
}

/* 
 * Function i2cMasterTransmit
 * Desc     Becomes i2c master and transmits data to a slave
 * Input    address: 7bit i2c slave address to be transmitted to
 *          data: pointer to data
 *          length: number of bytes to transmit
 *          sendStop: flag used to indicate repeated start or stop at end of transmission
 * Output   status of finished transmission. 0 = Success, 1 = data overflow
 */
uint8_t i2cMasterTransmit(uint8_t numBytes, uint8_t *data, uint8_t addr, uint8_t sendStop)
{
  // Fill TX buffer with incoming data.
  if (numBytes > TWI_TX_BUFFER_LEN)
  {
    return 1; // Data doesn't fit in buffer
  }
  // while(TWC & (1 << TWINT)); //Wait for TWINT flag to be cleared

  slaRW = addr << 1 | 0x00; // Write bit is 0

  for (int i = 0; i < numBytes; i++)
  {
    txBuffer[i] = data[i];
  }
  txBufferLen = numBytes;
  txBufferIndex = 0;

  // If the system is in a repeated start send address instead of start bit
  if (!stopStatus)
  {
    TWD = slaRW;
    // Enable interrupts again to re-enter ISR routine
    i2cClearFlag();
  }
  else
  {
    // printVal = TWC; //00000101
    i2cSendStart();
    // printVal = TWC; //00100101
    // printVal = 3;
  }

  // Set up STOP/RepeatStart condition for current transmission.
  stopStatus = sendStop;
  return 0;
}

/* 
 * Function i2cMasterReceive
 * Desc     Becomes i2c master and requests data from a slave
 * Input    address: 7bit i2c slave address to be transmitted to
 *          data: pointer to data
 *          length: number of bytes to transmit
 *          sendStop: flag used to indicate repeated start or stop at end of transmission
 * Output   status of finished transmission. 0 = Success, 1 = data overflow, 2 = transmission timeout
 */
uint8_t i2cMasterReceive(uint8_t numBytes, uint8_t *data, uint8_t addr, uint8_t sendStop)
{
  // Fill RX buffer with incoming data.
  if (numBytes > TWI_RX_BUFFER_LEN)
  {
    return 1; // Data doesn't fit in buffer
  }

  slaRW = addr << 1 | 0x01; // Read bit is 1
  rxBufferLen = numBytes;
  rxBufferIndex = 0;

  // If the system is in a repeated start send address instead of start bit
  if (!stopStatus)
  {
    TWD = slaRW;
    i2cClearFlag(); // Enable interrupts again to re-enter ISR routine
  }
  else
  {
    i2cSendStart();
  }

  receiveDataReady = 0;
  TCNT1 = 0;
  while (!receiveDataReady) {
    if (TCNT1 > 15625) {
      i2cSendStop();
      return 2; //Timeout
    }
  }

  for (int i = 0; i < rxBufferLen; i++)
  {
    data[i] = rxBuffer[i];
  }

  // Set up STOP/RepeatStart condition for current transmission.
  stopStatus = sendStop;
  return 0;
}

/* 
 * Function i2cSetSlaveTransmission
 * Desc     Set up the data to send to a I2C master read request
 * Input    data: pointer to data
 *          length: number of bytes to transmit
 * Output   status of finished transmission. 0 = Success, 1 = data overflow
 */
uint8_t i2cSetSlaveTransmission(uint8_t numBytes, uint8_t *data)
{
  //More data than allowed
  if (numBytes > TWI_TX_BUFFER_LEN)
  {
    return 1;
  }

  for (int i = 0; i < numBytes; i++)
  {
    txBuffer[i] = data[i];
  }

  txBufferIndex = 0;
  txBufferLen = numBytes;

  return 0;
}

// uint8_t i2cOnSlaveReceive(uint8_t numBytes, uint8_t *data) {
//   //More data than allowed
//   if (numBytes > TWI_TX_BUFFER_LEN)
//   {
//     return 1;
//   }

//   for (int i = 0; i < numBytes; i++) {
//     data[i] = rxBuffer[i];
//   }

//   rxBufferIndex = 0;

//   return 0;
// }

/* 
 * Function i2cOnSlaveReceive
 * Desc     Reads the latest data received from a I2C master
 * Input    data: pointer to data
 *          length: number of bytes to transmit
 * Output   status of finished transmission. 0 = Success, 1 = No data received, 
 *                2 = Less than expected amount of data received, 3 = More than expected amount of data received
 */
uint8_t i2cOnSlaveReceive(uint8_t numBytes, uint8_t *data) {
  if (rxBufferLen == 0) return 1; // No data

  for (int i = 0; i < rxBufferLen; i++) {
    data[i] = rxBuffer[i];
  }

  if (numBytes > rxBufferLen) {
    return 2; //Not enough data
  } else if (numBytes < rxBufferLen) {
    return 3; //More than requested amount of data
  } else {
    return 0;
  }
}

ISR(TWI_vect)
{
  switch (TWS & 0xF8)
  {
  // Send data address
  case M_START:
  case M_RSTART:
    TWD = slaRW;
    i2cClearFlag();
    break;

  // Master Transmitter
  // Byte was sent successfully
  case MT_SLA_W_ACK:
  case MT_DATA_ACK:
    // Check if there's more data
    if (txBufferIndex < txBufferLen)
    {
      TWD = txBuffer[txBufferIndex];
      txBufferIndex++;
      i2cClearFlag();
    }
    else
    {
      if (!stopStatus)
      {
        // Generate the start bit but disable interrupts until data is ready in the next transmit
        TWC = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
      }
      else
      {
        i2cSendStop();
      }
    }

    break;
  // Master Transmitter Error Cases
  case MT_SLA_W_NACK:
  case MT_DATA_NACK:
  case M_ARB_LOST:
    i2cSendStop();         // Terminate current bus transaction
    i2cError = TWS & 0xF8; // Mask for first 5 bits
    stopStatus = 1;        // Terminate repeated start
    break;

  // Master Receiver
  case MR_SLA_R_ACK:
    i2cSendAck();
    break;
  case MR_DATA_ACK:
    rxBuffer[rxBufferIndex] = TWD;
    printVal = rxBuffer[rxBufferIndex];
    rxBufferIndex++;

    // Check if there's more room for data
    if (rxBufferIndex + 1 < rxBufferLen)
    {
      i2cSendAck();
    }
    else
    {
      i2cSendNack();
    }
    break;
  // Receive last byte. Either send STOP or REPEAT START condition
  case MR_DATA_NACK:
    rxBuffer[rxBufferIndex] = TWD;
    printVal = rxBuffer[rxBufferIndex];
    rxBufferIndex++;
    receiveDataReady = 1;

    if (!stopStatus)
    {
      // Generate the start bit but disable interrupts until data is ready in the next transmit
      TWC = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    }
    else
    {
      i2cSendStop();
    }
    break;
  case MR_SLA_R_NACK:
    i2cError = TWS & 0xF8; // Mask for first 5 bits
    stopStatus = 1;        // Terminate repeated start
    i2cSendStop();         // Terminate current bus transaction
    break;

  // Slave Receiver
  // Setup receive buffer to store data
  case SR_SLA_W_ACK:
  case SR_ARB_LOST_SLA_W_ACK:
  case SR_GEN_CALL_ACK:
  case SR_ARB_LOST_GEN_CALL_ACK:
    rxBufferIndex = 0;
    rxBufferLen = 0;
    i2cSendAck();
    break;
  // Receive data if there's room
  case SR_DATA_ACK:
  case SR_PREV_GEN_CALL_DATA_ACK:
    if (rxBufferIndex < TWI_RX_BUFFER_LEN)
    {
      rxBuffer[rxBufferIndex] = TWD;
      rxBufferIndex++;
      rxBufferLen++;
      i2cSendAck();
    }
    else
    {
      i2cSendNack();
    }
    break;
  // Data received, send NACK back
  case SR_DATA_NACK:
  case SR_PREV_GEN_CALL_DATA_NACK:
    rxBuffer[rxBufferIndex] = TWD;
    rxBufferIndex++;
    rxBufferLen++;
    i2cSendNack();
    break;
  // Stop or repeated start received
  case SR_STOP_RSTART:
    i2cSendAck();
    break;

  // Slave Transmitter
  // Check if there is data in the tx buffer. If not send '123' in ASCII
  case ST_SLA_R_ACK:
  case ST_ARB_LOST_SLA_R_ACK:
    txBufferIndex = 0;

  // Serial.println("here");
  //  if (txBufferLen == 0) {
  //    txBufferLen = 3;
  //    txBufferIndex = 0;
  //    txBuffer[0] = 0xFF;
  //    txBuffer[1] = 0x02;
  //    txBuffer[2] = 0x03;
  //  }
  // i2cSendAck(); //Enabling this creates some weird results with the next data byte being transmitted
  // TWC = (1 << TWEA) | (1 << TWEN) | (1 << TWIE); //Send Ack bit but don't clear interrupt flag
  case ST_DATA_ACK:
    TWD = txBuffer[txBufferIndex];
    txBufferIndex++;
    if (txBufferIndex < txBufferLen)
    {
      i2cSendAck();
    }
    else
    {
      i2cClearFlag();
    }
    break;
  case ST_DATA_NACK:
  case ST_LAST_DATA_ACK:
    i2cSendAck();
    break;

  // Error Cases
  case BUS_ERROR:
    i2cSendStop();
    break;
  }
}