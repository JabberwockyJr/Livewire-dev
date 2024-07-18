#include "em_i2c.h"
#include "adafruit_INA219.h"
#include "em_gpio.h"
#include <stdio.h>
#include <math.h>
// Defines
#define I2C_ADDRESS                     0x80
#define I2C_ADDRESS_MASK                0xFF // Must match exact I2C_ADDRESS
#define I2C_RXBUFFER_SIZE                 2
uint32_t ina219_calValue;
uint32_t ina219_currentDivider_mA;
float ina219_powerMultiplier_mW;
// Buffers++
uint8_t i2c_txBuffer[4];
uint8_t i2c_txBufferSize = 0;
uint8_t i2c_rxBuffer[4];
uint8_t i2c_rxBufferIndex;


uint32_t I2C_curr_flag = I2C_FLAG_WRITE;
// Transmission flags
volatile bool i2c_rxInProgress;
volatile bool i2c_startTx;


void init_i2c_f(void){
  // Using default settings
    I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
    // Use ~100khz SCK
    i2cInit.freq = I2C_FREQ_STANDARD_MAX;
    i2cInit.clhr = i2cClockHLRStandard;

    // Using PC0 (SDA) and PC1 (SCL)
    GPIO_PinModeSet(gpioPortC, 0, gpioModeWiredAndPullUpFilter, 1);
    GPIO_PinModeSet(gpioPortC, 1, gpioModeWiredAndPullUpFilter, 1);

    // Enable pins at location 4 as specified in datasheet
    I2C0->ROUTEPEN = I2C_ROUTEPEN_SDAPEN | I2C_ROUTEPEN_SCLPEN;
    I2C0->ROUTELOC0 = (I2C0->ROUTELOC0 & (~_I2C_ROUTELOC0_SDALOC_MASK)) | I2C_ROUTELOC0_SDALOC_LOC4;
    I2C0->ROUTELOC0 = (I2C0->ROUTELOC0 & (~_I2C_ROUTELOC0_SCLLOC_MASK)) | I2C_ROUTELOC0_SCLLOC_LOC4;

    // Initializing the I2C
    I2C_Init(I2C0, &i2cInit);

}
uint16_t performI2CTransfer(uint8_t opcode)
{
  // Transfer structure
  I2C_TransferSeq_TypeDef i2cTransfer;
  I2C_TransferReturn_TypeDef result;

  //prepend the register address to access.
  //i2c_txBuffer>>=8;
  i2c_txBuffer[0]=opcode;
  // Initializing I2C transfer
  //i2c_txBuffer = 0x051000;
  i2cTransfer.addr          = I2C_ADDRESS;
  i2cTransfer.flags         = I2C_curr_flag;
  i2cTransfer.buf[0].data   = i2c_txBuffer;
  i2cTransfer.buf[0].len    = i2c_txBufferSize;
  i2cTransfer.buf[1].data   = i2c_rxBuffer;
  i2cTransfer.buf[1].len    = I2C_RXBUFFER_SIZE;
  result = I2C_TransferInit(I2C0, &i2cTransfer);

  // Sending data
  while (result == i2cTransferInProgress)
  {
    result = I2C_Transfer(I2C0);
  }


}

void setCalibration_32V_2A(){
  ina219_calValue = 4096;
  ina219_currentDivider_mA = 10; // Current LSB = 100uA per bit (1000/100 = 10)
  ina219_powerMultiplier_mW = 2; // Power LSB = 1mW per bit (2/1)

    // Set Calibration register to 'Cal' calculated above
   I2C_curr_flag = I2C_FLAG_WRITE;
   i2c_txBuffer[1] = (ina219_calValue>>8)&0xFF;
   i2c_txBuffer[2] = (ina219_calValue)&0xFF;
   i2c_txBufferSize = 3;
   performI2CTransfer(INA219_REG_CALIBRATION);
   /** Adafruit_BusIO_Register calibration_reg =
        Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
    calibration_reg.write(ina219_calValue, 2);
  **/
    // Set Config register to take into account the settings above
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                      INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
                      INA219_CONFIG_SADCRES_12BIT_1S_532US |
                      INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
    /**Adafruit_BusIO_Register config_reg =
        Adafruit_BusIO_Register(i2c_dev, INA219_REG_CONFIG, 2, MSBFIRST);
    _success = config_reg.write(config, 2);**/
    i2c_txBuffer[1] = (config>>8)&0xFF;
    i2c_txBuffer[2] = (config)&0xFF;
    i2c_txBufferSize = 2;
    performI2CTransfer(INA219_REG_CONFIG);
}
/*!
 *  @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
 *  @return the raw bus voltage reading
 */
int16_t getBusVoltage_raw() {
  I2C_curr_flag = I2C_FLAG_WRITE_READ;
  i2c_txBufferSize = 1;
  performI2CTransfer(INA219_REG_BUSVOLTAGE);
  /**
  Adafruit_BusIO_Register bus_voltage_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_BUSVOLTAGE, 2, MSBFIRST);
  _success = bus_voltage_reg.read(&value);
  **/
  int16_t returned = i2c_rxBuffer[0]<<8|i2c_rxBuffer[1];

  // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
  return (int16_t)((returned >> 3) * 4);
}

/*!
 *  @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
 *  @return the raw shunt voltage reading
 */
int16_t getShuntVoltage_raw() {
  /**
  Adafruit_BusIO_Register shunt_voltage_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_SHUNTVOLTAGE, 2, MSBFIRST);
  _success = shunt_voltage_reg.read(&value);

  **/
  I2C_curr_flag = I2C_FLAG_WRITE_READ;
  i2c_txBufferSize = 1;
  performI2CTransfer(INA219_REG_SHUNTVOLTAGE);
  int16_t returned = i2c_rxBuffer[0]<<8|i2c_rxBuffer[1];
  return returned;
}

/*!
 *  @brief  Gets the raw current value (16-bit signed integer, so +-32767)
 *  @return the raw current reading
 */
int16_t getCurrent_raw() {
  uint16_t value;

  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  I2C_curr_flag = I2C_FLAG_WRITE;
  i2c_txBuffer[1] = (ina219_calValue>>8)&0xFF;
  i2c_txBuffer[2] = (ina219_calValue)&0xFF;
  i2c_txBufferSize = 3;
     performI2CTransfer(INA219_REG_CALIBRATION);
     /**
  Adafruit_BusIO_Register calibration_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
  calibration_reg.write(ina219_calValue, 2);
  **/
  // Now we can safely read the CURRENT register!
  I2C_curr_flag = I2C_FLAG_WRITE_READ;
  i2c_txBufferSize = 1;
  performI2CTransfer(INA219_REG_CURRENT);
  int16_t returned = i2c_rxBuffer[0]<<8|i2c_rxBuffer[1];

  return  returned;
  /**
  Adafruit_BusIO_Register current_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CURRENT, 2, MSBFIRST);
  _success = current_reg.read(&value);
  return value;
  **/
}

/*!
 *  @brief  Gets the raw power value (16-bit signed integer, so +-32767)
 *  @return raw power reading
 */
int16_t getPower_raw() {
  uint16_t value;

  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  I2C_curr_flag = I2C_FLAG_WRITE;
  i2c_txBuffer[1] = (ina219_calValue>>8)&0xFF;
  i2c_txBuffer[2] = (ina219_calValue)&0xFF;
  i2c_txBufferSize = 3;
       performI2CTransfer(INA219_REG_CALIBRATION);
       /**
  Adafruit_BusIO_Register calibration_reg =
      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
  calibration_reg.write(ina219_calValue, 2);
  **/
       I2C_curr_flag = I2C_FLAG_WRITE_READ;
       i2c_txBufferSize = 1;
       performI2CTransfer(INA219_REG_POWER);

       int16_t returned = i2c_rxBuffer[0]<<8|i2c_rxBuffer[1];

       return returned;

}

/*!
 *  @brief  Gets the shunt voltage in mV (so +-327mV)
 *  @return the shunt voltage converted to millivolts
 */
float getShuntVoltage_mV() {
  int16_t value;
  value = getShuntVoltage_raw();
  return value * 0.01;
}

/*!
 *  @brief  Gets the bus voltage in volts
 *  @return the bus voltage converted to volts
 */
float getBusVoltage_V() {
  int16_t value = getBusVoltage_raw();
  return value * 0.001;
}

/*!
 *  @brief  Gets the current value in mA, taking into account the
 *          config settings and current LSB
 *  @return the current reading convereted to milliamps
 */
float getCurrent_mA() {
  float valueDec = getCurrent_raw();
  valueDec /= ina219_currentDivider_mA;
  return valueDec;
}

/*!
 *  @brief  Gets the power value in mW, taking into account the
 *          config settings and current LSB
 *  @return power reading converted to milliwatts
 */
float getPower_mW() {
  float valueDec = getPower_raw();
  valueDec *= ina219_powerMultiplier_mW;
  return valueDec;
}

int sign_f(float value) {
    if (value > 0) return 1;  // Positive number
    if (value < 0) return -1; // Negative number
    return 0;                 // Zero
}

void printFloat_f(float value) {
    int intPart = (int)value; // Get integer part
    int fracPart = (int)(round(fabs(value - intPart) * 1000)); // Get fractional part, rounded properly

    // Adjust for cases where rounding the fractional part results in 1000
    if (fracPart >= 1000) {
        intPart += 1;
        fracPart -= 1000;
    }

    // Print the integer part, dot, and first three digits of the fractional part
    if(intPart==0&&sign_f(value)==-1){
        printf("-");
    }
    printf("%d.%03d\n", intPart, fracPart);
}


void ina219_begin_f(){
  //NVIC_EnableIRQ(I2C0_IRQn);

  float shuntvoltage = 0;
    float busvoltage = 0;
    float current_mA = 0;
    float loadvoltage = 0;
    float power_mW = 0;

    setCalibration_32V_2A();
    shuntvoltage = getShuntVoltage_mV();
    busvoltage = getBusVoltage_V();
    current_mA = getCurrent_mA();
    power_mW = getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);

    printf("Shunt Voltage: ");
    printFloat_f(shuntvoltage);

    printf("Bus Voltage: ");
    printFloat_f(busvoltage);

    printf("Current: ");
    printFloat_f(current_mA);

    printf("Power: ");
    printFloat_f(power_mW);

    printf("Load Voltage: ");
    printFloat_f(loadvoltage);

}
