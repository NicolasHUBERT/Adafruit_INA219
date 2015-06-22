/**************************************************************************/
/*! 
    @file     Adafruit_INA219.cpp
    @author   K. Townsend (Adafruit Industries)
	@license  BSD (see license.txt)
	
	Driver for the INA219 current sensor

	This is a library for the Adafruit INA219 breakout
	----> https://www.adafruit.com/products/???
		
	Adafruit invests time and resources providing this open source code, 
	please support Adafruit and open-source hardware by purchasing 
	products from Adafruit!

	@section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

#include "Adafruit_INA219.h"

/**************************************************************************/
/*! 
    @brief  Sends a single command byte over I2C
*/
/**************************************************************************/
void Adafruit_INA219::wireWriteRegister (uint8_t reg, uint16_t value)
{
  Wire.beginTransmission(ina219_i2caddr);

  #if ARDUINO >= 100
    Wire.write(reg);                       // Register
    Wire.write((value >> 8) & 0xFF);       // Upper 8-bits
    Wire.write(value & 0xFF);              // Lower 8-bits
  #else
    Wire.send(reg);                        // Register
    Wire.send(value >> 8);                 // Upper 8-bits
    Wire.send(value & 0xFF);               // Lower 8-bits
  #endif
  Wire.endTransmission();
}

/**************************************************************************/
/*! 
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
void Adafruit_INA219::wireReadRegister(uint8_t reg, uint16_t *value)
{
  Wire.beginTransmission(ina219_i2caddr);
  #if ARDUINO >= 100
    Wire.write(reg);                       // Register
  #else
    Wire.send(reg);                        // Register
  #endif
  Wire.endTransmission();
  
  delayMicroseconds(ina219_conversionTime); 

  Wire.requestFrom(ina219_i2caddr, (uint8_t)2);  
  #if ARDUINO >= 100
    // Shift values to create properly formed integer
    *value = ((Wire.read() << 8) | Wire.read());
  #else
    // Shift values to create properly formed integer
    *value = ((Wire.receive() << 8) | Wire.receive());
  #endif
}

/**************************************************************************/
/*! 
    @brief  Instantiates a new INA219 class
*/
/**************************************************************************/
Adafruit_INA219::Adafruit_INA219(uint8_t addr) {
  ina219_i2caddr = addr;
  ina219_currentDivider_mA = 0;
  ina219_powerMultiplicator_mW = 0;
}

/**************************************************************************/
/*! 
    @brief  Configures the INA219 with the parameters desired
		
		@param[in] vbus_range		Bus full-scale range in volts (16V or 32V)
		@param[in] ishunt_max		Max expected current in milliamperes (up to 3200)
		
	@note   These calculations assume a 0.1 ohm resistor is present
*/
/**************************************************************************/
void Adafruit_INA219::setCalibration( uint8_t busVoltageRange, uint16_t maxCurrentExpected, uint8_t ADCResolution, uint8_t OPERATING_MODE) {
	uint16_t shuntVoltageRange;
	uint16_t CurrentLSB;
	uint16_t PowerLSB;
	
	// Determine the bus voltage range
	busVoltageRange = ((busVoltageRange == 32) ? INA219_CONFIG_BVOLTAGERANGE_32V : INA219_CONFIG_BVOLTAGERANGE_16V);
	
	// Determine the shunt voltage range according to the maximum expected current
	if (maxCurrentExpected <= 400)
		shuntVoltageRange = INA219_CONFIG_GAIN_1_40MV;
	else if (maxCurrentExpected <= 800)
		shuntVoltageRange = INA219_CONFIG_GAIN_2_80MV;
	else if (maxCurrentExpected <= 1600)
		shuntVoltageRange = INA219_CONFIG_GAIN_4_160MV;
	else
		shuntVoltageRange = INA219_CONFIG_GAIN_8_320MV;
	
	// Determine time conversion according to shunt ADC resolution
	switch (ADCResolution)
	{
		case 9:
			ADCResolution = INA219_CONFIG_SADCRES_9BIT_1S_84US;
			ina219_conversionTime = 84; 
			break;
		case 10:
			ADCResolution = INA219_CONFIG_SADCRES_10BIT_1S_148US;
			ina219_conversionTime = 148;
			break;
		case 11:
			ADCResolution = INA219_CONFIG_SADCRES_11BIT_1S_276US;
			ina219_conversionTime = 276;
			break;
		case 12:
			ADCResolution = INA219_CONFIG_SADCRES_12BIT_1S_532US;
			ina219_conversionTime = 532;
			break;
		case 13:
			ADCResolution = INA219_CONFIG_SADCRES_12BIT_2S_1060US;
			ina219_conversionTime = 1060;
			break;
		case 14:
			ADCResolution = INA219_CONFIG_SADCRES_12BIT_4S_2130US;
			ina219_conversionTime = 2130;
			break;
		case 15:
			ADCResolution = INA219_CONFIG_SADCRES_12BIT_8S_4260US; 
			ina219_conversionTime = 4620;
			break;
		case 16:
			ADCResolution = INA219_CONFIG_SADCRES_12BIT_16S_8510US;
			ina219_conversionTime = 8510;
			break;
		case 17:
			ADCResolution = INA219_CONFIG_SADCRES_12BIT_32S_17MS;
			ina219_conversionTime = 17020;
			break;
		case 18:
			ADCResolution = INA219_CONFIG_SADCRES_12BIT_64S_34MS;
			ina219_conversionTime = 34050;
			break;
		case 19:
			ADCResolution = INA219_CONFIG_SADCRES_12BIT_128S_69MS;
			ina219_conversionTime = 68100;
			break;
		default:
			ADCResolution = INA219_CONFIG_SADCRES_12BIT_1S_532US;
			ina219_conversionTime = 532;
			break;		
	}
	
	// Calculate the current LSB and the power LSB
	CurrentLSB = (uint16_t)(ceil((double)((maxCurrentExpected * 100.0) / 32768.0)) * 10);
	PowerLSB = 20 * CurrentLSB;
	
	// Compute the calibration value and set Calibration register
	ina219_calValue = trunc(409600 / CurrentLSB);
	wireWriteRegister(INA219_REG_CALIBRATION, ina219_calValue);
	
	// Set multipliers to convert raw current/power values
	ina219_currentDivider_mA = trunc( 1000 / CurrentLSB);
    	ina219_powerMultiplicator_mW = PowerLSB / 1000;

  
	// Set Config register to take into account the settings above
	uint16_t config = busVoltageRange |
					  shuntVoltageRange |
					  (ADCResolution << 4)|
					  ADCResolution |
					  OPERATING_MODE;
	wireWriteRegister(INA219_REG_CONFIG, config);
	
}

/**************************************************************************/
/*! 
    @brief  Setups the HW (defaults to 32V and 3.2A for calibration values)
*/
/**************************************************************************/
void Adafruit_INA219::begin() {
	Wire.begin();	
	setCalibration();
}

/**************************************************************************/
/*! 
    @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t Adafruit_INA219::getBusVoltage_raw() {
  uint16_t value;
  wireReadRegister(INA219_REG_BUSVOLTAGE, &value);
  // Shift to the right 3 to drop CNVR and OVF
  return (int16_t)(value >> 3);
}

/**************************************************************************/
/*! 
    @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t Adafruit_INA219::getShuntVoltage_raw() {
  uint16_t value;
  wireReadRegister(INA219_REG_SHUNTVOLTAGE, &value);
  return (int16_t)value;
}

/**************************************************************************/
/*! 
    @brief  Gets the raw current value (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t Adafruit_INA219::getCurrent_raw() {
  uint16_t value;
  wireReadRegister(INA219_REG_CURRENT, &value);
  return (int16_t)value;
}

/**************************************************************************/
/*! 
    @brief  Gets the raw power value (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t Adafruit_INA219::getPower_raw() {
  uint16_t value;
  wireReadRegister(INA219_REG_POWER, &value);
  return (int16_t)value;
}
 
/**************************************************************************/
/*! 
    @brief  Gets the shunt voltage in mV (so +-327mV)
*/
/**************************************************************************/
float Adafruit_INA219::getShuntVoltage_mV() {
  int16_t value;
  value = getShuntVoltage_raw();
  return value * 0.01; //LSB = 10uV = 0.01mV
}

/**************************************************************************/
/*! 
    @brief  Gets the shunt voltage in volts
*/
/**************************************************************************/
float Adafruit_INA219::getBusVoltage_mV() {
  int16_t value = getBusVoltage_raw();
  return value * 4; //LSB = 4mV
}

/**************************************************************************/
/*! 
    @brief  Gets the current value in mA, taking into account the
            config settings and current LSB
*/
/**************************************************************************/
float Adafruit_INA219::getCurrent_mA() {
  int16_t valueDec = getCurrent_raw();
  return float(valueDec) / float(ina219_currentDivider_mA);
}

/**************************************************************************/
/*! 
    @brief  Gets the power value in mW, taking into account the
            config settings and current LSB
*/
/**************************************************************************/
float Adafruit_INA219::getPower_mW() {
  int16_t valueDec = getPower_raw();
  return float(valueDec * ina219_powerMultiplicator_mW);
}
