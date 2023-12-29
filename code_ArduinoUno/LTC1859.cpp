#include <stdint.h>
#include "Linduino.h"
#include "LTC1859.h"
#include "LT_SPI.h"
#include <SPI.h>



// Builds the ADC command
uint8_t LTC1859_build_command(uint8_t ch_designate, uint8_t uni_bipolar, uint8_t range_low_high)
{
  uint8_t adc_command;
  adc_command = ch_designate | uni_bipolar | range_low_high;
  return (adc_command);
}


// Reads the ADC  and returns 16-bit data
void LTC1859_read(uint8_t cs, uint8_t adc_command, uint16_t *adc_code)
{
  spi_transfer_word(cs, (uint16_t)(adc_command << 8), adc_code);
}


// Calculates the LTC1859 input voltage given the data, vref, bits, and unipolar/bipolar status.
float LTC1859_code_to_voltage(uint16_t adc_code, float vref, uint8_t range_low_high, uint8_t uni_bipolar)
{
  float voltage;
  float sign = 1;

  if (range_low_high == LTC1859_LOW_RANGE_MODE)
    vref = vref / 2;

  if (uni_bipolar == LTC1859_UNIPOLAR_MODE)
  {
    voltage = (float)adc_code;
    voltage = voltage / (pow(2, 16) - 1); //! 2) This calculates the input as a fraction of the reference voltage (dimensionless)
  }
  else
  {
    if ((adc_code & 0x8000) == 0x8000)	//adc code is < 0
    {
      adc_code = (adc_code ^ 0xFFFF) + 1;                                  //! Convert ADC code from two's complement to binary
      sign = -1;
    }
    voltage = sign * (float)adc_code;
    voltage = voltage / (pow(2, 15) - 1); //! 2) This calculates the input as a fraction of the reference voltage (dimensionless)
  }

  voltage = voltage * vref;           //! 3) Multiply fraction by Vref to get the actual voltage at the input (in volts)

  return (voltage);
}
