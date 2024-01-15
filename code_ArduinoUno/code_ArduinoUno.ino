/*!
  ----------------------------------------------------------------------
  code_ArduinoUno.ino
  Objective: Lecture from FT sensor from ADC converter trought SPI protocol, send
  the string of 6 channels data via Uart to OpenCM904.
  
  Created By Antonio Camposeo | Politecnico di Bari
  Based on Code created by Luis Merida | UCLM Ciudad Real 

  ADC Config: Single_Ended, Bipolar measurement, Input Range High
  Linear Technology DC682A Demonstration Board.
  LTC1859: 16-Bit, 8-Channel 100ksps SAR ADC with SPI Interface.
  ----------------------------------------------------------------------
*/
#include <stdint.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "LTC1859.h"
#include "LowPassFilter.h"
#include <SPI.h>
#define UART0 Serial 

/* TODO: control the execution times of the various scripts to send messages in a defined time step */
// Global variables

LowPassFilter filter; // Define a struct for the filter, reference code LowPassFilter.h

static uint8_t uni_bipolar = LTC1859_BIPOLAR_MODE;    // Default set for unipolar mode
static uint8_t single_ended_differential = LTC1859_SINGLE_ENDED_MODE;    // LTC1859 Unipolar or Bipolar mode selection
static uint8_t range_low_high = LTC1859_HIGH_RANGE_MODE;   // Default set for high range mode

static float LTC1859_bits = 16;                      // Resolution (16 bits)
static float LTC1859_vref = 10;

// Constants
// Lookup table to build the command for single-ended mode, input with respect to GND
const uint8_t BUILD_COMMAND_SINGLE_ENDED[8] = {LTC1859_CH0, LTC1859_CH1, LTC1859_CH2, LTC1859_CH3,
                                               LTC1859_CH4, LTC1859_CH5, LTC1859_CH6, LTC1859_CH7
                                              }; // Builds the command for single-ended mode, input with respect to GND
                                               
uint8_t user_command;
uint16_t adc_command[8];         // The LTC1859 command byte
uint16_t adc_code = 0;        // The LTC1859 code

const int decimales = 4;
volatile float V_CH[8];
String str;
// Setup function
void setup()
{
  spi_init(); // Init spi to read sensor values
  UART0.begin(115200); // Serial connection arduino pin : 0|1 RX|TX

  LowPassFilter_Init(&filter,60,188); // Init Filter if necessary 

  // Check data reading 
  adc_command[0] = BUILD_COMMAND_SINGLE_ENDED[0] | uni_bipolar | range_low_high;
  // read single-ended
  LTC1859_read(LTC1859_CS, adc_command[0], &adc_code);     // Throws out last reading and starts CH0 conversion
  LTC1859_read(LTC1859_CS, adc_command[0], &adc_code);     // Throws out last reading and starts CH0 conversion

  for (int i = 0; i < 8; ++i) {
    adc_command[i] = BUILD_COMMAND_SINGLE_ENDED[i] | uni_bipolar | range_low_high;
  }
  
}
/* TODO: implement interrupt from GPIO of OpenCM904 to Pin 2 or 3 of arduino
in the function handler will be the UART0.write(str.c_str());
*/

void loop()
{ 
  
  // String esp = " ";
  while(UART0.available() == 0)
  { 
    for (int i = 0; i < 8; ++i) {      
      LTC1859_read(LTC1859_CS, adc_command[i], &adc_code);   // Read previous channel conversion (x-1) and start next one (x)
      LTC1859_read(LTC1859_CS, adc_command[i], &adc_code);   // Read previous channel conversion (x-1) and start next one (x)
      //V_CH[i] = LowPassFilter_Update(&filter,LTC1859_code_to_voltage(adc_code, LTC1859_vref, range_low_high, uni_bipolar));
      V_CH[i] =  LTC1859_code_to_voltage(adc_code, LTC1859_vref, range_low_high, uni_bipolar);
    }
    str = String((25 + V_CH[0]) * pow(10, decimales), 0)+String((25 + V_CH[1]) * pow(10, decimales), 0)+String((25 + V_CH[2]) * pow(10, decimales), 0) + String((25 + V_CH[3]) * pow(10,  decimales), 0)+String((25 + V_CH[4]) * pow(10, decimales), 0) + String((25 + V_CH[5]) * pow(10, decimales), 0)+ String("\n");
    
   }
   UART0.write(str.c_str()); 
   UART0.flush();
   // delay(1);
   
    //if(UART0.read() == '0')
    //{
    //}
  }
//}
