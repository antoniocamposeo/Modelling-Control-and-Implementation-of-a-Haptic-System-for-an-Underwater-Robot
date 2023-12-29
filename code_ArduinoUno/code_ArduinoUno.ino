/*!
  ----------------------------------------------------------------------
  OpenCM_Sensor.ino
  Obiective: lecture FT sensor from ADC converter trought SPI protocol.

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

//#include <SoftwareSerial.h>
//Demostrataion of chosing baund rate 
// we need to sent 43 byte every message and so we want to have a frequency 
// which is 250hz we need 250 message per second 
// So, 43*250 = 10750, are the byte every second 
// Considering this table:
// 57600 bauds  57600 bits/s  17.361 µs   7200 bytes/s  5760 bytes/s  173.611 µs
// 76800 bauds   76800 bits/s  13.021 µs   9600 bytes/s  7680 bytes/s  130.208 µs
// 115200 bauds  115200 bits/s   8.681 µs  14400 bytes/s   11520 bytes/s   86.806 µs
// the best choise is 115200baund, if we limit the message might be a good option 
// reduce the baundrate

/*todo: controllare i tempi di esecuzione dei vari script per inviare i messaggi in un time step definito */
// SoftwareSerial UART0(0, 1); // RX, TX
#define UART0 Serial
LowPassFilter filter;
// Global variables
static uint8_t uni_bipolar = LTC1859_BIPOLAR_MODE;    //!< Default set for unipolar mode
static uint8_t single_ended_differential = LTC1859_SINGLE_ENDED_MODE;    //!< LTC1859 Unipolar or Bipolar mode selection
static uint8_t range_low_high = LTC1859_HIGH_RANGE_MODE;   //!< Default set for high range mode

static float LTC1859_bits = 16;                      //!< Resolution (16 bits)
static float LTC1859_vref = 10;

// Constants
//! Lookup table to build the command for single-ended mode, input with respect to GND
const uint8_t BUILD_COMMAND_SINGLE_ENDED[8] = {LTC1859_CH0, LTC1859_CH1, LTC1859_CH2, LTC1859_CH3,
                                               LTC1859_CH4, LTC1859_CH5, LTC1859_CH6, LTC1859_CH7
                                              }; //!< Builds the command for single-ended mode, input with respect to GND
                                               
// float myTime;
// float myTime_a;
// int i = 0;

uint8_t user_command;
uint16_t adc_command[8];         // The LTC1859 command byte
uint16_t adc_code = 0;        // The LTC1859 code

//float V_CH[8];
//String esp = " ";
const int decimales = 4;
//String str;
//String esp = " ";

//! Initialize Linduino
void setup()
{
  spi_init(); // Init spi to read sensor values
  UART0.begin(115200); // Serial connection arduino pin:4|5 RX|TX
  //Serial.begin(115200);
  LowPassFilter_Init(&filter,60,188);
  adc_command[0] = BUILD_COMMAND_SINGLE_ENDED[0] | uni_bipolar | range_low_high;

  //read single-ended
  LTC1859_read(LTC1859_CS, adc_command[0], &adc_code);     // Throws out last reading and starts CH0 conversion
  LTC1859_read(LTC1859_CS, adc_command[0], &adc_code);     // Throws out last reading and starts CH0 conversion

  for (int i = 0; i < 8; ++i) {
    adc_command[i] = BUILD_COMMAND_SINGLE_ENDED[i] | uni_bipolar | range_low_high;
  }
  
}


void loop()
{ 
  volatile float V_CH[8];
  String str;
  String esp = " ";

  //UART0.write('0');
  while(UART0.available()==0){ 
  for (int i = 0; i < 8; ++i) {
    LTC1859_read(LTC1859_CS, adc_command[i], &adc_code);   // Read previous channel conversion (x-1) and start next one (x)
    LTC1859_read(LTC1859_CS, adc_command[i], &adc_code);   // Read previous channel conversion (x-1) and start next one (x)
    //V_CH[i] = LowPassFilter_Update(&filter,LTC1859_code_to_voltage(adc_code, LTC1859_vref, range_low_high, uni_bipolar));
    V_CH[i] =  LTC1859_code_to_voltage(adc_code, LTC1859_vref, range_low_high, uni_bipolar);
  
  }
  
  str = String((25 + V_CH[0]) * pow(10, decimales), 0)+String((25 + V_CH[1]) * pow(10, decimales), 0)+String((25 + V_CH[2]) * pow(10, decimales), 0) + String((25 + V_CH[3]) * pow(10,  decimales), 0)+String((25 + V_CH[4]) * pow(10, decimales), 0) + String((25 + V_CH[5]) * pow(10, decimales), 0)+ String("\n");
  }
  if(UART0.read() == '0'){
   
  /*for (int i = 0; i < 8; ++i) {
    LTC1859_read(LTC1859_CS, adc_command[i], &adc_code);   // Read previous channel conversion (x-1) and start next one (x)
    LTC1859_read(LTC1859_CS, adc_command[i], &adc_code);   // Read previous channel conversion (x-1) and start next one (x)
    V_CH[i] = LTC1859_code_to_voltage(adc_code, LTC1859_vref, range_low_high, uni_bipolar);
  }
  
  str = String((25 + V_CH[0]) * pow(10, decimales), 0)+String((25 + V_CH[1]) * pow(10, decimales), 0)+String((25 + V_CH[2]) * pow(10, decimales), 0) + String((25 + V_CH[3]) * pow(10,  decimales), 0)+String((25 + V_CH[4]) * pow(10, decimales), 0) + String((25 + V_CH[5]) * pow(10, decimales), 0)+ String("\n");
  */
  UART0.write(str.c_str());
  //UART0.flush();
  }
  //delay(10);
}
/*
void loop()
{ int test_t = millis();
  myTime_a = millis(); 
  for (int i = 0; i < 8; ++i) {
    LTC1859_read(LTC1859_CS, adc_command[i], &adc_code);   // Read previous channel conversion (x-1) and start next one (x)
    LTC1859_read(LTC1859_CS, adc_command[i], &adc_code);   // Read previous channel conversion (x-1) and start next one (x)
    V_CH[i] = LTC1859_code_to_voltage(adc_code, LTC1859_vref, range_low_high, uni_bipolar);
  }
  str =String((25 + V_CH[0]) * pow(10, decimales), 0)+String((25 + V_CH[1]) * pow(10, decimales), 0)+String((25 + V_CH[2]) * pow(10, decimales), 0) + String((25 + V_CH[3]) * pow(10,  decimales), 0)+String((25 + V_CH[4]) * pow(10, decimales), 0) + String((25 + V_CH[5]) * pow(10, decimales), 0)+ String("\n");

  
  if (myTime_a-myTime<=1000){
 
  
  i = i+1;
  
  }else if (myTime_a-myTime > 1000){
    UART0.println(i); 
    i = 0;
    myTime_a = 0;
    myTime= millis();
    }
   int test_p = millis();
   UART0.println(test_p-test_t);  
   UART0.write(str.c_str());
  //delay(1);
  
}*/
