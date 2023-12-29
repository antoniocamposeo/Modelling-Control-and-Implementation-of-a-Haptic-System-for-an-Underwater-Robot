#include <stdint.h>
#include "Linduino.h"
#include <SPI.h>
#include "LowPassFilter.h"

void spi_init(){
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV32);
  }
  
// Reads and sends a byte
// Return 0 if successful, 1 if failed
void spi_transfer_byte(uint8_t cs_pin, uint8_t tx, uint8_t *rx)
{
    output_low(cs_pin);                 //! 1) Pull CS low

    *rx = SPI.transfer(tx);             //! 2) Read byte and send byte
    
    output_high(cs_pin);                //! 3) Pull CS high
}

// Reads and sends a word
// Return 0 if successful, 1 if failed
void spi_transfer_word(uint8_t cs_pin, uint16_t tx, uint16_t *rx)
{
    union
    {
        uint8_t b[2];
        uint16_t w;
    } data_tx;
    
    union
    {
        uint8_t b[2];
        uint16_t w;
    } data_rx;

    data_tx.w = tx;

    output_low(cs_pin);                         //! 1) Pull CS low

    data_rx.b[1] = SPI.transfer(data_tx.b[1]);  //! 2) Read MSB and send MSB
    data_rx.b[0] = SPI.transfer(data_tx.b[0]);  //! 3) Read LSB and send LSB

    *rx = data_rx.w;

    output_high(cs_pin);                        //! 4) Pull CS high
}

// Reads and sends a byte array
void spi_transfer_block(uint8_t cs_pin, uint8_t *tx, uint8_t *rx, uint8_t length)
{
    int8_t i;
    
    output_low(cs_pin);                 //! 1) Pull CS low
    
    for(i=(length-1);  i >= 0; i--)
        rx[i] = SPI.transfer(tx[i]);    //! 2) Read and send byte array
    
    output_high(cs_pin);                //! 3) Pull CS high
}

void LowPassFilter_Init(LowPassFilter *filter, float cutoffFreq, float samplingFreq)
{
    /* Init Input Buffer */
    filter->input = 0.0f;

    /* Calculate Alpha Coefficient */
    filter->alpha = (float)__ALPHA(cutoffFreq, (float)__DELTAT(samplingFreq));

    /* Clear Filter Output */
    filter->output[0] = 0.0f;
    filter->output[1] = 0.0f;
}

float LowPassFilter_Update(LowPassFilter *filter, float input)
{
    /* Update Variables */
    filter->output[1] = filter->input;
    filter->input = input;

    /* Calculate Output */
    filter->output[0] = (filter->alpha * filter->input) + ((1.0f - filter->alpha) * filter->output[1]);

    return filter->output[0];
}
