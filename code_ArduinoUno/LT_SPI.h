//! @todo Review this document.
/*!
LT_SPI: Routines to communicate with ATmega328P's hardware SPI port.

REVISION HISTORY
$Revision: 3018 $
$Date: 2014-12-01 15:53:20 -0800 (Mon, 01 Dec 2014) $

Copyright (c) 2013, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.
*/

/*! @file
    @ingroup LT_SPI
    Library Header File for LT_SPI: Routines to communicate with ATmega328P's hardware SPI port.
*/

#ifndef LT_SPI_H
#define LT_SPI_H
#include <SPI.h>

#include <stdint.h>


void spi_init();
//! Reads and sends a byte
//! @return void
void spi_transfer_byte(uint8_t cs_pin,      //!< Chip select pin
                       uint8_t tx,          //!< Byte to be transmitted 
                       uint8_t *rx          //!< Byte to be received
                      );

//! Reads and sends a word
//! @return void
void spi_transfer_word(uint8_t cs_pin,      //!< Chip select pin
                       uint16_t tx,         //!< Byte to be transmitted 
                       uint16_t *rx         //!< Byte to be received
                      );

//! Reads and sends a byte array
//! @return void
void spi_transfer_block(uint8_t cs_pin,     //!< Chip select pin
                        uint8_t *tx,        //!< Byte array to be transmitted 
                        uint8_t *rx,        //!< Byte array to be received
                        uint8_t length      //!< Length of array
                       );


//! Disable the SPI hardware port
void spi_disable();

//! Write a data byte using the SPI hardware
void spi_write(int8_t data  //!< Byte to be written to SPI port
              );

//! Read and write a data byte using the SPI hardware
//! @return the data byte read
int8_t spi_read(int8_t data //!< The data byte to be written
               );

#endif  // LT_SPI_H
