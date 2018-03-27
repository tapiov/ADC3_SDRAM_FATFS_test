#ifndef _MY_STRING_H_
#define _MY_STRING_H_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  MyString.h
//
//  This file contains all of the defines, global variables and externs needed by the MyStrings.c module
//
//
// This source code is copyright 2017 (C) Katz Programming and Consulting.  All rights for use are granted as long as
// this copyright message is included.
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU Lesser General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Globals
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Module locals
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Defines
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define CONTROL_MASK    (0x1f)      //  ASCII mask for control characters
#define ASCII_MASK      (0x7f)      //  ASCII mask for standard character set
//
//  The method of output (normal I/O, Inhterrupt driven I/O or DMA driven I/O is determined by the
//  following defines found in the MyString.h file:
//
//  _USE_DMA_RX_        Receive will be done using DMA
//  _USE_DMA_TX_        Transmit will be done using DMA
//  _USR_INTERRRUPTS_RX Receive will be done using interrupts
//  _USR_INTERRRUPTS_TX Transmit will be done uisng interrupts
//
//  If neither of the Rx or Tx defines are defined normal I/O will be used.
//
//  Note:  It is the resonsibility of STM32CubeMx to have the appropriate drivers loaded and configured and the
//         programmer to insure that the appropriate initialization is done in main.
//
//
//  Note:  STM32CubeMx must be configured for DMA or interrupts before changing this.
//
//#define _USE_INTERRUPTS_RX_
//#define _USE_DMA_RX_
//#define _USE_INTERRUPTS_TX_
//#define _USE_DMA_TX_
//
//  Change this define to the uart you plan on using.
//
#define CONSOLE_UART_LOCAL    (huart1)
#define CONSOLE_UART          (&CONSOLE_UART_LOCAL)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Externs
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
extern UART_HandleTypeDef CONSOLE_UART_LOCAL;   //  HAL UART structure to be used for input/output
extern volatile bool String_RxReady;                   //  Serial Receive Data avaialble flag
extern volatile bool String_TxReady;                   //  Serial Transmit complete flag
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Prototypes
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t String_GetString( uint8_t *Buffer );
uint16_t String_PutString( char *Buffer );
uint16_t String_PutStringN( char *Buffer, uint16_t Length );
uint8_t  String_PutByte( uint8_t Byte );
uint8_t  String_GetByte( void );
bool     String_GetRxStatus( void );
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif  // #ifndef _MY_STRING_H_

