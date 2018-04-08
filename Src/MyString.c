///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  MyString.c
//
//  This file contains the Get and Put strings and bytes for the STM32CubeMx environment
//
//  Change the CONSOLE_UART define in MyString.h file to the appropriate UART
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
// No warranties express or implied on the usability or suitability of the attached code and the user assumes all liability
// that arises from the use of this code.
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Includes
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "main.h"
#include "stm32f7xx_hal.h"
#include "stdbool.h"
#include "string.h"
#include "MyString.h"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  locals
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
volatile bool String_TxReady = true;       //  Ready to Transmit Flag
volatile bool String_RxReady = false;      //  Recieve data ready flag
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Defines
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Functions
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  uint16_t    String_GetString( uint8_t *Buffer )
//
//  Gets a string from the serial port and appends a null to the end of the string
//
//  Inputs:     Buffer      Pointer where to store the incoming string
//
//  Returns     Length of the string not counting the trialing null.
//
//  The following special characters are recognized by this function:
//
//  Control-C       (0x03)  Erases the buffer and exits - does not erase the screen
//  Backspace       (0x08)  Deletes the previously typed character and erases it from the screen
//  Line Feed       (0x0A)  Adds a null to the buffer and exits.
//  Carriage Return (0x0D)  Adds a null to the buffer and exits.
//  Control-R       (0x12)  Outputs a CR/LF and the string as currently in the buffer
//  Control-U       (0x15)  Erases the current buffer and erases it from the screen
//  Control-W       (0x17)  Erases the current buffer and erases it from the screen
//  Escape          (0x1B)  Erases the current buffer and erases it from the screen
//
//  Note:  This routine does no buffer length checking so it is entirely possible to overrun the buffer.
//
//         Any character typed other than those above will be masked with 0x7F and stored in the buffer
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t    String_GetString( uint8_t *Buffer )
{
    uint16_t Length = 0;
    uint16_t Byte = 1;
    uint8_t  *Ptr;
    //
    //  Looop until exited
    //
    Ptr = Buffer;
    while ( Byte != 0 )
    {
        //
        //  Get the byte
        //
        Byte = String_GetByte() & ASCII_MASK;
        //
        //  Handle special characters
        //
        switch ( Byte )
        {
            //
            //  Carriage return and line feed
            //
            case '\n':
            case '\r':
                //
                //  Add a null to the end of the buffer
                //
                *Ptr = '\0';
                //
                //  Set Byte to zero to exit the while loop
                //
                Byte = 0;
                break;
            //
            //  Backspace
            //
            case '\b':
                //
                //  If we have characters to delete
                //  then delete the last one and decrement the length
                //
                if (Ptr > Buffer )
                {
                    Ptr--;
                    Length--;
                    //
                    //  Erase the character
                    //
                    String_PutStringN("\b \b", 3);
                }
                break;
            //
            //  Control-C
            //
            //  Abort data entry
            //
            case 'c' & CONTROL_MASK:
                //
                //  Set the length to zero
                //
                Length = 0;
                //
                //  Set Byte to zero to exit the while loop
                //
                Byte = 0;
                break;
            //
            //  Contrl-R
            //  Display the buffer
            //
            case 'r' & CONTROL_MASK:
                //
                //  new line
                //
                String_PutStringN( "\n\r", 2);
                //
                //  output buffer
                //
                String_PutStringN( (char *)Buffer, Length );
                break;
            //
            //  Escape, Control-u & control-w
            //
            //  Delete all input and try again
            //
            case '[' & CONTROL_MASK:
            case 'u' & CONTROL_MASK:
            case 'w' & CONTROL_MASK:
            {
                uint16_t Counter;
                //
                //  Point to the beginning of the buffer
                //
                Ptr = Buffer;
                //
                //  Erase the line visually
                //
                for ( Counter = 0 ; Counter < Length ; Counter++ )
                {
                    String_PutStringN("\b \b", 3);
                }
                //
                //  Zero out the length of the string
                //
                Length = 0;
                break;
            }
            //
            //  Add the character to the buffer and bump the count
            //
            default:
                //
                //  Store the byte
                //
                *(Ptr++) = Byte;
                //
                //  Increment the length
                //
                Length++;
                //
                // Echo it back to the user
                //
			//String_PutByte( Byte );
                break;
        }
    }
    return Length;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  uint16_t    String_PutString( char *Buffer )
//
//  Output a string to the serial port
//
//  Inputs:     Buffer      Pointer to the null terminated string to output
//
//  Returns:    The length of the string output minus the terminating null
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t    String_PutString( char *Buffer )
{
    uint16_t    Length;
    //
    //  Get the length of the string
    //
    Length = strlen( Buffer );
    //
    //  output it
    //
    String_PutStringN( Buffer, Length );
    //
    //  Return the length
    //
    return Length;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  uint16_t    PutStringN( char *Buffer, uint16_t Length )
//
//  Output a number of bytes to the serial port
//
//  Note:  This will output the number of bytes passed in length, regardless of nulls in the buffer.
//         No buffer overrun check is performed.
//
//  Note:   This routine interfaces directly with the STM32CubeMX Hal
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t    String_PutStringN( char *Buffer, uint16_t Length )
{
    //
    //  Don't do anything if length is zero
    //
    if ( Length > 0 )
    {
        //
        //  Use DMA for output
        //
#ifdef _USE_DMA_TX_
            //
            //  Wait for transmit ready
            //
            while ( !String_TxReady );
            //
            //  Set TxReady to false beacuse we are Txing now
            //
            String_TxReady = false;
            //
            //  output the buffer using DMA
            //
            HAL_UART_Transmit_DMA( CONSOLE_UART, (uint8_t *)Buffer, Length );
#else
        //
        //  Use interrupts for output
        //
#ifdef _USE_INTERRUPTS_TX_
            //
            //  Wait for transmit ready
            //
            while ( !String_TxReady );
            //
            //  Set TxReady to false beacuse we are Txing now
            //
            String_TxReady = false;
            //
            //  output the buffer using interrupts
            //
            HAL_UART_Transmit_IT( CONSOLE_UART, (uint8_t *)Buffer, Length );
        //
        //  Use normal charactrer output
        //
#else
            //
            //  Loop through the buffer
            //
            for ( ; Length > 0 ; Length-- )
            {
                //
                //  output a character
                //
                String_PutByte( (uint8_t)*(Buffer++) );
            }
#endif  // #ifdef _USE_INTERRUPTS_TX_
#endif  // #ifdef _USE_DMA_TX_
    }
    return Length;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  uint8_t String_PutByte( uint8_t Byte )
//
//  Output a single byte to the serial port
//
//  Inputs:     Byte        Byte to be transmitted
//
//  Returns:    The byte transmitted
//
//  Note:   This function interfaces directly with the STM32CubeMX Hal
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t String_PutByte( uint8_t Byte )
{
//
//  Use DMA for output
//
#ifdef _USE_DMA_TX_
    //
    //  Wait for transmit ready
    //
    while ( !String_TxReady );
    //
    //  Set TxReady to false beacuse we are Txing now
    //
    String_TxReady = false;
    //
    //  output the buffer using DMA
    //
    HAL_UART_Transmit_DMA( CONSOLE_UART, &Byte, 1 );
#else
//
//  Use interrupts for output
//
#ifdef _USE_INTERRUPTS_TX_
    //
    //  Wait for transmit ready
    //
    while ( !String_TxReady );
    //
    //  Set TxReady to false beacuse we are Txing now
    //
    String_TxReady = false;
    //
    //  output the buffer using interrupts
    //
    HAL_UART_Transmit_IT( CONSOLE_UART, &Byte, 1 );
//
//  Use normal charactrer output
//
#else
    //
    //  output the byte normally
    //
    HAL_UART_Transmit(CONSOLE_UART, &Byte, 1, 0xFFFFFFFF);
#endif  // #ifdef _USE_INTERRUPTS_TX_
#endif  // #ifdef _USE_DMA_TX_
    return Byte;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  uint8_t String_GetByte( void )
//
//  Gets a byte from the serial port
//
// Inputs:      None
//
//  Returns:    The byte received.
//
//  Note:   This routine interfaces directly with the STM32CubeMX Hal
//
//          This function used blocked I/O for receive because DMA and Interrupt
//          I/O flush the UART first which causes it not to work as expected with
//          String_GetRxStatus.
//
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t String_GetByte( void )
{
    uint8_t Byte;
    //
    //  read the byte normally
    //
    HAL_UART_Receive(CONSOLE_UART, &Byte, 1, 0xFFFFFFFF);
    return Byte;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  bool String_GetRxStatus( void )
//
//  Returns the uart receive data register not empty flag
//
//  Input:      None
//
//  Returns:    true    Receive Data Register is not empty (data waiting)
//              false   Receive Data Register is empty
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool String_GetRxStatus( void )
{
    //
    //  Read the UART Receive Data Register Not Empty flag and convert to true or false
    //
    return ( ( __HAL_UART_GET_FLAG( CONSOLE_UART, UART_FLAG_RXNE ) ? true : false ) );
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  The following functions are HAL callback routines
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  void HAL_UART_TxCpltCallback( UART_HandleTypeDef *huart )
//
//  Transmit Complete call back from the HAL.
//
//  Input:  Huart   Pointer to HAL UART configuration structure
//
//  Returns:  nothing
//
//  Used when _USE_DMA_TX_ or _USE_INTERRUPTS_TX_ are defined
//
//  Note:   This routine is called directly by the STM32CubeMx HAL
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HAL_UART_TxCpltCallback( UART_HandleTypeDef *huart )
{
    String_TxReady = true;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart )
//
//  Transmit Complete call back from the HAL.
//
//  Input:  Huart   Pointer to HAL UART configuration structure
//
//  Used when _USE_DMA_RX_ or _USE_INTERRUPTS_RX_ are defined
//
//  Note:   This routine is called directly by the STM32CubeMx HAL
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart )
{
    String_RxReady = true;
}

