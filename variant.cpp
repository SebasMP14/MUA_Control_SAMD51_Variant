/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
/**
 * PIO_SERCOM:      First SERCOM
 * PIO_SERCOM_ALT:  Second SERCOM
 */


#include "variant.h"

// PIN DESCRIPTIONS
const PinDescription g_APinDescription[]=
{
  // Digital Pins
  // 0....16 
  // -------------------------------------------------------------------------------------------------------------------------
  { PORTB,  4, PIO_DIGITAL,   PIN_ATTR_NONE,  No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },  // D0
  { PORTB,  5, PIO_DIGITAL,   PIN_ATTR_NONE,  No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },  // D1 
  { PORTB,  6, PIO_DIGITAL,   PIN_ATTR_NONE,  No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },  // D2
  { PORTB,  7, PIO_DIGITAL,   PIN_ATTR_NONE,  No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },  // D3
  { PORTB,  8, PIO_TIMER,   PIN_ATTR_PWM_E,  No_ADC_Channel, TC4_CH0,    TC4_CH0,      EXTERNAL_INT_8 },  // D4
  { PORTB,  9, PIO_DIGITAL,   PIN_ATTR_NONE,  No_ADC_Channel, TC4_CH1,    TC4_CH1,      EXTERNAL_INT_9 },  // D5
  { PORTB, 10, PIO_COM,       PIN_ATTR_NONE,  No_ADC_Channel, TCC1_CH0,   TC5_CH0,      EXTERNAL_INT_10 }, // D6     QSPI_SCK  // PIO_DIGITAL
  { PORTB, 11, PIO_COM,       PIN_ATTR_NONE,  No_ADC_Channel, TCC1_CH1,   TC5_CH1,      EXTERNAL_INT_11 }, // D7     QSPI_CS   // PIO_DIGITAL
  { PORTB, 12, PIO_DIGITAL,   PIN_ATTR_PWM_G, No_ADC_Channel, TCC3_CH0,   NOT_ON_TIMER/*TC4_CH0*/,      EXTERNAL_INT_12 }, // D8
  { PORTB, 13, PIO_DIGITAL,   PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH1,   TC4_CH1,      EXTERNAL_INT_13 }, // D9
  { PORTA, 14, PIO_DIGITAL,   PIN_ATTR_PWM_G, No_ADC_Channel, TCC2_CH0,   TC3_CH0,      EXTERNAL_INT_14 }, // D10
  { PORTA, 15, PIO_DIGITAL,   PIN_ATTR_PWM_G, No_ADC_Channel, TCC2_CH1,   TC3_CH1,      EXTERNAL_INT_15 }, // D12
  { PORTA, 18, PIO_DIGITAL,   PIN_ATTR_PWM_F, No_ADC_Channel, TCC1_CH2,   TC3_CH0,      EXTERNAL_INT_2 },  // D12 - I2C_INT
  { PORTA, 23, PIO_DIGITAL,   PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH3,   TC4_CH1,      EXTERNAL_INT_7 },  // D13
  { PORTA, 19, PIO_DIGITAL,   PIN_ATTR_NONE,  No_ADC_Channel, TCC1_CH3,   NOT_ON_TIMER,      EXTERNAL_INT_3 },  // D14 - 3.3V_EN
  { PORTB, 22, PIO_DIGITAL,   PIN_ATTR_NONE,  No_ADC_Channel, NOT_ON_PWM, TC7_CH0,      EXTERNAL_INT_6 },  // D15 - FLASH WP
  { PORTB, 23, PIO_DIGITAL,   PIN_ATTR_NONE,  No_ADC_Channel, NOT_ON_PWM, TC7_CH1,      EXTERNAL_INT_7 },  // D16 - FLASH HOLD
  // ------------------------------------------------------------------------------------------------------------------------- 
  
  // Analog Pins 
  // 17....27
  // ------------------------------------------------------------------------------------------------------------------------- 
  { PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG,     ADC_Channel0,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },   // A0
  { PORTB,  0, PIO_ANALOG, PIN_ATTR_ANALOG,     ADC_Channel12, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 },   // A1
  { PORTB,  1, PIO_ANALOG, PIN_ATTR_ANALOG,     ADC_Channel13, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 },   // A2
  { PORTB,  2, PIO_ANALOG, PIN_ATTR_ANALOG,     ADC_Channel14, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },   // A3
  { PORTB,  3, PIO_ANALOG, PIN_ATTR_ANALOG,     ADC_Channel15, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3 },   // A4
  { PORTB,  4, PIO_ANALOG, PIN_ATTR_ANALOG_ALT, ADC_Channel6,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },   // A5
  { PORTB,  5, PIO_ANALOG, PIN_ATTR_ANALOG_ALT, ADC_Channel7,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },   // A6
  { PORTB,  6, PIO_ANALOG, PIN_ATTR_ANALOG_ALT, ADC_Channel8,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },   // A7
  { PORTB,  7, PIO_ANALOG, PIN_ATTR_ANALOG_ALT, ADC_Channel9,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },   // A8
  { PORTB,  8, PIO_ANALOG, PIN_ATTR_ANALOG,     ADC_Channel2,  TC4_CH0,    TC4_CH0,      EXTERNAL_INT_8 },   // A9
  { PORTB,  9, PIO_ANALOG, PIN_ATTR_ANALOG,     ADC_Channel3,  TC4_CH1,    TC4_CH1,      EXTERNAL_INT_9 },   // A10
  // -------------------------------------------------------------------------------------------------------------------------- 

  // DAC
  // 28
  // --------------------------------------------------------------------------------------------------------------- 
  { PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG,  DAC_Channel0,  NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },      // DAC
  // --------------------------------------------------------------------------------------------------------------- 
 
  // USB/USB Host Enable
  // 29....31
  // --------------------------------------------------------------------------------------------------------------------
  { PORTA, 27, PIO_DIGITAL, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },      // USB HOST
  { PORTA, 24, PIO_COM,     PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 },       // USB DM
  { PORTA, 25, PIO_COM,     PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },       // USB DP
  // ------------------------------------------------------------------------------------------------------------==-----

  // UART : SERCOM5
  // 32..33
  // -----------------------------------------------------------------------------------------------------------------------
  { PORTB, 31, PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, TC0_CH0, EXTERNAL_INT_15 },        // 5? SER5.0 TX
  { PORTB, 30, PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, TC0_CH0, EXTERNAL_INT_14 },        // 6? SER5.1 RX
  // -----------------------------------------------------------------------------------------------------------------------
  
  // I2C pins (SDA/SCL) : SERCOM3
  // 34..35
  // -----------------------------------------------------------------------------------------------------------------------
  { PORTA, 16, PIO_SERCOM_ALT, PIN_ATTR_PWM_F, No_ADC_Channel, TCC1_CH0, TC2_CH0, EXTERNAL_INT_0 },          // SCL  SER3.1
  { PORTA, 17, PIO_SERCOM_ALT, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH5, NOT_ON_TIMER, EXTERNAL_INT_1 },          // SDA  SER3.0
  // -----------------------------------------------------------------------------------------------------------------------

  // I2C1 (SDA1/SCL1) ///////////// CAMBIADO A UART2 -> Serial2
  // 36..37
  // -----------------------------------------------------------------------------------------------------------------------------
  { PORTA, 12, PIO_SERCOM, PIN_ATTR_PWM_G, No_ADC_Channel, TCC1_CH2, TC2_CH0, EXTERNAL_INT_12 },             // SCL1 - RX [SER4.1] - TX [SER2.0]
  { PORTA, 13, PIO_SERCOM, PIN_ATTR_PWM_G, No_ADC_Channel, TCC1_CH3, NOT_ON_TIMER, EXTERNAL_INT_13 },             // SDA1 - TX [SER4.0] - RX [SER2.1]
  // -----------------------------------------------------------------------------------------------------------------------------
  
  // CAN BUS (CANRX/CANTX)
  // 38..39
  // ---------------------------------------------------------------------------------------------------------------------
  { PORTB, 14, PIO_AC_CLK, PIN_ATTR_PWM_F, No_ADC_Channel, TCC4_CH0, TC5_CH0, EXTERNAL_INT_14 },             // CAN TXO
  { PORTB, 15, PIO_AC_CLK, PIN_ATTR_PWM_F, No_ADC_Channel, TCC4_CH1, TC5_CH1, EXTERNAL_INT_15 },             // CAN RXI
  // ---------------------------------------------------------------------------------------------------------------------


  // I2S (SDO, SDI, FS, SCK)
  // 40....44
  // ---------------------------------------------------------------------------------------------------------------------
  { PORTA, 21, PIO_DIGITAL, PIN_ATTR_PWM_F,   No_ADC_Channel, TCC0_CH1,   TC7_CH0,      EXTERNAL_INT_5 },    // SDO
  { PORTA, 22, PIO_DIGITAL, PIN_ATTR_PWM_F,   No_ADC_Channel, TCC0_CH2,   TC4_CH0,      EXTERNAL_INT_6 },    // SDI
  { PORTB, 16, PIO_DIGITAL, PIN_ATTR_PWM_G,   No_ADC_Channel, TCC3_CH0,   TC6_CH0,      EXTERNAL_INT_0 },   // SCK 
  { PORTB, 17, PIO_DIGITAL, PIN_ATTR_PWM_F,   No_ADC_Channel, TCC3_CH1,   TC6_CH1,      EXTERNAL_INT_1 },   // MCLK 
  { PORTA, 20, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },    // FS
  // ---------------------------------------------------------------------------------------------------------------------

  // // SPI (COPI, CIPO, SCK, CS) // ORIGINAL
  // // 45....48
  // // ---------------------------------------------------------------------------------------------------------------------
  // { PORTA,  4, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },// COPI
  // { PORTA,  5, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },// SCK
  // { PORTA,  6, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },// CIPO
  // { PORTA,  7, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },// CS
  // // ---------------------------------------------------------------------------------------------------------------------

  // SPI (MOSI, SCK, MISO, CS) : SERCOM0  // AGREGADO/MODIFICADO Sebas
  // 45....48
  // ---------------------------------------------------------------------------------------------------------------------
  { PORTA,  4, PIO_SERCOM_ALT, PIN_ATTR_PWM_E, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },// COPI
  { PORTA,  5, PIO_SERCOM_ALT, PIN_ATTR_PWM_E, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },// SCK
  { PORTA,  6, PIO_SERCOM_ALT, PIN_ATTR_PWM_E, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },// CIPO
  { PORTA,  7, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 },// CS
  // -----------

  // // SPI (COPI, CIPO, SCK, CS)  // Luka
  // // 45....48
  // // Ref : https://github.com/adafruit/ArduinoCore-samd/blob/master/variants/metro_m4/variant.cpp
  // // ---------------------------------------------------------------------------------------------------------------------
  // { PORTA,  4, PIO_SERCOM_ALT, PIO_SERCOM, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 },    // COPI
  // { PORTA,  5, PIO_SERCOM_ALT, PIO_SERCOM, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },    // SCK
  // { PORTA,  6, PIO_SERCOM_ALT, PIO_SERCOM, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },    // CIPO
  // { PORTA,  7, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, // D3 
  // // ---------------------------------------------------------------------------------------------------------------------

  // SPI1 (COPI1, CIPO1, SCK1, CS1) // ORIGINAL
  // 49....52
  // -----------------------------------------------------------------------------------------------------------------------
  // { PORTA,  9,  PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },// COPI1 SER2.0
  // { PORTA,  8,  PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI },// SCK1  SER2.1
  // { PORTA, 10,  PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },// CIPO1 SER2.2
  // { PORTA, 11,  PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },// CS1   SER2.3
  // -----------------------------------------------------------------------------------------------------------------------
 
  // QSPI (D0, D1, D2, D3) // AGREGADO/MODIFICADO
  // 49....52
  // -----------------------------------------------------------------------------------------------------------------------
  { PORTA,  8,  PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI },// SCK1  SER2.1
  { PORTA,  9,  PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 },// COPI1 SER2.0
  { PORTA, 10,  PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 },// CIPO1 SER2.2
  { PORTA, 11,  PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 },// CS1   SER2.3
// -----------------------------------------------------------------------------------------------------------------------
 
  // AREF
  // 53 
  // ---------------------------------------------------------------------------------------------------------------------
  { PORTA,  3, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel1,   NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // DAC/VREFP
  // ---------------------------------------------------------------------------------------------------------------------

  // TC utilizado con PA15 -> TC3_CH1
  // 54
  // -----------------------------------------------------------------------------------------------------------------------
  { PORTA, 1, PIO_DIGITAL, PIN_ATTR_NONE,   No_ADC_Channel, TC2_CH1,    TC2_CH1,      EXTERNAL_INT_1},
  // -----------------------------------------------------------------------------------------------------------------------

} ;

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TCC3, TCC4, TC0, TC1, TC2, TC3, TC4, TC5, TC6 };
const uint32_t GCLK_CLKCTRL_IDs[TCC_INST_NUM+TC_INST_NUM] = { TCC0_GCLK_ID, TCC1_GCLK_ID, TCC2_GCLK_ID, TCC3_GCLK_ID, TCC4_GCLK_ID, TC0_GCLK_ID, TC1_GCLK_ID, TC2_GCLK_ID, TC3_GCLK_ID, TC4_GCLK_ID, TC5_GCLK_ID, TC6_GCLK_ID };

// Multi-serial objects instantiation
SERCOM sercom0( SERCOM0 ) ;
SERCOM sercom1( SERCOM1 ) ;
SERCOM sercom2( SERCOM2 ) ;
SERCOM sercom3( SERCOM3 ) ;
SERCOM sercom4( SERCOM4 ) ;
SERCOM sercom5( SERCOM5 ) ;

Uart Serial1( &sercom5, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX ) ;
Uart Serial2( &sercom2, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX ) ;

void SERCOM5_0_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM5_1_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM5_2_Handler()
{
  Serial1.IrqHandler();
}
void SERCOM5_3_Handler()
{
  Serial1.IrqHandler();
}

void SERCOM2_0_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM2_1_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM2_2_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM2_3_Handler()
{
  Serial2.IrqHandler();
}




// 27/09
// Resolving sparkfun_samd51_micromod dependencies...
// Platform Manager: Updating atmelsam @ 8.2.1
// Platform Manager: Removing atmelsam @ 8.2.1
// Platform Manager: atmelsam@8.2.1 has been removed!      
// Platform Manager: Installing platformio/atmelsam @ 8.3.0
// Downloading  [####################################]  100%
// Unpacking  [####################################]  100%
// Platform Manager: atmelsam@8.3.0 has been installed!
// PS C:\Users\Pc\Documents\SAMD_Prueba1> 