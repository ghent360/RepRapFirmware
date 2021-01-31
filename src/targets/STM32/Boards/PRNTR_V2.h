#ifndef PRNTR_VE_H
#define PRNTR_VE_H

#include "../Pins_STM32.h"

// List of assignable pins and their mapping from names to MPU ports. This is indexed by logical pin number.
// The names must match user input that has been converted to lowercase and had _ and - characters stripped out.
// Aliases are separate by the , character.
// If a pin name is prefixed by ! then this means the pin is hardware inverted. The same pin may have names for both the inverted and non-inverted cases,
// for example the inverted heater pins on the expansion connector are available as non-inverted servo pins on a DFueX.

constexpr PinEntry PinTable_PRNTR_V2[] =
{
#ifdef STM32F4    
    //Thermistors
    {PA0, PinCapability::ainrw, "e0temp,t0"},
    {PA1, PinCapability::ainrw, "e1temp,t1"},
    {PA2, PinCapability::ainrw, "e2temp,t2"},
    {PA3, PinCapability::ainrw, "bedtemp,t3"},
    {PA4, PinCapability::ainrw, "t4"},
    {PA5, PinCapability::ainrw, "t5"},

    //Endstops
    {PB1, PinCapability::rw, "xstop,x-stop"},
    {PC14, PinCapability::rw, "ystop,y-stop"},
    {PE7, PinCapability::rw, "zstop,z-stop"},
    {PC4, PinCapability::rw, "xstart,x-start"},
    {PC5, PinCapability::rw, "ystart,y-start"},
    {PB0, PinCapability::rw, "zstart,z-start"},
    {PE8, PinCapability::rw, "e0stop,e0det"},
    {PE9, PinCapability::rw, "e1stop,e1det"},
	
    //Heaters and Fans (Big and Small Mosfets}
    {PA8,  PinCapability::wpwm, "bed,hbed" },
    {PA15,  PinCapability::wpwm, "e0heat,he0" },
    {PB4,  PinCapability::wpwm, "e1heat,he1" },
    {PB8,  PinCapability::wpwm, "e2heat,he2" },
    {PB9,  PinCapability::wpwm, "fan0,fan" },
    {PC7,  PinCapability::wpwm, "fan1" },
    {PE5,  PinCapability::wpwm, "fan2" },
    {PE6,  PinCapability::wpwm, "fan3" },
    //{PD_15,  PinCapability::wpwm, "fan4" },

    //Servos
    {PD15,  PinCapability::rwpwm, "servo0" },
	
    //EXP1
    {PB15, PinCapability::rw, "enc_a,PB15"},
    {PB14, PinCapability::rw, "enc_b,PB14"},
    {PB12, PinCapability::rw, "enc_btn,PB12"},
    {PB11, PinCapability::rwpwm, "beep, PB11"},
    {PE10, PinCapability::rw, "lcd_e, PE10"},
    {PE11, PinCapability::rw, "lcd_rs, PE11"},
    {PE12, PinCapability::rw, "lcd_d7, PE12"},
    {PE13, PinCapability::rw, "lcd_d6, PE13"},
    {PE14, PinCapability::rw, "lcd_d5, PE14"},
    {PE15, PinCapability::rw, "lcd_d4, PE15"},

    {PD3, PinCapability::rw, "sd_detect, PD3"},
    {PC15, PinCapability::rw, "alarm, PC15"},

#if 0    
	//Wifi
	{PG_0, PinCapability::rwpwm, "wifi1,PG0"},
	{PG_1, PinCapability::rwpwm, "wifi2,PG1"},
	{PC_7, PinCapability::rwpwm, "wifi3,PC7"},
	{PC_6, PinCapability::rwpwm, "wifi4,PC6"},
	{PF_14, PinCapability::rwpwm, "wifi5,PF14"},
	{PF_15, PinCapability::rwpwm, "wifi6,PF15"},
#endif

	//SPI
	{PA6, PinCapability::rw, "miso1,PA6"},
    {PA7, PinCapability::rw, "mosi1,PA7"},
    {PB3, PinCapability::rw, "sck1,PB3"},
	{PC2, PinCapability::rw, "miso2,PC2"},
    {PC3, PinCapability::rw, "mosi2,PC3"},
    {PB13, PinCapability::rw, "sck2,PB13"},
	{PB2, PinCapability::rw, "X-CS,PB2"},
    {PC13, PinCapability::rw, "Y-CS,PC13"},
	{PE4, PinCapability::rw, "Z-CS,PE4"},
    {PE3, PinCapability::rw, "E0-CS,PE3"},
    {PE2, PinCapability::rw, "E1-CS,PE2"},
    {PE1, PinCapability::rw, "E2-CS,PE1"},
    {PD8, PinCapability::rw, "EXP-CS,PD8"},

	
	//I2C
	{PB7, PinCapability::rw, "i2c-sda,PB7"},
	{PB6, PinCapability::rw, "i2c-scl,PB6"},
	
	//UART
	{PA9, PinCapability::rw, "serial0-tx,PA9"},
	{PA10, PinCapability::rw, "serial0-rx,PA10"},
	{PD5, PinCapability::rw, "serial1-tx,PD5"},
	{PD6, PinCapability::rw, "serial1-rx,PD6"},
    {PB10, PinCapability::rw, "st-uart1,PB10"},
    {PC6, PinCapability::rw, "st-uart2,PC6"},
#endif // STM32F4    
};

constexpr BoardDefaults prntr_v2_Defaults = {
#ifdef STM32F4    
	6,											// Number of drivers
    {PC0, PC0, PC0, PC0, PC0, PC0},   	//enablePins
    {PE0, PB5, PD7, PD4, PD1, PD0},	    //stepPins
    {PD14, PD13, PD12, PD11, PD10, PD9},  //dirPins
#if TMC_SOFT_UART
    {PB10, PB10, PB10, PB10, PC6, PC6},   //uartPins
    6,                                      	// Smart drivers
#endif
    0                                       	//digiPot Factor
#endif    
};

#endif // PRNTR_VE_H
