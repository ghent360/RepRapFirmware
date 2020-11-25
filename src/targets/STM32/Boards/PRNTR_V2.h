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
    //Thermistors
    {PA_0, PinCapability::ainrw, "e0temp,t0"},
    {PA_1, PinCapability::ainrw, "e1temp,t1"},
    {PA_2, PinCapability::ainrw, "e2temp,t2"},
    {PA_3, PinCapability::ainrw, "bedtemp,t3"},
    {PA_4, PinCapability::ainrw, "t4"},
    {PA_5, PinCapability::ainrw, "t5"},

    //Endstops
    {PB_1, PinCapability::rw, "xstop,x-stop"},
    {PC_14, PinCapability::rw, "ystop,y-stop"},
    {PE_7, PinCapability::rw, "zstop,z-stop"},
    {PC_4, PinCapability::rw, "xstart,x-start"},
    {PC_5, PinCapability::rw, "ystart,y-start"},
    {PB_0, PinCapability::rw, "zstart,z-start"},
    {PE_8, PinCapability::rw, "e0stop,e0det"},
    {PE_9, PinCapability::rw, "e1stop,e1det"},
	
    //Heaters and Fans (Big and Small Mosfets}
    {PA_8,  PinCapability::wpwm, "bed,hbed" },
    {PA_15,  PinCapability::wpwm, "e0heat,he0" },
    {PB_4,  PinCapability::wpwm, "e1heat,he1" },
    {PB_8,  PinCapability::wpwm, "e2heat,he2" },
    {PB_9,  PinCapability::wpwm, "fan0,fan" },
    {PC_7,  PinCapability::wpwm, "fan1" },
    {PE_5,  PinCapability::wpwm, "fan2" },
    {PE_6,  PinCapability::wpwm, "fan3" },
    //{PD_15,  PinCapability::wpwm, "fan4" },

    //Servos
    {PD_15,  PinCapability::rwpwm, "servo0" },
	
    //EXP1
    {PB_15, PinCapability::rw, "enc_a,PB15"},
    {PB_14, PinCapability::rw, "enc_b,PB14"},
    {PB_12, PinCapability::rw, "enc_btn,PB12"},
    {PB_11, PinCapability::rwpwm, "beep, PB11"},
    {PE_10, PinCapability::rw, "lcd_e, PE10"},
    {PE_11, PinCapability::rw, "lcd_rs, PE11"},
    {PE_12, PinCapability::rw, "lcd_d7, PE12"},
    {PE_13, PinCapability::rw, "lcd_d6, PE13"},
    {PE_14, PinCapability::rw, "lcd_d5, PE14"},
    {PE_15, PinCapability::rw, "lcd_d4, PE15"},

    {PD_3, PinCapability::rw, "sd_detect, PD3"},
    {PC_15, PinCapability::rw, "alarm, PC15"},

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
	{PA_6, PinCapability::rw, "miso1,PA6"},
    {PA_7, PinCapability::rw, "mosi1,PA7"},
    {PB_3, PinCapability::rw, "sck1,PB3"},
	{PC_2, PinCapability::rw, "miso2,PC2"},
    {PC_3, PinCapability::rw, "mosi2,PC3"},
    {PB_13, PinCapability::rw, "sck2,PB13"},
	{PB_2, PinCapability::rw, "X-CS,PB2"},
    {PC_13, PinCapability::rw, "Y-CS,PC13"},
	{PE_4, PinCapability::rw, "Z-CS,PE4"},
    {PE_3, PinCapability::rw, "E0-CS,PE3"},
    {PE_2, PinCapability::rw, "E1-CS,PE2"},
    {PE_1, PinCapability::rw, "E2-CS,PE1"},
    {PD_8, PinCapability::rw, "EXP-CS,PD8"},

	
	//I2C
	{PB_7, PinCapability::rw, "i2c-sda,PB7"},
	{PB_6, PinCapability::rw, "i2c-scl,PB6"},
	
	//UART
	{PA_9, PinCapability::rw, "serial0-tx,PA9"},
	{PA_10, PinCapability::rw, "serial0-rx,PA10"},
	{PD_5, PinCapability::rw, "serial1-tx,PD5"},
	{PD_6, PinCapability::rw, "serial1-rx,PD6"},
    {PB_10, PinCapability::rw, "st-uart1,PB10"},
    {PC_6, PinCapability::rw, "st-uart2,PC6"},
};

constexpr BoardDefaults prntr_v2_Defaults = {
	6,											// Number of drivers
    {PC_0, PC_0,  PC_0, PC_0, PC_0, PC_0},   	//enablePins
    {PE_0, PB_5, PD_7, PD_4, PD_1, PD_0},	    //stepPins
    {PD_14, PD_13, PD_12, PD_11, PD_10, PD_9},  //dirPins
#if TMC_SOFT_UART
    {PB_10, PB_10, PB_10, PB_10, PC_6, PC_6},   //uartPins
    6,                                      	// Smart drivers
#endif
    0                                       	//digiPot Factor
};

#endif // PRNTR_VE_H
