/*****************************************************************************
 @Project	: SEM2306 Lab Assignment
 @File 		: i2c_sensor.c
 @Details : 
 @Author	: FongFH
 @Hardware: Tiva LaunchPad
 
 --------------------------------------------------------------------------
 @Revision	:
  Ver  	Author    	Date        	Changes
 --------------------------------------------------------------------------
   1.0  Fong FH     4 Jun 21  		Initial Release
   
******************************************************************************/

#include <Common.h>
#include "BSP.h"
#include "IRQ.h"
#include "i2c.h"
#include "i2c_sensors.h"
#include <math.h>

static PI2C_HANDLE g_pI2cHandle;
extern volatile BOOL g_bI2C0IsBusy;

static uint8_t 	rawDATA[6];
static uint32_t g_bHUMIDITY;
static uint32_t g_bTEMPERATURE;
/*****************************************************************************
 Local function
*****************************************************************************/
void delay_ms( int ms );

/*****************************************************************************
 Implementation
******************************************************************************/
void AHT10_Init( PI2C_HANDLE pHandle, AHT10 *pAHT10 )
{
	uint8_t data[2];
	
  g_pI2cHandle = pHandle;
	pAHT10->I2C_adr = AHT10_I2C_ADDRESS;
	
	data[0] = AHT10_INIT_CAL_ENABLE;	
	data[1] = AHT10_DATA_NOP;		// 0x00
	I2CWrite( g_pI2cHandle, pAHT10->I2C_adr, AHT10_INIT_CMD, data, 2U);  
  g_bI2C0IsBusy = TRUE;
	while( TRUE == g_bI2C0IsBusy);
}

void AHT10Trigger( PI2C_HANDLE pHandle, AHT10 *pAHT10 )
{
	uint8_t triggerdata[3];
	
  g_pI2cHandle = pHandle;	
	triggerdata[0] = AHT10_START_MEASURMENT_CMD;		// 0xAC
	triggerdata[1] = AHT10_DATA_MEASURMENT_CMD;			// 0x33
	triggerdata[2] = AHT10_DATA_NOP;								// 0x00
	I2CWrite( g_pI2cHandle, pAHT10->I2C_adr, AHT10_START_MEASURMENT_CMD, triggerdata, 3U);  
  g_bI2C0IsBusy = TRUE;
	while( TRUE == g_bI2C0IsBusy);
}

void AHT10ReadRawdata( PI2C_HANDLE pHandle, AHT10 *pAHT10 )
{
	I2CRead(g_pI2cHandle, pAHT10->I2C_adr, 0, rawDATA, 6U);
	g_bI2C0IsBusy = TRUE;
	while( TRUE == g_bI2C0IsBusy );
	
	g_bHUMIDITY &= ~0xFFFFFFFF;//clear the data first
	g_bHUMIDITY |= (rawDATA[1]<<16) + (rawDATA[2]<<8) + rawDATA[3];//concatenate the data
	g_bHUMIDITY = g_bHUMIDITY>>4;
	
	g_bTEMPERATURE &= ~0xFFFFFFFF;//clear the data first
	g_bTEMPERATURE = (rawDATA[3]<<16) + (rawDATA[4]<<8) + rawDATA[5];//concatenate the data
	g_bTEMPERATURE &= ~(0xF<<20);
	
	float humid = (float)(g_bHUMIDITY);//cast the type to float
	float temp	= (float)(g_bTEMPERATURE);//cast the type to float
	
	pAHT10->fHumidity = (humid/0x00100000) * 100;//formula
	pAHT10->fTemperature = ((temp/0x00100000) * 200) - 50;//formula
}

void I2CExpander_Init (PI2C_HANDLE pHandle, MCP23017 *pMCP23017)
{
	uint8_t data[2] = {0,0};
	
	g_pI2cHandle = pHandle;
	
	pMCP23017->I2C_addr = MCP23017_ADDR;
	data[0] = 0x00;	// 16-bit mode (BANK = 0), Sequential Operation (SEQOP = 0)
	I2CWrite( g_pI2cHandle, pMCP23017->I2C_addr, MCP23017_IOCONA, data, 1U);  
  g_bI2C0IsBusy = TRUE;
	while( TRUE == g_bI2C0IsBusy);
	
	data[0] = 0x00;	// data = 0x00 (port A as output)
	data[1] = 0x00;	// data = 0x00 (port B as output)
	I2CWrite( g_pI2cHandle, MCP23017_ADDR, MCP23017_IODIRA, data, 2U);  

  g_bI2C0IsBusy = TRUE;
	while( TRUE == g_bI2C0IsBusy);
}


/* function to write to the MCP23017 Output Latch registers    */
/* outputs from the MC23017 will drive the 7-segment displays  */
void write_I2CExpander (PI2C_HANDLE pHandle, MCP23017 *pMCP23017 )
{
	uint8_t leftD = pMCP23017->Digit1;//store digit 1 data to left d
	uint8_t rightD = pMCP23017->Digit2;//store digit 2 data to right d
	
	switch(leftD)
	{
		case 0: leftD = 0xC0; break;//if leftD = 0, assign 0xC0 to leftD to display 0 to 7 seg
		case 1: leftD = 0xF9; break;//if leftD = 1, assign 0xF9 to leftD to display 1 to 7 seg
		case 2: leftD = 0xA4; break;//if leftD = 2, assign 0xA4 to leftD to display 2 to 7 seg
		case 3: leftD = 0xB0; break;//if leftD = 3, assign 0xB0 to leftD to display 3 to 7 seg
		case 4: leftD = 0x99; break;//if leftD = 4, assign 0x99 to leftD to display 4 to 7 seg
		case 5: leftD = 0x92; break;//if leftD = 5, assign 0x92 to leftD to display 5 to 7 seg
		case 6: leftD = 0x82; break;//if leftD = 6, assign 0x82 to leftD to display 6 to 7 seg
		case 7: leftD = 0xF8; break;//if leftD = 7, assign 0xF8 to leftD to display 7 to 7 seg
		case 8: leftD = 0x80; break;//if leftD = 8, assign 0x80 to leftD to display 8 to 7 seg
		case 9: leftD = 0x98; break;//if leftD = 9, assign 0x98 to leftD to display 9 to 7 seg
	}
	
		switch(rightD)
	{
		case 0: rightD = 0xC0; break;//if rightD = 0, assign 0xC0 to rightD to display 0 to 7 seg
		case 1: rightD = 0xF9; break;//if rightD = 1, assign 0xF9 to rightD to display 1 to 7 seg
		case 2: rightD = 0xA4; break;//if rightD = 2, assign 0xA4 to rightD to display 2 to 7 seg
		case 3: rightD = 0xB0; break;//if rightD = 3, assign 0xB0 to rightD to display 3 to 7 seg
		case 4: rightD = 0x99; break;//if rightD = 4, assign 0x99 to rightD to display 4 to 7 seg
		case 5: rightD = 0x92; break;//if rightD = 5, assign 0x92 to rightD to display 5 to 7 seg
		case 6: rightD = 0x82; break;//if rightD = 6, assign 0x82 to rightD to display 6 to 7 seg
		case 7: rightD = 0xF8; break;//if rightD = 7, assign 0xF8 to rightD to display 7 to 7 seg
		case 8: rightD = 0x80; break;//if rightD = 8, assign 0x80 to rightD to display 8 to 7 seg
		case 9: rightD = 0x98; break;//if rightD = 9, assign 0x98 to rightD to display 9 to 7 seg
	}
	
	uint8_t data[2] = {leftD,rightD};
	I2CWrite( g_pI2cHandle, pMCP23017->I2C_addr, MCP23017_OLATA, data, 2U);  
	g_bI2C0IsBusy = TRUE;
	while( TRUE == g_bI2C0IsBusy);

}


void delay_ms( int ms )
{
	volatile int cnt = (SystemCoreClock/1000)*ms;
	
	while( cnt-- )
	{
		__NOP();
	}
}
