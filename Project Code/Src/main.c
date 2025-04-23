/*****************************************************************************
 @Project		: SEM2306 Lab Assignment
 @File 			: main.c
 @Details  	: This program will simulate a digital home lock security system with some security features added.
								Security features added:
							- User will be allowed 3 tries to key in the correct passcode. The correct passcode is defined as 138.
							- User will also be given 60 seconds to key in the correct passcode.
							- If user fail to key in correct passcode with 3 tries, the 7 segment will count down to 10s while the keypad is locked.
							- After 10s, user will be able to key in the passcode again and the user will be given 60s again to key in the correct passcode.
							- There is also a temperature sensor to alert the authorities when there is a temperature drop or temperature rise. This is simulated
								by using the slider to detect for temperature anomaly. Temperature drop might mean that there is someone that is trying to break into 
								the safe by using chemical means to spoil and freeze the locking mechanism of the safe. Temperature rise may mean that someone is 
								trying to break into the safe by using arson methods such as wielding the safe open. 
							- User can enter the password by pressing the 3 digit passcode on the keypad and pressing the hashtag button. 
							- Correct password will result in the safe to be opened. Wrong password will result in the LCD prompting the user to key in again.
							- To close back the safe, press switch 2. 
							
 @Author		: Nur Fatin Farzana and Muhd As-Siddique
 @Hardware	: Tiva LaunchPAd TM4C123
 
 --------------------------------------------------------------------------
 @Revision	:
  Ver  	Author    	Date        	Changes
 --------------------------------------------------------------------------
   1.0       			6 August 21  		Initial Release
******************************************************************************/
#include "Common.h"
#include "Hal.h"
#include "BSP.h"
#include "LED.h"
#include "IRQ.h"
#include "spim.h"
#include "LCD_ST7735R.h"
#include "gui.h"
#include "i2c.h"
#include "i2c_sensors.h"
#include "timer.h"
#include "SIT.c" 

/*****************************************************************************
 Define
******************************************************************************/
#define LCD_BUF_SIZE					4096			 /* LCD buffer size  						*/
#define LCD_UPDATE_MS					10U				 /* LCD update period in ms   	*/
#define KEYPAD_UPDATE_MS			50U        /* keypad update period in ms  */
#define BUZZER_MS							100U		 	 /* buzzer beep duration in ms  */
#define SHORT_BEEP						30U        /* short beep duration in ms   */
#define I2C_UPDATE_MS					1000U			 /* for I21C module 				 	  */
#define AHT10_UPDATE_MS				5000U		   /* for I21C module             */
#define SW_DEBOUNCE_INTRV 		10U        /* SW debounce period in ms    */

#define ADC0_SS0_Start() ((ADC0->PSSI |= ADC_PSSI_SS0) ) /* Processor trigger*/
#define ADC0_GET_FIFO() (ADC0->SSFIFO0)									/* Macro to get FIFO for ADC*/


#define ADC0_BUSY() ((ADC0->ACTSS & ADC_ACTSS_BUSY)==0x01? TRUE:FALSE)		/* Macro to check if ADC is ready */
#define ADC_UPDATE_MS 10U																									/* ADC update period in ms    */

/* defines states in the FSM array  */
#define S6  &fsm[0] // state 6
#define S2  &fsm[1] // state 2
#define S3  &fsm[2] // state 3
#define S1  &fsm[3] // state 1
#define S9  &fsm[4] // state 9
#define S8  &fsm[5] // state 8
#define S12 &fsm[6] // state 12
#define S4  &fsm[7] // state 4

#define MOTOR_STEP_TIME  	10  		/* time in ms between steps */
#define MOTOR_ANGLE				180 		/* motor angle to turn */
#define FRAME_COUNT				2U

/*****************************************************************************
 Type definition
******************************************************************************/
/*****************************************************************************
 Global Variables
******************************************************************************/
void GUI_AppDraw( BOOL bFrameStart );

/* structure represents a State of the FSM   */ 
struct State
{
  uint8_t Out;            
  const struct State *Next; 
};
typedef const struct State StateType;

StateType fsm[8]=																								//clockwise
{
  {0x60, S2 },   /* step 0, current state S6  */
	{0x20, S3 },   /* step 1, current state S2  */
  {0x30, S1 },   /* step 2, current state S3  */
	{0x10, S9 },   /* step 3, current state S1  */
	{0x90, S8 },   /* step 4, current state S9  */
	{0x80, S12 },  /* step 5, current state S8  */
	{0xC0, S4 },   /* step 6, current state S12 */
	{0x40, S6 },   /* step 7, current state S4  */
};

StateType afsm[8]=																							//anticlockwise
{
  {0x40, S6 },   /* step 0, current state S6  */
	{0xC0, S4 },   /* step 1, current state S2  */
  {0x80, S12 },  /* step 2, current state S3  */
	{0x90, S8 },   /* step 3, current state S1  */
	{0x10, S9 },   /* step 4, current state S9  */
	{0x30, S1 },   /* step 5, current state S8  */
	{0x20, S3 },   /* step 6, current state S12 */
	{0x60, S2 },   /* step 7, current state S4  */
};

const struct State *Pt;  // Current State
unsigned char cState; 

/*****************************************************************************
 Local const Variables
******************************************************************************/
static char KEY_DECODE_TABLE[4][3] = 
{
	{ '1', '2', '3' }, /* first row of the keypad  */
	{ '4', '5', '6' }, /* second row of the keypad  */
	{ '7', '8', '9' }, /* third row of the keypad  */
	{ '*', '0', '#' }  /* fourth row of the keypad  */
}; 

/*****************************************************************************
 Local Variables
******************************************************************************/
/* SysTick   */
static volatile BOOL	g_bSystemTick = FALSE;		 // SET SYSTEM TICK FLAG AS FALSE
static volatile BOOL 	g_bSystemTick1000 = FALSE; // FLAG CHANGES AFTER 1000 TICKS 
static volatile BOOL	g_bSecTick = FALSE;				 // SET SECTICK FLAG TO FALSE
static int            g_nCount = 0;							 // INITIALISE nCount TO 0 
static volatile BOOL	g_bToggle = FALSE;				 //	SET TOGGLE FLAG TO FALSE
static unsigned int		g_nTimeSec = 0;						 // INITIALISE TIMESEC TO 0

/** SPI Interface & LCD 				**/
static volatile int 	g_bSpiDone = FALSE;				// FLAG TO CHECK IF SPI IS DONE
static SPIM_HANDLE		g_SpimHandle;							// INITIALISE SPIMHANDLE
static GUI_DATA				g_aBuf[LCD_BUF_SIZE];
static GUI_MEMDEV			g_MemDev;
static volatile BOOL 	g_bLCDUpdate = FALSE;			// FLAG FOR LCD UPDATE
static volatile int 	g_nLCD = LCD_UPDATE_MS;		// ASSIGN LCD TO LCD UPDATE MS
static volatile BOOL	g_bBacklightOn = TRUE;		// SET BACK LIGHT ON FLAG AS TRUE 
static volatile BOOL 	g_bLcdFree  = TRUE;				// SET LCD FREE FLAG AS TRUE
static volatile int   g_nDelay = 0;							// INITIALISE DELAY TO 0

static volatile int 		g_bSTORE_Hundredth  = 0;	// TEMPORARY STORAGE FOR HUNDREDTH VALUE
static volatile int 		g_bSTORE_Tens  = 0;				// TEMPORARY STORAGE FOR TENS VALUE
static volatile int 		g_bSTORE_Ones  = 0;				// TEMPORARY STORAGE FOR ONES VALUE
static volatile int 		g_bHundredth  = 0;				// VARIABLE FOR HUNDREDTH VALUE
static volatile int 		g_bTens  = 0;							// VARIABLE FOR TENS VALUE
static volatile int 		g_bOnes  = 0;							// VARIABLE FOR ONE VALUE
static volatile char 		print_H  = '-';						// DISPLAY DASH FOR HUNDREDTH PLACE
static volatile char 		print_T  = '-';						// DISPLAY DASH FOR TENS PLACE
static volatile char 		print_O  = '-';						// DISPLAY DASH FOR ONES PLACE

/** keypad 					**/
static volatile BOOL 		g_nKeypadScan = FALSE;					// FLAG FOR KEYPADSCAN
static volatile int 		g_nKeypad = KEYPAD_UPDATE_MS;		// ASSIGN THE KEYPAD SCAN UPDATE VALUE TO 50U
static unsigned char 		g_cKey = '-';										// VARIABLE TO DETECT WHAT VALUE IS BEING INPUTTED
static volatile	BOOL		g_bKeypadPressed  = FALSE;			// FLAG TO DETECT IF KEYPAD IS PRESSED
static volatile BOOL    g_bpressed = FALSE;							// ADDITIONAL FLAG TO DETECT IF KEYPAD IS PRESSED
static volatile int 		g_debounce = 0;									// INITIALISE DEBOUNCE TO 0 
static volatile int 		wrong_password_counter  = 0;		// COUNTER TO DETECT HOW MANY TIMES THE USER HAS INPUT THE PASSCODE
static volatile BOOL 		wrong_password_input  = FALSE;  // FLAG FOR WRONG PASSWWORD INPUT
static volatile BOOL		bKeyPressed = FALSE;						// ADDITIONAL FLAG TO DETECT IF THE KEYPAD IS PRESSED

/* I2C0     */
static I2C_HANDLE			g_I2C0Handle;
volatile BOOL	g_bI2C0IsBusy = FALSE;

/* AHT10 */
static AHT10 g_AHT10;																					// DATA STRUCTURE
static volatile uint8_t 	g_nAHT10Delay;											// MEASUREMENT DELAY
static volatile int				g_nAHT10Update = AHT10_UPDATE_MS;		// CONTROLS AHT10 UPDATE INTERVAL
static volatile BOOL			g_bAHT10Update = FALSE;   					// AHT10 UPDATE FLAG

/* MCP23017 */
static MCP23017 					g_MCP23017;													// DATA STRUCTURE

/** Buzzer     **/
static volatile int	g_nBeep_Count = 0;										 		// INTIIALISE BEEP COUNT TO 0 
static volatile int  g_nLEDCount = 0;  												// INITIALISE LED COUNT TO 0
static volatile int  g_nBUZZCount = 0; 												// INITIALISE BUZZ COUNT TO 0
static volatile BOOL			g_bBUZZTick = FALSE; 								// BUZZER FLAG FOR SLIDER SOUND
static volatile BOOL			g_Buzzer = FALSE;										// FLAG FOR MASTER BUZZER
static volatile BOOL			CORRECTPASSWORD_BUZZERFLAG = FALSE;	// BUZZER FLAG FOR CORRECT PASSWORD
static volatile BOOL			CLOSESAFE_BUZZERFLAG = FALSE;       // BUZZER FLAG FOR CLOSING SAFE
static volatile BOOL			SLIDEROFF_BUZZERFLAG = FALSE;       // BUZZER FLAG FOR TURNING OFF BUZZER WHEN SLIDER IN DEFAULT POSITION
static volatile BOOL			WRONGPASSWORD_BUZZERFLAG = FALSE;   // BUZZER FLAG FOR WRONG PASSWORD
static volatile BOOL			SLIDERLEFT_BUZZERFLAG = FALSE;      // BUZZER FLAG FOR SLIDER TEMP DROP
static volatile BOOL			SLIDERMIDDLE_BUZZERFLAG = FALSE;    // BUZZER FLAG FOR TURNING OFF BUZZER WHEN SLIDER IN DEFAULT POSITION
static volatile BOOL			SLIDERRIGHT_BUZZERFLAG = FALSE;     // BUZZER FLAG FOR SLIDER TEMP RISE
static volatile BOOL			COUNTDOWN_BUZZERFLAG = FALSE;       // BUZZER FLAG FOR 7 SEGMENT 60S AND 10S COUNTDOWN
static volatile BOOL			COUNTDOWN_REACH_0 = FALSE;          // BUZZER FLAG FOR WHEN COUNTDOWN REACHES 0

/** I2c Expander     **/
static volatile uint8_t	dig1 = 0; // ASSIGN FIRST DIGIT OF 7 SEGMENT AS 0
static volatile uint8_t	dig2 = 1; // ASSIGN SECOND DIGIT OF 7 SEGMENT AS 1

/** 7-segment */
static uint16_t						g_n7SegCount = 60;												// INITIALISE 7 SEGMENT COUNT TO 60 FOR 60S COUNTDONW
static uint16_t						g_n7Seg10sCount = 10;											// INITIALISE 7 SEGMENT COUNT TO 10 FOR 10S COUNTDONW
static volatile BOOL			g_bCountDownFlag = FALSE;									// FLAG TO SET g_bInitiateCountdown TO TRUE
static uint16_t						g_nCountDown = 0;													// FLAG TO SET g_bCountDownFlag TO TRUE
static uint16_t						g_nCountDown10s = 0;											// FLAG TO SET g_bCountDownFlag10s TO TRUE
static volatile BOOL			g_bCountDownFlag10s = FALSE;							// FLAG TO INITIATE 10S COUNTDOWN
static volatile BOOL			g_bPOWERUP = TRUE;												// FLAG TO DISPLAY 60 ON 7 SEGMENT UPON POWER UP
static volatile BOOL			g_bInitiateCountdown = FALSE;						  // FLAG TO INTIALISE 60S COUNTDOWN
static volatile BOOL			g_bInitiate10sCountdown = FALSE;					// FLAG TO INTIALISE 10S COUNTDOWN
static volatile BOOL			g_bInitiateCountdownAsterisk = FALSE;			// FLAG TO INTIALISE 60S COUNTDOWN WHEN USER KEY IN THE FIRST DIGIT
static volatile BOOL			SEVENSEG_STOP = FALSE; 										// FLAG TO STOP THE 60S COUNTDOWN
static volatile BOOL			SEVENSEG_STOP10s = FALSE;									// FLAG TO STOP THE 10S COUNTDOWN

/* 60s countdown after 10s    */
static volatile BOOL			g_bInitiate60sCountdownAfter10s = FALSE;  // FLAG TO RESET THE 60S COUNTDOWN AFTER THE 10S COUNTDOWN
static volatile BOOL			g_bInitiateCountdown60safter10s = FALSE;  // FLAG TO INCREMENT g_nCountDown60sAfter10s
static uint16_t						g_nCountDown60sAfter10s = 0;  						// FLAG TO SET g_bCountDownFlag60sAfter10s TO TRUE
static volatile BOOL			g_bCountDownFlag60sAfter10s = FALSE; 			// FLAG TO INCREMENT g_nCountDown60sAfter10s

/* Switch     */
static BOOL								SW2_Pressed = FALSE; 											// FLAG FOR STATUS OF SWITCH 2

/* LED     */
static volatile BOOL			g_bLedTick = FALSE;												// FLAG TO TOGGLE LED
static volatile BOOL			g_bLEDToggle = FALSE;											// FLAG TO TOGGLE LED
static volatile BOOL			CORRECTPASSWORD;													// FLAG TO DETERMINE IF CORRECT PASSWORD IS INPUTTED
static volatile BOOL			g_LED = FALSE;														// FLAG FOR MASTER LED
static volatile BOOL			g_LED_OPENSAFE = FALSE;										// FLAG FOR LED WHEN SAFE IS OPENED
static volatile BOOL			g_LED_CLOSESAFE = FALSE;									// FLAG FOR LED WHEN SAFE IS CLOSED
static volatile BOOL			g_LED_WRONGPW = FALSE;										// FLAG FOR LED WHEN WRONG PASSWORD IS INPUTTED
static volatile BOOL			g_LED_LEFTSLIDER = FALSE;									// FLAG FOR LED WHEN SLIDER IS ON THE LEFT
static volatile BOOL			g_LED_MIDDLESLIDER = FALSE;								// FLAG FOR LED WHEN SLIDER IS IN THE MIDDLE
static volatile BOOL			g_LED_RIGHTSLIDER = FALSE;								// FLAG FOR LED WHEN SLIDER IS ON THE RIGHT
static volatile BOOL			g_LED_SLIDEROFF = FALSE;									// FLAG FOR LED WHEN SLIDEROFF FLAG IS SET

/** stepper motor   **/
static volatile int   		g_nMotorCount = MOTOR_STEP_TIME; 			   // ASSIGN MOTOR COUNT TO 10U
static volatile BOOL  		g_bmotor_move = FALSE;						 	     // FLAG FOR MOVEMENT OF MOTOR
int steps;																									       // NUMBER OF STEPS MOTOR WILL TAKE 
static volatile BOOL   		g_bMotor_ACW  = 1;		     					     // FLAG FOR DIRECTION OF ROTATION OF MOTOR
static volatile int				UserInput = 0;							      	     // INITIALISE USER INPUT AS 0
static volatile int				CorrectPassword = 138;                   // DEFINE CORRECT PASSWORD AS 138. ANY CHANGES TO THE PASSWORD MADE HERE.
static volatile int 			counter_digit = 0;							         // COUNT HOW MANY TIMES THE USER HAS PRESSED THE KEYPAD
static volatile BOOL			g_bOpenSafe = FALSE;						         // FLAG TO DETERMINE IF THE SAFE IS OPENED
static volatile BOOL			g_bCONFIRMOpenSafe = FALSE;				       // FIRST FLAG FOR SWITCH 2 TO OPERATE ONLY IF THE SAFE IS OPENED
static volatile BOOL			g_bCLOSE_SAFE_ONLY_WHEN_OPENED = FALSE;  // SECOND FLAG FOR SWITCH 2 TO OPERATE ONLY IF THE SAFE IS OPENED

/*ADC*/
static volatile int 			g_nADCUpdate = ADC_UPDATE_MS; // ASSIGN ADC UPDATE TO 10U
static volatile BOOL 			g_bAdcUpdate = FALSE;					// FLAG FOR ADC UPDATE
static volatile float 		acttemperature = 0; 					// INITIALISE ACTUAL TEMPERATURE TO 0
static volatile float 		LDR = 0;											// INITIALISE LDR TO 0
static volatile float 		slider = 0;										// INTIALISE SLIDER TO 0
static volatile float 		settemperature = 0; 					// INITIALISE SET TEMPERATURE TO 0

/*Image frames*/
const unsigned short *frames[2] = {bmpunlock_2, bmpunlock_1};
const unsigned short *open[2] = {bmpopen1, bmpopen2};

/*KEYPAD*/
static volatile BOOL			LOCK_KEYPAD = FALSE; // FLAG TO LOCK THE KEYPAD
/*****************************************************************************
 Callbacks Prototypes
******************************************************************************/
void main_cbI2C0Isdone( void );

/*****************************************************************************
 Local Functions Prototypes
******************************************************************************/
static void main_LcdInit( void ); 
static void main_KeyScan( void );
static void motor_output (int);
void GUI_AppDraw( BOOL bFrameStart );
static void main_AdcUpdate(void);
static void main_AdcInit(void);

/*****************************************************************************
 Implementation
******************************************************************************/
int main()
{
	Port_Init();
	SystemCoreClockUpdate ();
	SysTick_Config( SystemCoreClock/1000 );  
	
	/* SSI initialization */
	NVIC_SetPriority( SSI0_IRQn, 0 );
	
	// initialize SSI port for TFT LCD
	SpimInit(
		&g_SpimHandle,
		0U,
		25000000U,
		SPI_CLK_INACT_LOW,
		SPI_CLK_RISING_EDGE,
		SPI_DATA_SIZE_8 );
	
	main_LcdInit();
	main_AdcInit();
	IRQ_Init();
	
	/* Initialiae I2C0 bus   */
	I2CInit(&g_I2C0Handle, I2C_0,I2C_400K);
	I2CAddCallback(&g_I2C0Handle, main_cbI2C0Isdone); // add Callback for I2C bus transaction
	NVIC_SetPriority( I2C0_IRQn, 1 );
	
	AHT10_Init (&g_I2C0Handle, &g_AHT10); 							// initialize AHT10 module
	I2CExpander_Init (&g_I2C0Handle, &g_MCP23017); 		// initialize MCP23017
		
	IRQ_Init();
	
	cState = 0; //Initial State	
	steps = (int)(MOTOR_ANGLE * 1024/360);
	
	/* print to Virtual COM port temrinal */
	printf ("\n\rHello World! \n\r"); // display to virtual COM port
	printf ("Count Mode: %d\n\r", g_MCP23017.Mode);
	
	for(;;)
  {
/**************************SLIDER**************************************************/	
		/* If Actual is >0.5°C compared to Set, LED = BLUE (TEMPERATURE DROP ANOMALY). */
		if ((acttemperature > (settemperature + 2.0))) //IF ACTUAL TEMPERATURE IS MORE THAN SET TEMPERATURE+2
			{
				g_Buzzer = TRUE; 							// SET THE MASTER BUZZER FLAG TO TRUE 
				g_LED = TRUE;									// SET THE MASTER LED FLAG TO TRUE 
				g_LED_LEFTSLIDER = TRUE;			// SET THE LED LEFT SLIDER TO TRUE
				SLIDERLEFT_BUZZERFLAG = TRUE;	// SET THE BUZZER LEFT SLIDER FLAG TO TRUE
		}
		
		/* If Actual temperature is within ±0.5ºC of Set temperature, LED colour = GREEN.  */
		if (((settemperature - 2.0) < acttemperature) && (acttemperature < (settemperature + 2.0)) && SLIDEROFF_BUZZERFLAG == FALSE && g_LED_SLIDEROFF == FALSE)
		{
			if(CLOSESAFE_BUZZERFLAG == FALSE) // IF CLOSE SAFE BUZZER FLAG IS SET TO FALSE
			{
				g_Buzzer = TRUE;									// SET THE MASTER BUZZER FLAG TO TRUE 
				SLIDERMIDDLE_BUZZERFLAG = TRUE;		// SET THE BUZZER MIDDLE SLIDER FLAG TO TRUE
				g_LED = TRUE;											// SET THE MASTER LED FLAG TO TRUE 
				g_LED_MIDDLESLIDER = TRUE;				// SET THE LED MIDDLE SLIDER FLAG TO TRUE
			}
		}
		
		/* If Actual is <0.5°C compared to Set, LED = YELLOW (TEMPERATURE RISE ANOMALY). */
		if (acttemperature < (settemperature - 2.0))
			{
				if(CORRECTPASSWORD == FALSE) // IF CORRECT PASSWORD FLAG IS SET TO FALSE
				{
								g_LED = TRUE;										// SET THE MASTER LED FLAG TO TRUE 
								g_LED_RIGHTSLIDER = TRUE;				// SET THE LED RIGHT SLIDER FLAG TO TRUE
								g_Buzzer = TRUE;								// SET THE MASTER BUZZER FLAG TO TRUE 
								SLIDERRIGHT_BUZZERFLAG = TRUE;	// SSET THE BUZZER RIGHT SLIDER FLAG TO TRUE
				}	
		}
	
/**************************SLIDER**************************************************/

/******************************MASTER LED************************************
		CONTROLS ALL THE LED FUNCTION BASED ON THE FLAG THAT IS CALLED
*****************************************************************************/
			if(	g_LED == TRUE) // IF MASTER LED FLAG IS CALLED
			{
				g_LED = FALSE; // RESET MASTER LED FLAG
				
				/* LED MASTER FLAG FOR OPEN SAFE */
				if(g_LED_OPENSAFE == TRUE) // IF LED OPEN SAFE IS SET TO TRUE
				{
					LED_RGB_SET(RGB_OFF); 	// TURN OFF ALL LED
					LED_RGB_SET(RGB_GREEN); // TURN ON GREEN LED
				}
				if(g_LED_OPENSAFE == FALSE) // IF LED OPEN SAFE IS SET TO FALSE
				{
					LED_RGB_SET(RGB_OFF);	// TURN OFF ALL LED 
				}
				
				/* LED MASTER FLAG FOR CLOSE SAFE */
				if(g_LED_CLOSESAFE == TRUE) // IF LED CLOSE SAFE IS SET TO TRUE
				{
					LED_RGB_SET(RGB_OFF);	// TURN OFF ALL LED 
					LED_RGB_SET(RGB_MAGENTA); // TURN ON MAGENTA LED
				}
				if(g_LED_CLOSESAFE == FALSE && g_LED_OPENSAFE == FALSE) // IF LED CLOSE SAFE IS SET TO FALSE
				{
					LED_RGB_SET(RGB_OFF);	// TURN OFF ALL LED 
				}
				
				/* LED MASTER FLAG FOR WRONG PW */
				if(g_LED_WRONGPW == TRUE) // IF WRONG PASSWORD LED FLAG IS TRUE
				{
					g_LED_WRONGPW = FALSE; // RESET LED WRONG PASSWORD FLAG
					LED_RGB_SET(RGB_OFF);  // TURN OFF ALL LED 
					LED_RGB_SET(RGB_RED);  // TURN ON RED LED
				}
				
				/* LED MASTER FLAG FOR MIDDLE SLIDER*/
				if(g_LED_MIDDLESLIDER == TRUE && g_LED_SLIDEROFF == FALSE) // IF LED LEFT MIDDLE SET TO TRUE AND LED SLIDER OFF SET TO TRUE
				{
					g_LED_MIDDLESLIDER = FALSE; // RESET LED MIDDLE SLIDER FLAG
					LED_RGB_SET(RGB_OFF);	      // TURN OFF ALL LED 
				}
				
				/* LED MASTER FLAG FOR LEFT SLIDER */
				if(g_LED_LEFTSLIDER == TRUE && g_LED_SLIDEROFF == FALSE) // IF LED LEFT MIDDLE SET TO TRUE AND LED SLIDER OFF SET TO TRUE
				{
					g_LED_LEFTSLIDER = FALSE; // RESET LED LEFT SLIDER FLAG
					LED_RGB_SET(RGB_OFF);			// TURN OFF ALL LED 
					LED_RGB_SET(RGB_BLUE);	  // TURN ON BLUE LED
				}
				
				/* LED MASTER FLAG FOR RIGHT SLIDER */
				if(g_LED_RIGHTSLIDER == TRUE && g_LED_SLIDEROFF == FALSE) // IF LED RIGHT MIDDLE SET TO TRUE AND LED SLIDER OFF SET TO TRUE
				{ 
					g_LED_RIGHTSLIDER = FALSE; // RESET LED RIGHT SLIDER FLAG
					LED_RGB_SET(RGB_OFF);			 // TURN OFF ALL LED 
					LED_RGB_SET(RGB_YELLOW); 	 // TURN ON YELLOW LED
				}
				
			}
/******************************MASTER LED************************************/		
		
/******************************MASTER BUZZER************************************
		CONTROLS ALL THE BUZZER FUNCTION BASED ON THE FLAG THAT IS CALLED
*****************************************************************************/			
			if(	g_Buzzer == TRUE) // IF MASTER BUZZER FLAG IS SET
			{
				g_Buzzer = FALSE; // RESET MASTER BUZZER FLAG
				
				/* BUZZER MASTER FLAG FOR CORRECT PASSWORD */
				if(CORRECTPASSWORD_BUZZERFLAG == TRUE && SLIDEROFF_BUZZERFLAG == TRUE) // IF CORRECT PASSWORD BUZZER FLAG IS SET TO TRUE AND THE SLIDER OFF BUZZER FLAG IS SET TO TRUE
				{
					CORRECTPASSWORD_BUZZERFLAG = FALSE; // RESET THE CORRECT PASSWORD BUZZER FLAG
					SLIDEROFF_BUZZERFLAG = FALSE;				// RESET THE SLIDER OFF BUZZER FLAG
					BUZZER_ON();												// TURN ON THE BUZZER
					g_nBeep_Count = 500;								// SET THE BUZZER DURATION TO TURN ON FOR 5S
				}
				
				/* BUZZER MASTER FLAG FOR WRONG PASSWORD */
				if(WRONGPASSWORD_BUZZERFLAG == TRUE && SLIDEROFF_BUZZERFLAG == TRUE) // IF WRONG PASSWORD BUZZER FLAG IS SET TO TRUE AND THE SLIDER OFF BUZZER FLAG IS SET TO TRUE
				{
					WRONGPASSWORD_BUZZERFLAG = FALSE;	// RESET THE WRONG PASSWORD BUZZER FLAG
					BUZZER_ON();											// TURN ON THE BUZZER
					g_nBeep_Count = 900;							// SET THE BUZZER DURATION TO TURN ON FOR 9S
				}
				
				/* BUZZER MASTER FLAG FOR CLOSIG SAFE */
				if(CLOSESAFE_BUZZERFLAG == TRUE && SLIDEROFF_BUZZERFLAG == FALSE) // IF CLOSE SAFE BUZZER FLAG IS SET TO TRUE AND THE SLIDER OFF BUZZER FLAG IS SET TO TRUE
				{
					CLOSESAFE_BUZZERFLAG = FALSE;	// RESET THE CLOSE SAFE BUZZER FLAG
					SLIDEROFF_BUZZERFLAG = FALSE;	// RESET THE SLIDER OFF BUZZER FLAG
					BUZZER_ON();									// TURN ON THE BUZZER
					g_nBeep_Count = 500;					// SET THE BUZZER DURATION TO TURN ON FOR 5S
				}
				
				/* BUZZER MASTER FLAG FOR TEMP DROP SLIDER */
				if(SLIDERLEFT_BUZZERFLAG == TRUE ) // IF SLIDER LEFT BUZZER FLAG IS SET TO TRUE
				{
					SLIDERLEFT_BUZZERFLAG = FALSE;	// RESET THE SLIDER LEFT BUZZER FLAG
					BUZZER_ON();										// TURN ON THE BUZZER
					g_nBeep_Count = 900;						// SET THE BUZZER DURATION TO TURN ON FOR 9S
				}
				
				/* BUZZER MASTER FLAG FOR TEMP RISE SLIDER */
				if(SLIDERRIGHT_BUZZERFLAG == TRUE) // IF SLIDER RIGHT BUZZER FLAG IS SET TO TRUE
				{
					SLIDERRIGHT_BUZZERFLAG = FALSE;	// RESET THE SLIDER RIGHT BUZZER FLAG
					BUZZER_ON();										// TURN ON THE BUZZER
					g_nBeep_Count = 900;						// SET THE BUZZER DURATION TO TURN ON FOR 9S
				}
				
				/* BUZZER MASTER FLAG FOR 7 SEGMENT COUNTDOWN */
				if(COUNTDOWN_BUZZERFLAG == TRUE) // IF COUNTDOWN BUZZER FLAG IS SET TO TRUE
				{
					COUNTDOWN_BUZZERFLAG = FALSE;	// RESET THE COUNTDOWN BUZZER FLAG
					BUZZER_ON();									// TURN ON THE BUZZER
					g_nBeep_Count = 100;					// SET THE BUZZER DURATION TO TURN ON FOR 1S
		  	}
				
			 /* BUZZER MASTER FLAG FOR END OF COUNTDOWN */
	   	 if(COUNTDOWN_REACH_0 == TRUE) //IF 60S COUNTDOWN REACHES 0  
				{
					COUNTDOWN_REACH_0 = FALSE; // RESET THE COUNTDOWN REACH 0 FLAG
				}
				
			}
			

/******************************MASTER BUZZER************************************/
		
/******************************OPEN SAFE************************************/
if (g_bOpenSafe == TRUE && SW2_Pressed == FALSE) // IF OPEN SAFE FLAG IS TRUE AND SWITCH 2 IS NOT PRESSED
		{	
			g_MCP23017.Mode = UNLOCK; // SET MODE TO UNLOCK
			if (((FALSE != g_bmotor_move) && (0 != steps)) && (g_bpressed == TRUE)) // IF MOTOR MOVE IS SET TO TRUE, STEPS IS NOT EQUAL TO 0, g_bpressed SET TO TRUE 
			{			
				g_bmotor_move = FALSE; 	// SET MOTOR MOVE FLAG TO TRUE
				g_bMotor_ACW = FALSE;	 	// SET MOTOR ROTATION DIRECTION TO CW
				
				g_LED = TRUE;						// SET MASTER LED FLAG TO TRUE 
				g_LED_OPENSAFE = TRUE;	// SET LED OPEN SAFE FLAG TO TRUE
				g_LED_SLIDEROFF = TRUE;	// SET LED SLIDEROFF TO TRUE			
				
					g_Buzzer = TRUE;												// SET MASTER BUZZER FLAG TO TRUE 
					SLIDEROFF_BUZZERFLAG = TRUE;						// SET SLIDEROFF BUZZER FLAG TO TRUE 
					motor_output (cState);									// SET MOTOR OUTPUT TO C STATE
					cState++;																// INCREMENT C STATE
					if (cState == 8)												// IF C STATE EQUAL TO 8
						cState = 0;														// SET C STATE TO 0
					steps--;																// DECREMENT STEPS
					if (steps == 0)													// IF STEPS EQUAL TO 0
					{ g_LED_OPENSAFE = FALSE;								// SET LED OPEN SAFE FLAG TO FALSE
						g_LED_SLIDEROFF = FALSE; 							// SET LED SLIDEROFF FLAG TO FALSE
					g_bCLOSE_SAFE_ONLY_WHEN_OPENED = TRUE;	// SET CLOSE SAFE ONLY WHEN OPENED FLAG TO FALSE
					g_bOpenSafe = FALSE;										// SET OPEN SAFE FLAG TO FALSE
					g_bpressed = FALSE;											// SET g_bpressed TO FALSE
					g_bOnes = 0;														// ONES PLACE
					g_bTens = 0;														// TENS PLACE
					g_bHundredth = 0;												// HUNDREDS PLACE
					print_H = '-';													// PRINT 3 DASHES WHEN MOTOR HAS ROTATED 180 CW COMPLETELY
					print_T = '-';													// PRINT 3 DASHES WHEN MOTOR HAS ROTATED 180 CW COMPLETELY
					print_O = '-';													// PRINT 3 DASHES WHEN MOTOR HAS ROTATED 180 CW COMPLETELY
					counter_digit = 0;											// SET COUNTER DIGIT TO 0 
					steps = (int)(MOTOR_ANGLE * 1024/360);	// CALCULATE THE NUMBER OF STEPS THE MOTOR HAS TO TAKE
				}
			}
		}
		
		/*WRONG PASSWORD MOTOR*/
		if (g_bOpenSafe == FALSE && SW2_Pressed == FALSE) // IF OPEN SAFE FLAG IS FALSE AND SWITCH 2 IS NOT PRESSED
		{	
			g_MCP23017.Mode = UNLOCK; // SET MODE TO UNLOCK
			if (((FALSE != g_bmotor_move) && (0 != steps)) && (g_bpressed == TRUE)) // IF MOTOR MOVE IS SET TO TRUE, STEPS IS NOT EQUAL TO 0, g_bpressed SET TO TRUE 
			{			
				g_bmotor_move = FALSE; 	// SET MOTOR MOVE FLAG TO TRUE
				g_bMotor_ACW = FALSE;	 	// SET MOTOR ROTATION DIRECTION TO CW
				
				g_LED = TRUE;						// SET MASTER LED FLAG TO TRUE 
				g_LED_WRONGPW = TRUE;	  // SET WRONG PASSWORD FLAG TO TRUE 
				g_LED_SLIDEROFF = TRUE; // SET LED SLIDEROFF TO TRUE

				g_Buzzer = FALSE;						 // SET MASTER BUZZER FLAG TO TRUE 
				SLIDEROFF_BUZZERFLAG = TRUE; // SET SLIDEROFF BUZZER FLAG TO TRUE 
				
				steps = 0;									 // SET STEPS TO 0 
				if(steps == 0)							 // IF STEPS EQUAL TO 0 
				{
					g_LED = TRUE;											// SET MASTER LED FLAG TO TRUE  
					g_LED_SLIDEROFF = FALSE;					// SET LED SLIDEROFF TO FALSE			
					g_LED_CLOSESAFE = FALSE;					// SET LED CLOSE SAFE FLAG TO TRUE
					
					g_Buzzer = FALSE;												// SET MASTER BUZZER FLAG TO TRUE  
					CLOSESAFE_BUZZERFLAG = TRUE;						// SET CLOSE SAFE BUZZER FLAG TO TRUE
					SLIDERMIDDLE_BUZZERFLAG = FALSE;				// SET MIDDLE SLIDER BUZZER FLAG TO FALSE
					SLIDEROFF_BUZZERFLAG = FALSE;						// SET SLIDER OFF BUZZER FLAG TO FALSE
					g_MCP23017.Mode = LOCK;									// SET MODE TO LOCK 
					g_bOpenSafe = FALSE;										// SET OPEN SAFE FLAG TO FALSE
					g_bpressed = FALSE;											// SET g_bpressed TO FALSE
					g_bOnes = 0;														// ONES PLACE
					g_bTens = 0;														// TENS PLACE
					g_bHundredth = 0;												// HUNDREDS PLACE
					print_H = '-';													// PRINT 3 DASHES ON THE LCD
					print_T = '-';													// PRINT 3 DASHES ON THE LCD
					print_O = '-';													// PRINT 3 DASHES ON THE LCD
					counter_digit = 0;											// SET COUNTER DIGIT TO 0 
					steps = (int)(MOTOR_ANGLE * 1024/360);	// CALCULATE THE NUMBER OF STEPS THE MOTOR HAS TO TAKE
				}
				
			}
		}
/******************************OPEN SAFE************************************/

/******************************CORRECT PASSWORD************************************/		
		if(CORRECTPASSWORD == TRUE && g_bOpenSafe == TRUE && SW2_Pressed == FALSE) // IF CORRECTPASSWORD FLAG IS SET TO TRUE, OPEN SAFE FLAG IS SET TO TRUE, SWITCH 2 IS NOT PRESSED
		{
			CORRECTPASSWORD = FALSE;					 // RESET THE CORRECT PASSWORD FLAG
			g_Buzzer = TRUE;									 // SET THE MASTER BUZZER FLAG TO TRUE
			CORRECTPASSWORD_BUZZERFLAG = TRUE; // SET THE CORRECT PASSWORD BUZZER FLAG TO TRUE
		}
/******************************CORRECT PASSWORD************************************/
		
	

/******************************WRONG PASSWORD************************************/		
		if(wrong_password_input == TRUE && g_bOpenSafe == FALSE && SW2_Pressed == FALSE) // IF USER INPUT WRONG PASSWORD THRICE, SAFE IS NOT OPENED, SWITCHH 2 IS NOT PRESSED
		{
			wrong_password_input = FALSE;			// RESET WRONG PASSWORD INPUT FLAG
			WRONGPASSWORD_BUZZERFLAG = TRUE;	// SET WRONG PASSWORD INPUT BUZZER FLAG TO TRUE
			g_Buzzer = TRUE;									// SET MASTER BUZZER FLAG TO TRUE
			SLIDEROFF_BUZZERFLAG = TRUE;		  // SET THE SLIDEROFF BUZZER FLAG TO TRUE
			g_LED = TRUE;											// SET MASTER LED FLAG TO TRUE
			g_LED_WRONGPW = TRUE;							// SET WRONG PASSWORD LED FLAG TO TRUE

		}
/******************************WRONG PASSWORD************************************/
/******************************CLOSE SAFE************************************/	
		
	  		if(g_bCLOSE_SAFE_ONLY_WHEN_OPENED == TRUE && g_bOpenSafe == FALSE ) // IF SAFE IS OPENED AND WAITING TO BE CLOSED
				{
					LOCK_KEYPAD = TRUE; // SET LOCK KEYPAD FLAG TO TRUE TO LOCK THE KEYPAD
				}
				
				if (g_bOpenSafe == FALSE && (SW2_Pressed == TRUE) && ((g_debounce == 0)) && g_bCLOSE_SAFE_ONLY_WHEN_OPENED == TRUE)
		{		
				if(g_MCP23017.Mode == UNLOCK)	//IF SWITCH 2 IS PRESSED ON UNLOCK
				{
					g_LED = TRUE;						// SET MASTER LED FLAG TO TRUE 
					g_LED_CLOSESAFE = TRUE;	// SET LED CLOSE SAFE FLAG TO TRUE 
					
				if (FALSE != g_bmotor_move && 0 != steps) // IF MOTOR MOVE FLAG IS TRUE AND STEPS IS NOT EQUAL TO 0 
				{	
					g_bmotor_move = FALSE; 													// SET MOTOR MOVE FLAG TO FALSE
					g_bMotor_ACW = TRUE;	 													// MAKE MOTOR ROTATION TO BE ACW
					g_Buzzer = FALSE;																// SET MASTER BUZZER FLAG TO FALSE
					SLIDEROFF_BUZZERFLAG = TRUE;										// SET SLIDEROFF BUZZER FLAG TO TRUE 
					motor_output (cState);													// SET MOTOR OUTPUT TO C STATE
					cState++;																				// INCREMENT C STATE
					if (cState == 8) 																// IF C STATE EQUAL TO 8
					cState = 0;																			// SET C STATE TO 0
					steps--;																				// DECREMENT STEPS
					
					if (steps == 0)																	// IF STEPS EQUAL TO 0
					{		g_bCLOSE_SAFE_ONLY_WHEN_OPENED = FALSE;					// setbuf CLOSE SAFE ONLY WHEN OPENED FLAG TO FALSE
					
							g_n7SegCount = 60;															// ASSIGN 60 TO 7 SEG COUNT 
							g_MCP23017.Digit1 = 0x82; 											// DISPLAY 60 ON 7 SEGMENT 												
							g_MCP23017.Digit2 = 0xC0;												// DISPLAY 60 ON 7 SEGMENT 		
							write_I2CExpander (&g_I2C0Handle, &g_MCP23017);	// WRITE TO I2C EXPANDER
							g_MCP23017.Mode = LOCK;													// SET MODE TO LOCK
							g_bpressed = FALSE;															// SET g_bpressed TO FALSE TO RESET KEYPAD
							bKeyPressed = FALSE;														// SET bKeyPressed TO FALSE TO RESET KEYPAD
							SW2_Pressed = FALSE;														// SET SWITCH 2 STATUS TO FALSE
							g_LED = TRUE;																		// SET MASTER LED FLAG TO TRUE 
							g_LED_CLOSESAFE = FALSE;												// SET LED CLOSE SAFE FLAG TO TRUE 
							g_LED_SLIDEROFF = FALSE;												// SET LED SLIDER OFF FLAG TO TRUE 
							g_Buzzer = TRUE;																// SET MASTER BUZZER FLAG TO TRUE 
							CLOSESAFE_BUZZERFLAG = TRUE;										// SET CLOSE SAFE BUZZER FLAG TO TRUE 
							SLIDERMIDDLE_BUZZERFLAG = FALSE;								// SET SLIDER MIDDLE BUZZER FLAG TO TRUE 
							SLIDEROFF_BUZZERFLAG = FALSE;										// SET SLIDER OFF BUZZER FLAG TO TRUE 
							steps = (int)(MOTOR_ANGLE * 1024/360);					// CALCULATE THE NUMBER OF STEPS THE MOTOR HAS TO TAKE
					}
				}
			}
		}
						
		if (g_bOpenSafe == FALSE && (SW2_Pressed == TRUE) && ((g_debounce == 0))) // IF OPEN SAFE FLAG IS TRUE, IF SWITCH2 PRESSED IS TRUE, IF DEBOUNCE IS EQUAL TO 0
		{
				if(g_MCP23017.Mode == LOCK) // IF MODE IS LOCK
				{
					g_bmotor_move = FALSE; // RESET MOTOR MOVE FLAG 
				}
		}
		
/******************************CLOSE SAFE************************************/		
/******************************DISPLAY 60 ON 7SEG************************************/		
		if(g_bPOWERUP == TRUE) //IF POWER IS SUPPLIED
		{

			g_MCP23017.Digit1 = 0x82; 											// 7 SEGMENT DISPLAYS 60												
			g_MCP23017.Digit2 = 0xC0;												// 7 SEGMENT DISPLAYS 60												
			write_I2CExpander (&g_I2C0Handle, &g_MCP23017); // WRITE TO THE I2C EXPANDER
			g_bPOWERUP = FALSE;															// RESET THE POWERUP FLAG
			
		}
/******************************DISPLAY 60 ON 7SEG************************************/		

		if(LOCK_KEYPAD == TRUE) // IF LOCK KEYPAD FLAG IS SET TO TRUE
		{			
					LOCK_KEYPAD = FALSE; // RESET THE LOCK KEYPAD FLAG
					g_bpressed = TRUE;	 // LOCK THE KEYPAD
					bKeyPressed = TRUE;	 // LOCK THE KEYPAD
		}
		
/******************************INITIATE 10S COUNTDOWN************************************/			
		if(wrong_password_counter == 3 ) // IF USER INPUT THE WRONG PASSWORD 3 TIMES
		{
			g_MCP23017.Digit1 = 0xF9; 											// 7 SEG TO DISPLAY 10 												
			g_MCP23017.Digit2 = 0xC0;												// 7 SEG TO DISPLAY 10 													
			write_I2CExpander (&g_I2C0Handle, &g_MCP23017); // WRITE TO THE I2C EXPANDER
			g_bInitiate10sCountdown = TRUE; 								// SET THE 10S COUNTDOWN FLAG TO TRUE
			g_n7Seg10sCount = 10;														// SET THE 7 SEG COUNTDOWN FOR 10S TO 10
		}
		
		if(g_bInitiate10sCountdown == TRUE)	// IF 10S COUNTDOWN FLAG IS SET TO TRUE						
		{
						wrong_password_counter = 0; // RESET THE WRONG PASSWORD COUNTER
						g_bInitiateCountdown = FALSE;
						g_bInitiateCountdownAsterisk = FALSE;

						LOCK_KEYPAD = TRUE; //LOCK THE KEYPAD
						if(g_bCountDownFlag10s == TRUE)	// IF  COUNTDOWN FLAG IS SET 
					{
						g_bCountDownFlag10s = FALSE;	// RESET COUNT DOWN FLAG
						if(g_n7Seg10sCount == 0)			// IF 7 SEG COUNT VALUE IS EQUAL TO 0 
						{
							g_n7Seg10sCount = g_n7Seg10sCount;// 7 SEG VALUE REMAIN AT 0
						}
						if(g_n7Seg10sCount != 0) // IF 7 SEG VALUE NOT EQUAL TO 0 
						{

							g_n7Seg10sCount--;// DECREMENT THE 7 SEG VALUE
						}
						g_MCP23017.Digit1 = g_n7Seg10sCount/10;					// QUOTIENT OF 7 SEG VALUE IS THE LEFT DIGIT
						g_MCP23017.Digit2 = g_n7Seg10sCount%10;					// REMAINDER OF THE 7 SEG VALUE IS THE SECOND DIGIT
						write_I2CExpander (&g_I2C0Handle, &g_MCP23017);	// WRITE TO THE I2C EXPANDER 
						g_Buzzer = TRUE;																// SET THE MASTER BUZZER FLAG TO TRUE 
						COUNTDOWN_BUZZERFLAG = TRUE;										// SET THE COUNTFDOWN BUZZER FLAG TO TRUE
					}
		}
		
		if(g_n7Seg10sCount == 0) // IF 7 SEG COUNT REACHES 0
		{
			g_n7Seg10sCount = 10; 		// RESET THE 7 SEGCOUNT  TO 10 
			SEVENSEG_STOP10s = TRUE;	// STOP THE 10S COUNTDOWN
			g_bPOWERUP = TRUE;				// DISPLAY 60 ON THE 7 SEG
		}

		if(SEVENSEG_STOP10s == TRUE) // IF THE 7SEG COUNTDOOWN STOP FOR 10S IS SET TO TRUE
		{
				SEVENSEG_STOP10s = FALSE; 							// RESET THE SEG COUNTDOOWN STOP FOR 10S FLAG
				g_bInitiate10sCountdown = FALSE;				// RESET THE 10S COUNTDOWN FLAG
				g_bInitiate60sCountdownAfter10s = TRUE; // SET THE 60S COUNTDOWN AFTER 10S COUNTDOWN TO TRUE
				
		}
				
		
		
/******************************INITIATE 10S COUNTDOWN************************************/
				
/******************************INITIATE 60S COUNTDOWN************************************/	
		if(g_bInitiateCountdownAsterisk == TRUE && g_debounce == 0)// IF g_bInitiateCountdownAsterisk IS SET TO TRUE AND DEBOUNCE EQUALS TO 0
		{
			if(g_bInitiateCountdownAsterisk == TRUE)	//IF USER INPUTS THE FIRST DIGIT FLAG SET TO TRUE
			{
					g_bInitiateCountdownAsterisk = FALSE;	// RESET USER INPUTS THE FIRST DIGIT FLAG
					
  				  g_bInitiateCountdown = TRUE;				// SET 60S COUNTDOWN FLAG TO TRUE
				
					if(g_bInitiate60sCountdownAfter10s == TRUE)	// IF g_bInitiate60sCountdownAfter10s FLAG IS SET TO TRUE
					{
						g_bInitiate60sCountdownAfter10s = FALSE; // RESET g_bInitiate60sCountdownAfter10s FLAG
						g_n7SegCount = 60;											 // ASSIGN 7 SEG COUNT TO 60
					}
				
			}
		}
		
		if(g_bInitiateCountdown == TRUE )//IF INITIATE COUNTDOWN FOR 60S FLAG IS SET TO TRUE
				{
					
					g_bPOWERUP = FALSE;// RESET THE 7 SEGMENT TO STOP DISPLAYING 60
					if(g_bCountDownFlag == TRUE)// IF  COUNTDOWN FLAG IS SET 
					{
						g_bCountDownFlag = FALSE;// RESET COUNT DOWN FLAG
						if(g_n7SegCount == 0)// IF 7 SEG COUNT VALUE IS EQUAL TO 0 
						{
							g_n7SegCount = g_n7SegCount;// 7 SEG VALUE REMAIN AT 0
						}
						if(g_n7SegCount != 0)// IF 7 SEG VALUE NOT EQUAL TO 0 
						{
							g_n7SegCount--;// DECREMENT THE 7 SEG VALUE
						}
						g_MCP23017.Digit1 = g_n7SegCount/10;						// QUOTIENT OF 7 SEG VALUE IS THE LEFT DIGIT
						g_MCP23017.Digit2 = g_n7SegCount%10;						// REMAINDER OF THE 7 SEG VALUE IS THE SECOND DIGIT
						write_I2CExpander (&g_I2C0Handle, &g_MCP23017);	// WRITE TO THE I2C EXPANDER 
						g_Buzzer = TRUE;																// SET THE MASTER BUZZER FLAG TO TRUE 
						COUNTDOWN_BUZZERFLAG = TRUE;										// SET THE COUNTFDOWN BUZZER FLAG TO TRUE
					}
					
			}
				
						if(SEVENSEG_STOP == TRUE)  // IF 7 SEGMENT FLAG TO STOP IS SET TO TRUE 
				{
					SEVENSEG_STOP = FALSE;				 //RESET THE 7 SEGMENT FLAG TO STOP
					g_n7SegCount = g_n7SegCount;	 //DISPLAY CURRENT 7 SEG VALUE
					g_bInitiateCountdown = FALSE;  // RESET THE 60S COUNTDOWN FLAG
					
				}
		 				if(g_n7SegCount == 0)//IF 7 SEG COUNT REACHES 0 
				{
					LOCK_KEYPAD = TRUE;		//LOCK THE KEYPAD
					SEVENSEG_STOP = TRUE; //STOP THE 60S COUNTDOWN
				}
					
/******************************INITIATE 60S COUNTDOWN************************************/						
					
/******************************HARDWARE FLAGS*******************************************/	
		/* Keypad update */
		if( FALSE != g_nKeypadScan ) // IF KEYPADSCAN FLAG IS SET TO TRUE
		{ 
			g_nKeypadScan = FALSE; // RESET THE KEYPAD SCAN FLAG
			main_KeyScan();        // CALL OUT THE MAIN KEYSCAN FUNCTION
		}
		
		/* LCD update */
			if( FALSE != g_bLCDUpdate ) // IF LCDUPDATE FLAG IS SET TO TRUE
		{
			if( 0 != g_bLcdFree ) // IF LCDFREE FLAG IS SET TO TRUE
			{
				g_bLCDUpdate = FALSE;// RESET THE LCDUPDATE FLAG
				g_bLcdFree = FALSE;  // RESET LCDFREE FLAG
				
				/* Draw every block. Consumes less time  */
				GUI_Draw_Exe();      // CALL OUT GUI DRAW FUNCTION
			}
		}
				
		if(g_bAdcUpdate != FALSE && ADC0_BUSY() == 0) // IF ADCUPDATE FLAG IS SET TO TRUE AND ADC0 IS NOT BUSY
			{ 
				g_bAdcUpdate = FALSE; // RESET THE ADC UPDATE FLAG
				main_AdcUpdate();     // CALL OUT MAIN ADC UPDATE FUNCTION
		}		
					
/******************************HARDWARE FLAGS************************************/					
				
	}
}	

/*****************************************************************************
 Callback functions
******************************************************************************/
void SysTick_Handler( void )  
{
	
			if (0 != g_nADCUpdate){
		g_nADCUpdate--;
			
			if(0 == g_nADCUpdate){
				g_bAdcUpdate = TRUE;
				g_nADCUpdate = ADC_UPDATE_MS;
		}
	}
	
		g_bSystemTick = TRUE;

		
		g_nLEDCount++;//increment led count
	if(g_nLEDCount == 300)//if led count reaches 0.3s
	{
		g_bLedTick = TRUE;//set led tick to true
		g_nLEDCount = 0;//reset led count to 0 
	}
/***************************TEMP DROP/RISE BUZZER(SLIDER)************************/	
	
			g_nBUZZCount++;//increment led count
	if(g_nBUZZCount == 300)//if led count reaches 0.3s
	{
		g_bBUZZTick = TRUE;//set led tick to true
		g_nBUZZCount = 0;//reset led count to 0 
	}
/***************************TEMP DROP/RISE BUZZER(SLIDER)************************/

/***************************7SEG COUNTDOWN BUZZER(OPEN/CLOSE SAFE)************************/	
		if(g_nBeep_Count != 0){//if beep count not equal to 0 
			g_nBeep_Count--;//decrement bee count
		}
		if(g_nBeep_Count == 0){//if beep count reaches 0 
			BUZZER_OFF();//turn off buzzer
		}
/***************************7SEG COUNTDOWN BUZZER(OPEN/CLOSE SAFE)************************/	
	
	if(g_debounce != 0){//if debounce not equal to 0
		g_debounce--;//decrement the debounce until it equals to 0 
	}
		
	/* Provide system tick */
  g_nCount++;
  if (g_nCount == 1000)
  {
    g_bSecTick = TRUE;
		g_bSystemTick1000 = TRUE;
		g_nCount=0;
		
		/* Keep track of time based on 1 sec interval */ 
		g_nTimeSec++;
		if(g_nTimeSec > 24*60*60)
		{
			g_nTimeSec = 0;
		}
		
  }
		
		if(g_bInitiateCountdown60safter10s == TRUE)//if mode is DOWN
		{
			g_nCountDown60sAfter10s++;//increment countdown timing
		}

		if(g_bInitiate10sCountdown == TRUE)//if mode is DOWN
		{
			g_nCountDown10s++;//increment countdown timing
		}
			
		if(g_bInitiateCountdown == TRUE)//if mode is DOWN
		{
			g_nCountDown++;//increment countdown timing
		}
		
		
		if(g_nCountDown60sAfter10s == 1000)//once count down timing reaches 1000 (1s)
		{
			g_bCountDownFlag60sAfter10s = TRUE;//set count down flag to true
			g_nCountDown60sAfter10s = 0;//reset count down timing
		}	
		
		if(g_nCountDown == 1000)//once count down timing reaches 1000 (1s)
		{
			g_bCountDownFlag = TRUE;//set count down flag to true
			g_nCountDown = 0;//reset count down timing
		}	
		
				if(g_nCountDown10s == 1000)//once count down timing reaches 1000 (1s)
		{
			g_bCountDownFlag10s = TRUE;//set count down flag to true
			g_nCountDown10s = 0;//reset count down timing
		}
		
		/**  LCD flag	**/
		if( 0 != g_nLCD )
	{
		g_nLCD--;
		if( 0 == g_nLCD )
		{
			g_nLCD = LCD_UPDATE_MS;
			g_bLCDUpdate = TRUE;
		}
	}
	
		/**  Keypad flag	**/
	if( 0 != g_nKeypad )
	{
		g_nKeypad--;
		if( 0 == g_nKeypad )
		{
			g_nKeypad = KEYPAD_UPDATE_MS;
			g_nKeypadScan = TRUE;
		}
	}
	
		/** Motor flag **/
	if (0 != g_nMotorCount)
	{
		g_nMotorCount--;
		if (0 == g_nMotorCount)
		{
			g_bmotor_move = TRUE;
			g_nMotorCount = MOTOR_STEP_TIME;
		}		
	}
}

void GUI_AppDraw( BOOL bFrameStart )
{
/* This function invokes from GUI library */
	char buf[128];
		
			GUI_Clear( ClrBisque  ); 
			GUI_SetColor(ClrBlack);
			GUI_DrawFilledRect(3,3, 124, 40);
			GUI_SetFont( &FONT_Arialbold12 );
		
			GUI_SetColor(ClrBlack);
			GUI_DrawFilledRect(3, 76,124,135);
			GUI_SetFont( &FONT_Arialbold12 );	
			GUI_PrintString( "--  Digital Lock  --", ClrTurquoise, 12, 8 ); 
			GUI_PrintString( "ENTER PASSWORD", ClrWhite, 7, 27 );

			GUI_SetFont( &FONT_Arialbold24 );

			GUI_SetColor(ClrOrange);
			GUI_DrawFilledRect(23, 46, 43,70);
			sprintf( buf, "%c", print_H);
			GUI_PrintString( buf, ClrBlack, 30, 50 );

			GUI_SetColor(ClrCornflowerBlue);
			GUI_DrawFilledRect(53,46,73,70);
			sprintf( buf, "%c",print_T);
			GUI_PrintString( buf, ClrBlack, 60, 50 );

			GUI_SetColor(ClrCrimson);
			GUI_DrawFilledRect(83,46,103,70);
			sprintf( buf, "%c",print_O );
			GUI_PrintString( buf, ClrBlack, 90, 50 );

			if ((acttemperature > (settemperature + 2.0)) && (CORRECTPASSWORD == FALSE) && (g_bOpenSafe != TRUE))
			{
			GUI_SetFont( &FONT_Arialbold12 );
			GUI_PrintString( "ALERT! ALERT!", ClrOrange, 20, 85 );
			GUI_PrintString( "TEMP DROP", ClrCyan   , 28, 105 );
			
			sprintf(buf," %.1fC", settemperature);
			GUI_PrintString( buf, ClrCyan , 43, 120);

	
			}
			
//		/* If Actual is <0.5°C compared to Set, LED = RED (heater). */
			if (acttemperature < (settemperature - 2.0) )
				{
				GUI_SetFont( &FONT_Arialbold12 );
				GUI_PrintString( "ALERT! ALERT!", ClrOrange, 20, 85 );
				GUI_PrintString( "TEMP RISE", ClrRed  , 28, 105 );
				
				sprintf(buf," %.1fC", settemperature);
				GUI_PrintString( buf, ClrRed, 43, 120);
			}		
		
		if((UserInput == 138 && (g_cKey == '#')) && (SW2_Pressed == FALSE) && (g_bOpenSafe == TRUE)) // IF USER INPUTS THE CORRECT PASSWORD
		{
			GUI_SetFont( &FONT_Arialbold12 );
			GUI_DrawBitmap(open[g_nCount/(1000/FRAME_COUNT)], 9, 80);
			GUI_DrawBitmap(open[g_nCount/(1000/FRAME_COUNT)], 55, 80);
			GUI_DrawBitmap(open[g_nCount/(1000/FRAME_COUNT)], 100, 80);
			GUI_SetColor(ClrBisque);
			GUI_DrawFilledRect(0,103,125,135);
			GUI_PrintString( "ACCESS GRANTED", ClrGreen  , 10, 105 );
			GUI_PrintString( "WELCOME HOME", ClrGreen  , 12, 120 );
		}
		
			if(((UserInput != 138) && g_cKey == '#') && g_bCLOSE_SAFE_ONLY_WHEN_OPENED == FALSE && (g_bOpenSafe == FALSE)) // IF USER INPUTS THE WRONG PASSWORD
		{
			GUI_SetFont( &FONT_Arialbold12 );
			GUI_SetColor(ClrRed);
			GUI_DrawFilledRect(7,89,25,90);
			GUI_DrawFilledRect(35,89, 53, 90);
			GUI_DrawFilledRect(72,89, 90, 90);
			GUI_DrawFilledRect(100,89, 118, 90);
			GUI_SetColor(ClrBisque);
			GUI_DrawFilledRect(0,103,125,135);
			GUI_PrintString( "! ACCESS DENIED !", ClrRed , 9, 105 );
			GUI_PrintString( "TRY AGAIN", ClrRed , 30, 120 );
			
			if(g_bOpenSafe == FALSE && (SW2_Pressed == TRUE) && ((g_debounce == 0)) && g_MCP23017.Mode == UNLOCK && g_bCLOSE_SAFE_ONLY_WHEN_OPENED == TRUE)
			{
			GUI_SetFont( &FONT_Arialbold12 );
			GUI_SetColor(ClrWhite);
			GUI_DrawFilledRect(7,110,118,130);
			GUI_PrintString( "DOOR CLOSING", ClrDarkMagenta , 18, 115 ); // DISPLAY DOOR CLOSING ON LCD
			}
		}
		
		if (g_bOpenSafe == FALSE && (SW2_Pressed == TRUE) && ((g_debounce == 0)) && g_MCP23017.Mode == UNLOCK && g_bCLOSE_SAFE_ONLY_WHEN_OPENED == TRUE)
		{
			GUI_SetFont( &FONT_Arialbold12 );
			GUI_SetColor(ClrWhite);
			GUI_DrawFilledRect(7,110,118,130);
			GUI_DrawBitmap(frames[g_nCount/(1000/FRAME_COUNT)], 9, 82);
			GUI_DrawBitmap(frames[g_nCount/(1000/FRAME_COUNT)], 55, 82);
			GUI_DrawBitmap(frames[g_nCount/(1000/FRAME_COUNT)], 100, 82);
			GUI_PrintString( "DOOR CLOSING", ClrDarkMagenta , 18, 115 ); // DISPLAY DOOR CLOSING ON LCD
			
		}
		
		GUI_SetColor( ClrBlack );
		GUI_DrawFilledRect( 0, 140, 127, 159);
		GUI_SetFont( &FONT_Arialbold12 );
		GUI_SetFontBackColor( ClrLightCyan );
		sprintf( buf, "%02u:%02u:%02u", (g_nTimeSec/3600)%24, (g_nTimeSec/60)%60, g_nTimeSec%60 );
		GUI_PrintString( buf, ClrWhite, 35, 145 );
}

static void main_cbLcdTransferDone( void )
{
	g_bLcdFree = TRUE;
}

static void main_cbGuiFrameEnd( void )
{
	g_bLcdFree = TRUE;
}

void main_cbI2C0Isdone ( void )
{
	g_bI2C0IsBusy = FALSE; 
}

/*****************************************************************************
 Local functions
******************************************************************************/
static void main_KeyScan( void )
{
  int nRow, nCol, input;


  /* Set all rows to high so that if any key is pressed, 
     a falling edge on the column line can be detected */
  KEYPAD_ALL_ROWS_ON();

  for( nRow=0; nRow<4; nRow++ )
  {
    /* Pull row by row low to determined which button is pressed */
    KEPAD_ROW_MASKED &= ~(1U << nRow);
		
    /* Short delay to stabalize row that has just been pulled low */
    __nop(); __nop(); __nop();
		
    /* Read input */
    input = KEYPAD_COL_IN();	
	
    /* If a column is asserted, then look for the corresponding key 
       Make use of the KEY_DECODE_TABLE, exit loop once key found!  */	
		
/**********************************ENTER PASSWORD**************************************************/		
		    /* If a column is asserted, then look for the corresponding key 
       Make use of the KEY_DECODE_TABLE, exit loop once key found!  */
		if(((input != 7 ) && (bKeyPressed == FALSE) && (g_bpressed == FALSE) && (g_debounce == 0)) )// IF INPUT IS NOT EQUAL TO 7, DEBOUNCE=0, bKeyPressed SET TO FALSE, g_bpressedSET TO FALSE
			{
		if (input == 6)
			{
				nCol = 0;
			}
			else if (input == 5)
			{
				nCol = 1;
			}
			else if (input == 3)
			{
				nCol = 2;
			}
			
			if (bKeyPressed == FALSE)
			{
				g_cKey = KEY_DECODE_TABLE[nRow][nCol];
				print_H = '*'; // DISPLAY 3 ASTERISKS TO COVER USER'S PASSWORD
				print_T = '*'; // DISPLAY 3 ASTERISKS TO COVER USER'S PASSWORD
				print_O = '*'; // DISPLAY 3 ASTERISKS TO COVER USER'S PASSWORD
				bKeyPressed = TRUE; // SET bKeyPressed FLAG TO TRUE 

			}				
			
			if((g_cKey != '*') && (g_cKey != '#') && (counter_digit < 3)) // IF 
				{
					
						counter_digit++;									// INCREMENT COUNTER TO TENS, ONES, AND HUNDREDS PLACE		
																							// COUNT MUST NOT BE MORE THAN 999, INPUT FROM 0 TO 99	
					if(counter_digit == 1)							// IF COUNTER DIGIT IS EQUAL TO 1
					{												
						g_bSTORE_Hundredth  = g_cKey - '0';		// HUNDREDS		
						g_bInitiateCountdownAsterisk = TRUE;  // SET 60S COUNTDOWN FLAG TO TRUE
						
						if(g_bSTORE_Hundredth  > 9)						//IF HUNDREDS PLACE IS MORE THAN 9 
						{												
							counter_digit = 0;										//COUNTER RESET TO 0 SO THAT IT WILL LOOK FOR THE HUNDREDS PLACE
						} 
						
						else 
						{
							g_bHundredth  = g_cKey - '0';				// CONVERTING ASCII TO HEXADECIMAL
							print_H  = g_cKey;									// DISPLAY VALUE FOR HUNDRED PLACE TO LCD
						}
					}
		
				if(counter_digit == 2)
				{												
				g_bSTORE_Tens  = g_cKey - '0';									 // TENS
				if((g_bHundredth  == 9) && (g_bSTORE_Tens  > 9)) // TENS PLACE CANNOT BE MORE THAN 2tens place cannot be more than 2
				{			
					counter_digit = 0;												// IF TENS PLACE IS > 2, RESET DIGIT TO 0 SO THAT IT WILL LOOK FOR THE VALID TENS PLACE	
				} 
				
				else 
				{
					g_bTens  = g_cKey - '0';							// CONVERTING ASCII TO HEXADECIMAL
					print_T  = g_cKey;										// DISPLAY VALUE FOR TENS PLACE TO LCD
				}
			}
			
			if(counter_digit == 3){												
				g_bSTORE_Ones  = g_cKey - '0';																			// ONES
				if((g_bHundredth == 9) && (g_bTens  == 9) && ( g_bSTORE_Ones  > 9)) // IF TENS PLACE IS 9 AND ONES PLACE IS 9 WHEN HUNDRED IS = 9, RESET TO 0
				{				
					counter_digit = 0;
				} 
				
				else
				{
					g_bOnes  = g_cKey - '0';							// CONVERTING ASCII TO HEXADECIMAL
					print_O  = g_cKey;										// DISPLAY VALUE FOR HUNDRED PLACE TO LCD
				}
			}
					
		}// END OF DIGIT INPUT
		
			if(g_cKey == '#') // IF USER PRESSES THE HASHTAG
			{	
				g_bInitiateCountdownAsterisk = FALSE; 													// RESET THE 60S COUNTDOWN FLAG
				UserInput = (g_bHundredth  * 100) + (g_bTens  * 10) + g_bOnes;	// POSITIONING THE USER INPUT AS A 3 DIGIT NUMBER 						
				
				if(UserInput == 138 && g_bpressed == FALSE) // IF USER INPUTS THE CORRECT PASSWORD
				{
					CORRECTPASSWORD = TRUE;  // SET CORRECT PASSWORD FLAG TO TRUE 
					SEVENSEG_STOP = TRUE;		 // STOP 60S COUNTDOWN
					g_bOpenSafe = TRUE;			 // SET OPEN SAFE FLAG TO TRUE
					LED_RGB_SET(RGB_GREEN);	 // TURN ON GREEN LED
				}
				
				if(UserInput != 138) // IF USER INPUTS THE WRONG PASSWORD 
				{
					wrong_password_counter++; 		//INCREMENT WRONG PASSWORD COUNTER
					wrong_password_input = TRUE;	// SET WRONG PASSWORD INPUT FLAG TO TRUE
					g_bOpenSafe = FALSE;					// SET OPEN SAFE FLAG TO FALSE
				}
				g_bpressed = TRUE;	// SET g_bpressed FLAG TO TRUE
			}
		}
/**********************************ENTER PASSWORD**************************************************/			 		
	}//end of FOR loop
 
	/* Check if key is released */
		if(input == 7 && bKeyPressed == TRUE) // IF INPUT IS EQUAL TO 7 AND bKeyPressed FLAG SET TO TRUE
		{
			bKeyPressed = FALSE; // RESET bKeyPressed FLAG
		}
		
  /* Check if key is released */

  /* Reset all rows for next key detection */
  KEYPAD_ALL_ROWS_OFF();
}

	

static void main_LcdInit( void )
{
	int screenx;
	int screeny;
	
	/* g_SpimHandle shall be initialized before use */
	
	/* Choosing a landscape orientation */
	LcdInit( &g_SpimHandle, LCD_POTRAIT_180 );
	
	/* Get physical LCD size in pixels */
	LCD_GetSize( &screenx, &screeny );
	
	/* Initialize GUI */
	GUI_Init(
		&g_MemDev,
		screenx,
		screeny,
		g_aBuf,
		sizeof(g_aBuf) );
	
	/* Switch to transfer word for faster performance */
	SpimSetDataSize( &g_SpimHandle, SPI_DATA_SIZE_16 );
	GUI_16BitPerPixel( TRUE );
	
	/* Clear LCD screen to Blue */
	GUI_Clear( ClrBlue );

  /* set font color background */
  GUI_SetFontBackColor( ClrBlue );
    
  /* Set font */
	GUI_SetFont( &g_sFontCalibri10 );
	
	LCD_AddCallback( main_cbLcdTransferDone );
	
	GUI_AddCbFrameEnd( main_cbGuiFrameEnd );
	
	/* Backlight ON */
	LCD_BL_ON();
}

void motor_output (int state)
{
	if(g_bMotor_ACW  == 0) // IF MOTOR_ACW FLAG SET TO FALSE
	{						
		GPIOC->DATA = fsm [state].Out; // ROTATE THE MOTOR CW
	}
	if(g_bMotor_ACW  == 1) // IF MOTOR_ACW FLAG SET TO TRUE
	{						
		GPIOC->DATA = afsm [state].Out; //ROTATE THE MOTOR ACW
	}
}

void main_AdcInit(void) {
	/** Set SS0 for AIN8, AIN9 **/
	
	ADC0->ACTSS &= ~ADC_ACTSS_ASEN0;								/* Disable ADC while initializing */
	ADC0->EMUX |= ADC_EMUX_EM0_PROCESSOR; 					/* Set Processer as trigger */
	ADC0->PC |= ADC_PC_SR_125K; 										/* sample at 125K */
	ADC0->SSPRI = (ADC_SSPRI_SS0_M & 0x03);					/* Set priority for SS0 */
	ADC0->SSMUX0 |= (0x08 << ADC_SSMUX0_MUX0_S); 		/* AIN8 for Step 0 */
	ADC0->SSMUX0 |= (0x09 << ADC_SSMUX0_MUX1_S); 		/* AIN9 for Step 1 */
	ADC0->SSMUX0 |= (0x0A << ADC_SSMUX0_MUX2_S); 		/* AIN10 for Step 2 */
	ADC0->SSCTL0 |= ADC_SSCTL0_END2; 								/* Set 2nd sample as End of Seq */
	ADC0->SAC |= ADC_SAC_AVG_32X; 									/* Set hardware averaging to 32X*/
	ADC0->ACTSS |= ADC_ACTSS_ASEN0; 								/* Enable ADC */
	ADC0_SS0_Start(); 															/* Start first measurement */

}

void main_AdcUpdate(void)
{
	uint16_t AIN8Result, AIN9Result, AIN10Result;
	
	/* Read ADC measurements */
	AIN8Result = ADC0_GET_FIFO();
	AIN9Result = ADC0_GET_FIFO();
	AIN10Result = ADC0_GET_FIFO();
	
	/* insert other program codes */
	acttemperature 	= (AIN10Result - 1.0)/(10*7.5);
	LDR 					 	= (AIN9Result/4130.0)*100;
	slider 				 	= (AIN8Result/341.3)*100;
	settemperature 	=  slider/10.0 - 20;
	
	/* Start next measurement */
	ADC0_SS0_Start();
}
/*****************************************************************************
 Interrupt functions
******************************************************************************/
/* GPIOF_Button_IRQHandler */
void GPIOF_Button_IRQHandler( uint32_t Status )
{
		if((Status & BIT(PF_SW2)) != FALSE ) //IF SWITCH 2 IS PRESSED
	{
		SW2_Pressed  = TRUE;// SET SWITCH 2 IS PRESSED FLAG TO TRUE 
		g_debounce = 20;		// ASSIGN 20 TO DEBOUNCE
	}
	
	GPIOF->ICR |= BIT(PF_SW1)|BIT(PF_SW2); /* CLEAR INTERRUPT  */		
}  
/* ADC0 IRQ HANDLER */
extern void ADC0_IRQHandler(uint32_t Status){
	
	if(0 != (Status & ADC_RIS_INR0)) 
	{
		ADC0->ISC |= ADC_ISC_IN0;
	}
}
	
