/**
*
* @file project1_surya.c
*
* @author Surya Ravikumar (surya@pdx.edu)
*
*
* @Description:	This program reads in a user entered Hue, Saturation, value parameters
* 				using Pmod Rotary encoder and pushbuttons on Nexys A7, and converts
* 				it to RGB before writing the values to Pmod Oled RGB and onboard RGB LEDs.
*
* 				The program also detects the duty cycle of the RGB leds by counting
* 				numbers iterations a signal was high during a total period.
*
* 				It also reads in gpio ports to get values for duty cycle calculated
* 				by a hardware module
******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "platform.h"
#include "xparameters.h"
#include "xstatus.h"
#include "microblaze_sleep.h"
#include "nexys4IO.h"
#include "PmodOLEDrgb.h"
#include "PmodENC.h"
#include "xgpio.h"
#include "xintc.h"
#include "xtmrctr.h"

/************************** Constant Definitions ****************************/

// Clock frequencies
#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ

// AXI timer parameters
#define AXI_TIMER_DEVICE_ID		XPAR_AXI_TIMER_0_DEVICE_ID
#define AXI_TIMER_BASEADDR		XPAR_AXI_TIMER_0_BASEADDR
#define AXI_TIMER_HIGHADDR		XPAR_AXI_TIMER_0_HIGHADDR
#define TmrCtrNumber			0


// Definitions for peripheral NEXYS4IO
#define NX4IO_DEVICE_ID		XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR		XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR		XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

// Definitions for peripheral PMODOLEDRGB
#define RGBDSPLY_DEVICE_ID		XPAR_PMODOLEDRGB_0_DEVICE_ID
#define RGBDSPLY_GPIO_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR
#define RGBDSPLY_GPIO_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_HIGHADD
#define RGBDSPLY_SPI_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR
#define RGBDSPLY_SPI_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_HIGHADDR

// Definitions for peripheral PMODENC
#define PMODENC_DEVICE_ID		XPAR_PMODENC_0_DEVICE_ID
#define PMODENC_BASEADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_BASEADDR
#define PMODENC_HIGHADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_HIGHADDR

// Fixed Interval timer - 100 MHz input clock, 40KHz output clock
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001
#define FIT_IN_CLOCK_FREQ_HZ	CPU_CLOCK_FREQ_HZ
#define FIT_CLOCK_FREQ_HZ		40000
#define FIT_COUNT				(FIT_IN_CLOCK_FREQ_HZ / FIT_CLOCK_FREQ_HZ)
#define FIT_COUNT_1MSEC			40

// GPIO parameters
#define GPIO_0_DEVICE_ID			XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_0_INPUT_0_CHANNEL		1
#define GPIO_0_OUTPUT_0_CHANNEL		2

#define GPIO_1_DEVICE_ID			XPAR_AXI_GPIO_1_DEVICE_ID
#define	GPIO_1_INPUT_1_CHANNEL		1
#define	GPIO_1_INPUT_2_CHANNEL		2

#define GPIO_2_DEVICE_ID			XPAR_AXI_GPIO_2_DEVICE_ID
#define	GPIO_2_INPUT_1_CHANNEL		1
#define	GPIO_2_INPUT_2_CHANNEL		2

// Interrupt Controller parameters
#define INTC_DEVICE_ID			XPAR_INTC_0_DEVICE_ID
#define FIT_INTERRUPT_ID		XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Variable Definitions ****************************/
// Microblaze peripheral instances
PmodOLEDrgb	pmodOLEDrgb_inst;
PmodENC 	pmodENC_inst;
XGpio		GPIOInst0;					// GPIO instance
XGpio		GPIOInst1;					//red
XGpio		GPIOInst2;					//green

XIntc 		IntrptCtlrInst;				// Interrupt Controller instance
XTmrCtr		AXITimerInst;				// PWM timer instance


// The following variables are shared between non-interrupt processing and
// interrupt processing such that they must be global(and declared volatile)
// These variables are controlled by the FIT timer interrupt handler


volatile u32		gpio_in;			// GPIO input ports
volatile u32 		red_gpio_in;
volatile u32		green_gpio_in;
volatile u32		blue_gpio_in;
volatile u32		count_gpio_in;


/************************** Function Prototypes *****************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix);
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num);
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix);
int	 do_init(void);											// initialize system
void FIT_Handler(void);										// fixed interval timer interrupt handler
int AXI_Timer_initialize(void);
void hsv2rgb(u8 *R, u8* G, u8 *B, s16 H, s8 S, s8 V);	//hsv to rgb convert
void displayToPmod(s16 hue, s8 sat, s8 val, u8 R, u8 G, u8 B);	//display info to pmod

void projectloop(void);		//main loop for project one


/************************** MAIN PROGRAM ************************************/
int main(void)
{
    init_platform();

	uint32_t sts;

	sts = do_init();
	if (XST_SUCCESS != sts)
	{
		exit(1);
	}

	microblaze_enable_interrupts();

	projectloop();
	
	//turn of leds
	NX4IO_RGBLED_setChnlEn(RGB1, false, false, false);
	NX4IO_RGBLED_setChnlEn(RGB2, false, false, false);

	// blank the display digits and turn off the decimal points
	NX410_SSEG_setAllDigits(SSEGLO, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
	
	//display bye bye on pmod and 7 seg display
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 2, 3);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"BYE BYE"); 
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_B, CC_LCY, CC_E, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGLO, CC_B, CC_LCY, CC_E, CC_BLANK, DP_NONE);
	usleep(5000 * 1000);
	
	// clear the displays and power down the pmodOLEDrbg

	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_end(&pmodOLEDrgb_inst);
	
	// cleanup and exit
    cleanup_platform();
    exit(0);
}



void hsv2rgb(u8 *R, u8* G, u8 *B, s16 H, s8 s, s8 v)
{
//function to convert hsv to rgb
//h, s, v values passed in
//r, g, b values calculated and "returned" using pointers
//code based on stackoverflow linked posted in lecture
	
	float hh, p, q, t, ff;
	float S=s/100.0, V=v/100.0;

	hh = H/60.0;
	ff = hh - (u16)(H/60);

	p = V * (1.0 - S);
	q = V * (1.0 - (S * ff));
	t = V * (1.0 - S * (1.0 - ff));

	if(H < 60)
	{
		*R = V * 255;
		*G = t * 255;
		*B = p * 255;
	}

	else if(H < 120)
	{
		*R = q * 255;
		*G = V * 255;
		*B = p * 255;
	}

	else if(H < 180)
	{
		*R = p * 255;
		*G = V * 255;
		*B = t * 255;
	}

	else if(H < 240)
	{
		*R = p * 255;
		*G = q * 255;
		*B = V * 255;
	}

	else if(H < 300)
	{
		*R = t * 255;
		*G = p * 255;
		*B = V * 255;
	}

	else if(H < 360)
	{
		*R = V * 255;
		*G = p * 255;
		*B = q * 255;
	}
}

void displayToPmod(s16 hue, s8 sat, s8 val, u8 R, u8 G, u8 B)
{
	//function to display HSV values and color box in Pmod
	//takes HSV values
	//also pass in RGB values to display for checking duty cycle purpose
	
	//display color box
	OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst, 60, 10, 90, 40, 0, 1, OLEDrgb_BuildRGB(R, G, B));

	//display hue
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 1);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 1);
	PMDIO_putnum(&pmodOLEDrgb_inst, hue, 10);

	//display saturation
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
	PMDIO_putnum(&pmodOLEDrgb_inst, sat, 10);
	
	//display value
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 5);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"   ");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 5);
	PMDIO_putnum(&pmodOLEDrgb_inst, val, 10);


	//display RGB values...
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 7);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"           ");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 7);
	PMDIO_putnum(&pmodOLEDrgb_inst, R, 10);
	OLEDrgb_PutString(&pmodOLEDrgb_inst," ");
	PMDIO_putnum(&pmodOLEDrgb_inst, G, 10);
	OLEDrgb_PutString(&pmodOLEDrgb_inst," ");
	PMDIO_putnum(&pmodOLEDrgb_inst, B, 10);
}



void projectloop(void)
{
	//function that implements all actions for project 1
	
	u32 state, laststate; //comparing current and previous state to detect edges on GPIO pins.
	s16 hue = 0, lasthue = 1;	//compare present and previos hue

	u32 count = 0;	//variable to count period
	u32 onesR=0, onesG=0, onesB=0;	//variables that hold high counts of R G B signals
	u32 aveR=0, aveG=0, aveB=0;		//variables that hold the duty cycle of R G B signals

	s8 sat = 100, lastsat = 1, val = 100, lastval = 1;	//present and previous saturation and value

	u8 R=0, G=0, B=0;		//value to hold RGB values
	
	//enable duty cycle for all channels...both LEDs
	NX4IO_RGBLED_setChnlEn(RGB2, true, true, true);
	NX4IO_RGBLED_setChnlEn(RGB1, true, true, true);

	// turn off all of the decimal points
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT7, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT6, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT5, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT4, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT3, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT2, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT1, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT0, false);

	// Set up the display output
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst,OLEDrgb_BuildRGB(200, 12, 44));
	
	//print Hue Sat Val tags
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 1);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Hue:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 3);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Sat:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 5);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"Val:");


	// get the previous state
	laststate = ENC_getState(&pmodENC_inst);

	//clear 7 seg display
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGLO, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
	//clear LEDs
	NX4IO_setLEDs(0);

	while(1)
	{
		// get the PmodENC state
		state = ENC_getState(&pmodENC_inst);

		if (ENC_buttonPressed(state) && !ENC_buttonPressed(laststate))//only check on button posedge
		{
			break;
		}

		//check if center button pressed
		if (NX4IO_isPressed(BTNC))
		{
			break;
		}

		//if right button pressed
		if(NX4IO_isPressed(BTNR))
		{
			//if saturation is less than 100 then increment
			if(sat < 100)
				sat++;

			//if not, retain at 100
			else
				sat=100;
		}

		//if left button pressed
		if(NX4IO_isPressed(BTNL))
		{
			//if saturation above 0 then decrement
			if(sat > 0)
				sat--;

			//if not, then retain at 0
			else
				sat = 0;
		}

		//check if up button pressed
		if(NX4IO_isPressed(BTNU))
		{
			//if value is less than 100 then increment
			if(val < 100)
				val++;

			//if not, retain at 100
			else
				val=100;
		}

		//check if down button pressed
		if(NX4IO_isPressed(BTND))
		{
			//if value above 0 then decrement
			if(val > 0)
				val--;

			//if not, then retain at 0
			else
				val = 0;
		}

		//check for rotation and update rotation which is Hue
		hue += ENC_getRotation(state, laststate);

		//wrap back to 0 degrees if new hue is over 39 degrees
		if(hue > 359)
			hue = 0;

		//wrap back to 359 degrees if new hue degree is less than 0
		if(hue < 0)
			hue = 359;

		//calculate RGB
		hsv2rgb(&R, &G, &B, hue, sat, val);


		//read in gpio ports
		gpio_in = XGpio_DiscreteRead(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL);
		
		//hardware high counts and total count
		red_gpio_in = XGpio_DiscreteRead(&GPIOInst1, GPIO_1_INPUT_1_CHANNEL);
		green_gpio_in = XGpio_DiscreteRead(&GPIOInst1, GPIO_1_INPUT_2_CHANNEL);
		blue_gpio_in = XGpio_DiscreteRead(&GPIOInst2, GPIO_2_INPUT_1_CHANNEL);
		count_gpio_in = XGpio_DiscreteRead(&GPIOInst2, GPIO_2_INPUT_2_CHANNEL);

		//if hardware detection selected...slide switch 0
		if((NX4IO_getSwitches() & 0x1))
		{
			//if total count returned by hardware within the range of predetermined period
			if(count_gpio_in > 0x8f8000)
			{
				//calculate duty cyle
				aveR = (red_gpio_in*200)/count_gpio_in;
				aveG = (green_gpio_in*200)/count_gpio_in;
				aveB = (blue_gpio_in*200)/count_gpio_in;
			}

			//reset software high count variables
			onesR = 0;
			onesG = 0;
			onesB = 0;
			count = 0;

			//turn on led to show hardware detection selcted
			NX4IO_setLEDs(1);

		}

		//if software detection selected
		else
		{
			//increment high count variables if needed by masking out respective bits from gpio port read
			//increments if high, same if low
			onesR += (gpio_in >> 2) & 0x1;
			onesG += (gpio_in >> 0) & 0x1;
			onesB += (gpio_in >> 1) & 0x1;
			count++;

			//if count reaches predetermined period
			if(count == 10000)
			{
				//calculate duty cycle
				aveR = (onesR * 100) / count;
				aveG = (onesG * 100) / count;
				aveB = (onesB * 100) / count;

				aveR = aveR * 2;
				aveB = aveB * 2;
				aveG = aveG * 2;

				//reset counters
				onesR = 0;
				onesG = 0;
				onesB = 0;
				count = 0;
			}
			
			//turn of led 0 to show software detection
			NX4IO_setLEDs(0);
		}
		
		//if hue or saturation or value has changed, then display new values on pmod and set new values for RGB leds
		if (hue != lasthue || sat != lastsat || val != lastval)
		{
			displayToPmod(hue, sat, val, R, G, B);
			NX4IO_RGBLED_setDutyCycle(RGB1, R, G, B);
			NX4IO_RGBLED_setDutyCycle(RGB2, R, G, B);
		}

		//cap off duty cycle at 99%
		if(aveR >= 100)
			aveR = 99;

		if(aveG >= 100)
			aveG = 99;

		if(aveB >= 100)
			aveB = 99;

		//display to 7 seg
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT0, (aveB%10));
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT1, ((aveB/10)%10));
		NX4IO_SSEG_setDigit(SSEGLO, DIGIT3, (aveG%10));
		NX4IO_SSEG_setDigit(SSEGHI, DIGIT4, ((aveG/10)%10));
		NX4IO_SSEG_setDigit(SSEGHI, DIGIT6, (aveR%10));
		NX4IO_SSEG_setDigit(SSEGHI, DIGIT7, ((aveR/10)%10));

		//update previous stat with present state
		lastsat = sat;
		lastval = val;
		laststate = state;
		lasthue = hue;

	} // rotary button has been pressed - exit the loop


	//clear pmod
 	OLEDrgb_Clear(&pmodOLEDrgb_inst);

	return;
}


/**************************** HELPER FUNCTIONS ******************************/

/****************************************************************************/
/**
* initialize the system
*
* This function is executed once at start-up and after resets.  It initializes
* the peripherals and registers the interrupt handler(s)
*****************************************************************************/

int	 do_init(void)
{
	uint32_t status;				// status from Xilinx Lib calls

	// initialize the Nexys4 driver and (some of)the devices
	status = (uint32_t) NX4IO_initialize(NX4IO_BASEADDR);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// set all of the display digits to blanks and turn off
	// the decimal points using the "raw" set functions.
	// These registers are formatted according to the spec
	// and should remain unchanged when written to Nexys4IO...
	// something else to check w/ the debugger when we bring the
	// drivers up for the first time
	NX4IO_SSEG_setSSEG_DATA(SSEGHI, 0x0058E30E);
	NX4IO_SSEG_setSSEG_DATA(SSEGLO, 0x00144116);

	OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);

	// initialize the pmodENC and hardware
	ENC_begin(&pmodENC_inst, PMODENC_BASEADDR);

	// initialize the GPIO instances
	status = XGpio_Initialize(&GPIOInst0, GPIO_0_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	status = XGpio_Initialize(&GPIOInst1, GPIO_1_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	status = XGpio_Initialize(&GPIOInst2, GPIO_2_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}
	// GPIO0 channel 1 is an 8-bit input port.
	// GPIO0 channel 2 is an 8-bit output port.
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL, 0xFF);
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_OUTPUT_0_CHANNEL, 0x00);

	XGpio_SetDataDirection(&GPIOInst1, GPIO_1_INPUT_1_CHANNEL, 0xFFFFFFFF);
	XGpio_SetDataDirection(&GPIOInst1, GPIO_1_INPUT_2_CHANNEL, 0xFFFFFFFF);

	XGpio_SetDataDirection(&GPIOInst2, GPIO_2_INPUT_1_CHANNEL, 0xFFFFFFFF);
	XGpio_SetDataDirection(&GPIOInst2, GPIO_2_INPUT_2_CHANNEL, 0xFFFFFFFF);

	status = AXI_Timer_initialize();
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// initialize the interrupt controller
	status = XIntc_Initialize(&IntrptCtlrInst, INTC_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
	   return XST_FAILURE;
	}
	
	// connect the fixed interval timer (FIT) handler to the interrupt
	status = XIntc_Connect(&IntrptCtlrInst, FIT_INTERRUPT_ID,
						   (XInterruptHandler)FIT_Handler,
						   (void *)0);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;

	}

	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts.
	status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// enable the FIT interrupt
	XIntc_Enable(&IntrptCtlrInst, FIT_INTERRUPT_ID);
	return XST_SUCCESS;
}
/*
 * AXI timer initializes it to generate out a 4Khz signal, Which is given to the Nexys4IO module as clock input.
 * DO NOT MODIFY
 */
int AXI_Timer_initialize(void){

	uint32_t status;				// status from Xilinx Lib calls
	u32		ctlsts;		// control/status register or mask

	status = XTmrCtr_Initialize(&AXITimerInst,AXI_TIMER_DEVICE_ID);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	status = XTmrCtr_SelfTest(&AXITimerInst, TmrCtrNumber);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	ctlsts = XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_EXT_GENERATE_MASK | XTC_CSR_LOAD_MASK |XTC_CSR_DOWN_COUNT_MASK ;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber,ctlsts);

	//Set the value that is loaded into the timer counter and cause it to be loaded into the timer counter
	XTmrCtr_SetLoadReg(AXI_TIMER_BASEADDR, TmrCtrNumber, 24998);
	XTmrCtr_LoadTimerCounterReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts &= (~XTC_CSR_LOAD_MASK);
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts |= XTC_CSR_ENABLE_TMR_MASK;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	XTmrCtr_Enable(AXI_TIMER_BASEADDR, TmrCtrNumber);
	return XST_SUCCESS;

}

/*********************** DISPLAY-RELATED FUNCTIONS ***********************************/

/****************************************************************************/
/**
* Converts an integer to ASCII characters
*
* algorithm borrowed from ReactOS system libraries
*
* Converts an integer to ASCII in the specified base.  Assumes string[] is
* long enough to hold the result plus the terminating null
*
* @param 	value is the integer to convert
* @param 	*string is a pointer to a buffer large enough to hold the converted number plus
*  			the terminating null
* @param	radix is the base to use in conversion, 
*
* @return  *NONE*
*
* @note
* No size check is done on the return string size.  Make sure you leave room
* for the full string plus the terminating null in string
*****************************************************************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix)
{
	char tmp[33];
	char *tp = tmp;
	int32_t i;
	uint32_t v;
	int32_t  sign;
	char *sp;

	if (radix > 36 || radix <= 1)
	{
		return;
	}

	sign = ((10 == radix) && (value < 0));
	if (sign)
	{
		v = -value;
	}
	else
	{
		v = (uint32_t) value;
	}
	
  	while (v || tp == tmp)
  	{
		i = v % radix;
		v = v / radix;
		if (i < 10)
		{
			*tp++ = i+'0';
		}
		else
		{
			*tp++ = i + 'a' - 10;
		}
	}
	sp = string;
	
	if (sign)
		*sp++ = '-';

	while (tp > tmp)
		*sp++ = *--tp;
	*sp = 0;
	
  	return;
}


/****************************************************************************/
/**
* Write a 32-bit unsigned hex number to PmodOLEDrgb in Hex
*       
* Writes  32-bit unsigned number to the pmodOLEDrgb display starting at the current
* cursor position.
*
* @param num is the number to display as a hex value
*
* @return  *NONE*
*
* @note
* No size checking is done to make sure the string will fit into a single line,
* or the entire display, for that matter.  Watch your string sizes.
*****************************************************************************/ 
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num)
{
  char  buf[9];
  int32_t   cnt;
  char  *ptr;
  int32_t  digit;
  
  ptr = buf;
  for (cnt = 7; cnt >= 0; cnt--) {
    digit = (num >> (cnt * 4)) & 0xF;
    
    if (digit <= 9)
	{
      *ptr++ = (char) ('0' + digit);
	}
    else
	{
      *ptr++ = (char) ('a' - 10 + digit);
	}
  }

  *ptr = (char) 0;
  OLEDrgb_PutString(InstancePtr,buf);
  
  return;
}


/****************************************************************************/
/**
* Write a 32-bit number in Radix "radix" to LCD display
*
* Writes a 32-bit number to the LCD display starting at the current
* cursor position. "radix" is the base to output the number in.
*
* @param num is the number to display
*
* @param radix is the radix to display number in
*
* @return *NONE*
*
* @note
* No size checking is done to make sure the string will fit into a single line,
* or the entire display, for that matter.  Watch your string sizes.
*****************************************************************************/ 
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix)
{
  char  buf[16];
  
  PMDIO_itoa(num, buf, radix);
  OLEDrgb_PutString(InstancePtr,buf);
  
  return;
}


/**************************** INTERRUPT HANDLERS ******************************/

/****************************************************************************/
/**
* Fixed interval timer interrupt handler
*
* Reads the GPIO port which reads back the hardware generated PWM wave for the RGB Leds
*
* @note
* ECE 544 students - When you implement your software solution for pulse width detection in
* Project 1 this could be a reasonable place to do that processing.
 *****************************************************************************/

void FIT_Handler(void)
{
	// Read the GPIO port to read back the generated PWM signal for RGB led's
	gpio_in = XGpio_DiscreteRead(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL);

}

