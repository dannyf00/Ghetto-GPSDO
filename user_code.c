//dsPIC33duino Ghetto GPSDO
//
//   discussed here: https://dannyelectronics.wordpress.com/2023/11/17/ghetto-gpsdo-approach/
//
// - version history
// - v0.1, 11/17/2023: initial implementation
//
//
//               dsPIC33FJ
//              |=====================|
//    Vcc       |                     |
//     |        |                Vcap |>--[.1u]-+->GND
//     |        |                     |         |
//     +-[10K]-<| MCLR        DISVreg |>--------+
//              |                     |
//              |                     |
//     +------->| OSC_IN          Vdd |>--+------->Vcc
//  [Xtal]      |                     | [.1u]
//     +-------<| OSC_OUT         Vss |>--+------->GND
//              |                     |
//              |                 RB0 |>---------->Uart2TX
//              |                     |
//              |                 RB7 |<---------->PPS_IN (from a frequency reference)
//              |                     |
//              |                 RB6 |>---------->PPS_OUT (PWM3, optional)
//              |                     |
//              |                 RB5 |>---------->LED/PWM_OUT (PWM2) --->[100k]---+---[100k]----+---[C1:22pf]---+--->OSC_IN/[OSC_OUT]
//              |                     |                                            |             |               |
//              |                     |                                           ---          BB910            ---
//              |                     |                                          [4.7u]         / \            [15p]
//              |                     |                                           ---          -----            ---
//              |                     |                                            |             |               |
//              |                     |                                            |             |               |
//              |                     |                                          [GND]         [GND]           [GND]
//              |                     |
//              |                     |
//              |=====================|
//
//

#include "dspic33duino.h"				//we use dsPIC33duino

//hardware configuration
#define LED			PB5					//led pin
#define LED_DLY		(F_CPU / 2)			//half a second

//Pulse input: the software will automatically determine the likely BPH rate
#define PPS_IN		PB7					//pps pulse train input on EINT0 on pb7
#define PPS_INN		2					//pps pulses per second on EINT0 on pb7
#define TKS_MIN		5000				//minimum number of ticks to be considered a value input pulse
#define VTRLSLOPE_P						//if Vcontrl slope is positive (higher output voltage -> higher frequency). comment out if negative

//two approaches for closed loop control
//option 1: use the pid - generally performs better
#define USE_PID							//switch between errPID() or errSum()
//PID settings
#define PID_Ix		1					//PID gain for I_term, normalized by PID_x: normalized to 1
#define PID_Px		128					//pid gain for P_term, normalized by PID_x: 64 .. 512
#define PID_Dx		0					//pid gain for D_term, normalized by PID_x: not used
#define PID_x		1					//normalizer for PID_gains
//option 2: use a low-pass filter
//expSmoothing settings
#define EXP_CNT		8					//number of measurement windows: 16 .. 64
#define EXP_GAIN	2					//gain when exponential smoothing is used. 1..4

#define PPSDO_DEBUG						//use if we are debugging - enabling print out from errPID() and errSUM()
#define TKS_OFST	(-810+6)			//tks offset - for debugging

//PWM output for timing errors - optional
#define PWM_OUT							//pwm2/pb5 generator output pin. comment out if not needed

//PPS output - optional
//generating Beats Per Hour pulses - to simulate a GPS 1pps output
#define PPS_OUT							//pps pulse output on pwm3/PB6. comment out if not needed
#define PPS_OUTN		1				//number of pulses per second
//end hardware configuration

//global defines

//global variables
//variables related to GPSDO
volatile uint32_t ppsTks=0;				//pps ticks in between two pulses
volatile char ppsFlg=0;					//0->no new data, 1->new data

//other variables
static uint32_t tick0=0;				//for loop control

//pps isr
void ppsISR(void) {
	static uint32_t tks0=0;				//ticks for the previous rising edge
	uint32_t tks=ticks();				//stake a snapshot

	//optional - anti-glitch
	if (tks - tks0 > TKS_MIN) {			//only pulse elapses longer than TKS_MIN are considered valid
		ppsTks=tks - tks0; 				//ppsTks = ticks expired
		tks0=tks;						//cupdate tks0
		ppsFlg=1;						//1->new data has arrived
	}
}

//initialize pps
//PWM output pwm2 on RP5: timebase tmr2, 1:1 prescaler
//PPS output pwm3 on RP6: timebase tmr3, variable prescaler (256:1, 64:1, 8:1 or 1:1), based on F_CPU
void ppsInit(void) {
	char ps = TMR_PS256x;				//default prescaler

	ppsFlg=0;							//initialize ppsFlg to 0->no new data

	pinMode(PPS_IN, INPUT_PULLUP);		//input, pull up

	//enable pps input on eint0/pb7
	int0Init();							//active on rising edge
	int0AttachISR(ppsISR);				//install user isr

	//configure 16-bit PWM output for timing errors
#if defined(PWM_OUT)
	//tmr2 as pwm2 time base, 1:1 prescaler
#if defined(SYSTICK_TMR1)
	tmr2Init(TMR_PS1x);					//reset tmr2 only if it hasn't be used as timebase.
#endif	//use tmr2
	//pwm2 output on RP5/PB5, 16-bit, duty cycle 50%, active high
	//tmr2 as timebase -> 16 bit pwm
	pwm2Init(); pwm2SetDC(pwm2GetPR() / 2);
#endif

	//configure pps output
#if defined(PPS_OUT)
	//tmr3 as pwm3 time base, 1% DC
	//determining prescaler: 256:1, 64:1 and 8:1 or 1:1
	if (F_CPU / 64 / (PPS_OUTN) > 65535) ps = TMR_PS256x; else {if (F_CPU / 8  / (PPS_OUTN) > 65535) ps = TMR_PS64x; else {if (F_CPU / 1  / (PPS_OUTN) > 65535) ps = TMR_PS8x; else ps = TMR_PS1x;}}
	tmr3Init(ps); TMR3 = -1; pwm3Init(); OC3CONbits.OCTSEL = 1; pwm3SetPR(F_CPU / tmr3GetPS() / (PPS_OUTN)); pwm3SetDC(pwm3GetPR() / 100);
	//manually select precaler
	//{tmr3Init(TMR_PS64x); pwm3Init(); OC3CONbits.OCTSEL = 1; pwm3SetPR(F_CPU / 64 / (BPH_OUT / 3600)); pwm3SetDC(pwm3GetPR() / 100);}
#endif

}

//flip led
void led_flp(void) {
	pinFlip(LED);									//flip led
}

//test if a number is within a bound
//return true if low < x < high
char inRange(int32_t x, int32_t low, int32_t high) {
	return ((low < x) && (x < high))?1:0;
}

//pid timing error
//gain specified via macros
int32_t errPID(int32_t tkserr) {
	static int32_t I_term=0;						//sum of all errors
	static int32_t P_term=0;						//current error
	static int32_t D_term=0;						//calculate the delta
	static int32_t errPrev=0;						//previous error term
	//int32_t pid=0;
	//static char initialized=0;					//0->first time running the code, not yet initialized

	//calculate the vairous terms
	I_term += tkserr;
	P_term  = tkserr;
	D_term  = tkserr - errPrev; errPrev = tkserr;

#if defined(PPSDO_DEBUG)
	//for debugging only - comment out in actual runs
	//for fine tunning the pid
	u2Print("I_trm=                ", I_term);
	//u2Print("P_trm=                ", P_term);
	//u2Print("D_trm=                ", D_term);
#endif	//ppsdo_debug
	return (I_term * PID_Ix + P_term * PID_Px + D_term * PID_Dx) / PID_x;						//fixed gain

}

//sum of err - exponential smothing
//EXP_CNT determines window width
int32_t errSum(int32_t tkserr) {
	static int32_t sum=0;							//keeps the sum of tick errors
	static char initiated=0;						//1 if sum has been initialized

	if (initiated==0) {initiated=1; sum = tkserr * EXP_CNT;}	//calculate the average
#if defined(PPSDO_DEBUG)
	//for debugging only - comment out in actual runs
	//for fine tunning the LPF
	u2Print("errSum=                ", sum);
#endif	//ppsdo_debug
	sum += tkserr - sum / EXP_CNT;			//return the sum
	return sum * EXP_GAIN;
}

//returns error (in tks) per second
//return + if target oscillator is too fast
int32_t tks2Err(void) {
#if defined(PPSDO_DEBUG)
	return (ppsTks * PPS_INN - F_CPU) - TKS_OFST;	//calculate measured tiks vs. expected ticks, with offset (for debugging)
#else
	return (ppsTks * PPS_INN - F_CPU);				//calculate measured tiks vs. expected ticks
#endif	//tks_ofst
}

//generating control signal
//accommodates both positive or negative slopes
//USE_PID: 16-bit PWM generator, return PWM duty cycle changes
//otherwise: use exponential smoothing
int32_t tksErr2Ctrl(int32_t tksErr) {
	int32_t tmp=0;

#if defined(USE_PID)
	tmp = errPID(tksErr);							//use PID
#else
	tmp = errSum(tksErr);							//use simple summation
#endif	//use_pid

	//establish a negative feedback loop
#if defined(VTRLSLOPE_P)
	//for positive slopes, positive errors should lead to lower output voltage / lower PWM duty cycle
	return -tmp;									//positive slope: higher control voltage -> higher frequency. positive error -> lower control voltage -> less positive error
#else
	//for negative slopes, positive errors should lead to higher output voltage / higher PWM duty cycle
	return +tmp;									//negative slope: higher control voltage -> lower frequency. positive error -> higher control voltage -> less positive error
#endif	//VTRLSLOPE_P
}

//output control signal
//option1: using pwm
uint16_t Ctrl2DC16(int32_t ctrl) {
	pwm2SetDC(constrain(pwm2GetDC() + ctrl, 1, 0xffff));			//output timing errors on the pwm2 generator
	return pwm2GetDC();
}

//user defined set up code
void setup(void) {
	//switch clock - not all clock sources shown (LPRC)
	//SystemCoreClockFRC();							//switch to FRC
	//SystemCoreClockFRCPLL0x4();						//switch to FRCxPLL
	SystemCoreClockPOSC();							//switch to primary oscillator
	//SystemCoreClockPOSCPLL();						//run pll (@4x default)
	//SystemCoreClockPOSCPLL2x();					//switch to primary oscillator x PLL
	//FRCSetTrim(0);								//trim FRC

	digitalWrite(LED, HIGH); pinMode(LED, OUTPUT);							//led as output pin

	//initialize the uart
	//uart1Init(UART_BR9600);						//initialize uart1
	uart2Init(UART_BR38K4);							//initialize uart2

	//initialize the pps module
	ppsInit(); while (ppsFlg==0); ppsFlg=0;			//skip over the first measurement

	tick0=ticks();									//time stamp tick0

	//tmr45Init(2);									//configure 32-bit timer

	ei();											//enable interrupts
}

//user defined main loop
void loop(void) {
	int32_t tmp0, tmp;
	static uint32_t cnt=0;
	int i;

	//if (tick0--==0) { tick0=0x10000ul;				//increment a counter
	//if (ticks() - tick0 > LED_DLY) { tick0 += LED_DLY;	//if enough time has passed
	if (ppsFlg) { ppsFlg = 0;						//if new data has arrived
		//pinFlip(LED);								//flip the led

		//measure timing
		tmp0=ticks();
		//put some here to benchmark
		tmp0=ticks() - tmp0;

		//measure timing error
		tmp = tks2Err();
		//update the pwm generator
#if 1	//1->closed loop, 0->open loop: generate two frequencies
		tmp = tksErr2Ctrl(tmp);
		Ctrl2DC16(tmp);								//output timing errors on the pwm2 generator
#else	//0->open loop: toggle two pwm dc
		pwm2SetDC((cnt++&(1<<4))?1:-1);				//output timing errors on the pwm2 generator
#endif	//toggle vs pdf

		//display something
		//u2Print("F_CPU =                  ", F_CPU);							//display cpu frequency
		//u2Print("ticks =                  ", ticks());						//display cpu frequency
		//u2Print("tmp0  =                  ", tmp0);
		//u2Print("ppsTks=                  ", tks2Err());
		u2Print("tksErr=                  ", tks2Err());
		u2Print("DC_Chg=                  ", (tmp));								//display timing error, in ms per day
		u2Print("PWMDC%=                  ", pwm2GetDC() * 10000ul/pwm2GetPR()*1);	//display timing error, off 50.0%, 1% = 1 sec.
		//u2Print("PWMDC#=                  ", pwm2GetDC());						//display timing error, off 50.0%, 1% = 1 sec.
		//u2Print("u2bps =                  ", u2bps());
		u2Println();
	}
}

