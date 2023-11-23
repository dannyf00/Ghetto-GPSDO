//Ghetto GPSDO
//
//  target: dsPIC33
//
//   discussed here: https://dannyelectronics.wordpress.com/2023/11/17/ghetto-gpsdo-approach/
//
// - version history
// - v0.1, 11/17/2023: initial implementation
// - v0.2, 11/18/2023: added a ppsLock indicator, oversampling support and reference frequency output
// - v0.3, 11/19/2023: supports very long oversampling period (practically unlimited)
// - v0.4, 11/21/2023: support up to 999,999,999.999999 hz calculation and string formation
// - v0.5, 11/21/2023: updated documentation
// - v0.6, 11/21/2023: ppsLock shows lock only if the frequency falls below a pre-specified threshold for a user-specified period of time
// - v0.6a,11/21/2023: implemented adaptive PID/variable oversampling
//
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
//              |                 RB0 |>---[1k]--->Uart2TX
//              |                     |
//              |                 RB7 |<---[1k]---<PPS_IN (from a frequency reference)
//              |                     |
//              |                 RB6 |>---[1k]--->PPS_OUT (PWM4, optional)
//              |                     |
//              |                 RB5 |>---------->LED/PWM_OUT (PWM2) --->[100k]---+---[100k]----+---[C1:22pf]---+--->OSC_IN/[OSC_OUT]
//              |                     |                                            |             |               |
//              |                 RB8 |>---[1k]--->ppsLock status                 ---          BB910            ---
//              |                     |                                          [4.7u]         / \            [15p]
//              |                 RB13|>---[50R]-->REFCLK0                        ---          -----            ---
//              |                     |                                            |             |               |
//              |                 RBx |>---[50R]-->REFCLK1                         |             |               |
//              |                     |                                          [GND]         [GND]           [GND]
//              |                 RBy |>---[50R]-->REFCLK2
//              |                     |
//              |=====================|
//
//
//documentation
//  the code contains reasonably detailed documentation itself.
//  the following is provided to augment that.
//
//key features:
//  1) simplicity: the Ghetto GPSDO uses minimal amount of external parts to discipline a local VCXO to lock onto a reference pps signal
//  2) adaptive PID / oversampling: it changes its PID parameters and oversampling periods to achieve fast lock and high resolution.
//  3) high frequency resolution: the instrument can measure frequency up to 999,999,999.999999hz (subject to hardware limitation), with 1uHz resolution
//  4) output its own PPS signal: the device generates a pps signal and up to 3 reference clocks (subject to hardware limitation)
//  5) small code base: as a minimalist GPSDO, the code base is small and once compiled, around 7KB so it should fix into much smaller chips
//  6) portability: coded entirely in C, this code should be quite portable to other chips, like dsPIC/PIC24/PIC32, or Cortex-M chips.
//  7) user configurability: many macros are provided for user to reconfigure the GPSDO for his or her own environment
//
//key macros:
//  PPS_INN: 		the pulse rate of the incoming reference pps. 1 for 1PPS, 2 for 2PPS, .... Should not be too frequency (>1000 would be too much)
//  PPB_LOCK:		the threshold to determine if the VCXO has locked to the pps input signal, in ppb
//  PPS_LOCKN:		the number of times frequency error falls below PPB_LOCK before a lock is deemed
//  VTRLSLOPE_P: 	specify if the VCXO has a positive slope or not. Positive slope means the output frequency goes up with the control voltage
//  PPS_OUT: 		defines if the device will output a PPS signal. Output pps rate defined by PPS_OUTN: 1=1pps, 2=2pps, ... implemented in hardware
//  REFCLKn_N: 		reference clock output, n=0, 1 and 2. Not always available.
//  PWM22RP():		specify pwm control/PWM2 output pin, in dsPIC33duino.h
//  PWM42RP():		specify pps output /PWM4 output pin, in dsPIC33duino.h
//  cPID_gains:		coarse PID parameters by the PID algorithm before the VCXO is locked to the pps reference signal: shorter oversampling period, higher gain
//  fPID_gains:		fine   PID parameters by the PID algorithm after  the VCXO is locked to the pps reference signal: longer oversampling period, smaller gain
//  F_XTAL:			nominal frequency of the local VCXO, specified in dsPIC33duino.h
//
//key variables:
//  tksErr:			frequency error over the sampling period defined by PPS_OVN
//  ppsFlg:			pps flag: 1 if new frequency measurement is available. 0 otherwise
//  freq:			frequency measurement: a struct with Hz, uHz and string representation of the VCXO frequency
//  PPS_OVN:		pps oversampling period, in seconds. ppsISR() switches this variable between fPPS_OVN and cPPS_OVN
//  ppsLocked:		1->if VCXO is locked, and 0->if VCXO is not. User can use this variable to check lock status
//
//key routines:
//	ppsISR():		called when pps rising edge has arrived. calculates tksErr and sets the flag
//  errPID():		PID control algorithm that returns the control needed to correct the loop
//  tksErr2Ctrl():	turn tksErr into a control variable by implementing a negative feedback loop
//  Ctrl2DC16():	using the control to drive a 16-bit PWM generator to produce the control voltage on our VCXO
//  ppsLock():		return 1 if the frequency error falls below that specified by PPB_LOCK for consecutive times. called through ppsISR() only
//  refClknOut():	produce reference clock output, n=0/1/2. Hardware dependent and may not be available on all targets
//  ppsInit():		reset the pps module to its initial state: TIM2 as time base for PWM2 (pwm generation), TIM3 as time base for PWM4 (optional pps generation)
//  tks2freq():		calculate Hz and uHz from tksErr
//  freq2uHz():		form a string of frequency measurements in uHz, 21-char long
//  freq2mHz():		form a string of frequency measurements in mHz, 18-char long

#include "dspic33duino.h"				//we use dsPIC33duino

//hardware configuration
//#define LED			PB5					//led pin, comment out if not used
#define LED_DLY		(F_CPU / 2)			//half a second

//pps lock indicator, active low
#define LED_LOCK	PB8					//0->pps not locked, 1->pps locked (frequency error less than specified by PPB_LOCK, in ppb
#define PPB_LOCK	500					//ppb error to determine if pps is locked: lock deemed if frequency error falls within the PPB_LOCK
#define PPS_LOCKN	4					//number of consecutive times that frequency error must fall below PPB_LOCK before a lock is deemed

//Pulse input: the software will automatically determine the likely BPH rate
#define PPS_IN		PB7					//pps pulse train input on EINT0 on pb7
#define PPS_INN		2					//pps pulses per second on EINT0 on pb7
#define TKS_MIN		5000				//minimum number of ticks to be considered a value input pulse
#define VTRLSLOPE_P						//if Vcontrl slope is positive (higher output voltage -> higher frequency). comment out if negative

//#define USE_PID						//deprecated: switch between errPID() or errSum()
//PID settings for coarse PID - prior to the OCXO has locked to the pps
//generally, shorter oversampling window (cPPS_OVN < fPPS_OVN) and higher gains
#define cPPS_OVN	2					//oversample. gate time. practically unlimited
#define cPID_Ix		1					//PID gain for I_term, normalized by PID_x: normalized to 1
#define cPID_Px		128					//pid gain for P_term, normalized by PID_x: 64 .. 512
#define cPID_Dx		0					//pid gain for D_term, normalized by PID_x: not used
#define cPID_x		1					//normalizer for PID_gains: may use it if the gain / timer period is too big (PPS_OVN >= 256)
//PID settings for fine PID - after the OCXO has locked to the pps
//generally, longer oversampling window (fPPS_OVN > cPPS_OVN) and lower gains
//good practice to answer fPID parameters to cPID parameters
#define fPPS_OVN	10					//oversample. gate time. practically unlimited
#define fPID_Ix		(cPID_Ix)			//PID gain for I_term, normalized by PID_x: normalized to 1
#define fPID_Px		(cPID_Px / 4)		//pid gain for P_term, normalized by PID_x: 64 .. 512
#define fPID_Dx		(cPID_Dx)			//pid gain for D_term, normalized by PID_x: not used
#define fPID_x		(cPID_x)			//normalizer for PID_gains: may use it if the gain / timer period is too big (PPS_OVN >= 256)

#define PPSDO_DEBUG						//use if we are debugging - enabling print out from errPID() and errSUM()
#define TKS_OFST	(-100-8*2)			//tks offset - for debugging

//PWM output for timing errors - must have for the gpsdo to function
#define PWM_OUT							//pwm2/pb5 generator output pin. comment out if not needed

//PPS output - optional
//generating Beats Per Hour pulses - to simulate a GPS 1pps output
#define PPS_OUT							//pps pulse output on pwm4/PB6. comment out if not needed
#define PPS_OUTN	1					//number of pulses per second. 1=1pps output, 2=2pps output

//reference frequency output - comment out if not needed
//user implemented -> may not be available on all chips
#define REFCLK0_N	5					//divider for refclk0 - using REFCO (PIC24/PIC32) or MCO (STM32/PY32)
#define REFCLK1_N	100					//divider for refclk1 - using timer / pwm (50% dc)
#define REFCLK2_N	200					//divider for refclk2 - using timer / pwm (50% dc)

//end hardware configuration

//global defines
//data type to hold frequency measurements in two pieces:
//1. Hz: measured in hz
//2. uHz: measured in uHz
//str[] is the c-string converted freq.uHz
typedef struct {
	uint32_t Hz;						//frequency, in hz
	uint32_t uHz;						//frequency, in uHz (0.000001Hz)
	char str[40];						//string representation: " xxx,xxx,xxx.yyyyyy"
} FREQ_TypeDef;

//global variables
//variables related to GPSDO
volatile char ppsFlg=0;					//0->no new data, 1->new data
volatile int32_t tksErr=0;				//frequency error over PPS_OVN seconds
FREQ_TypeDef freq;						//frequency
uint32_t PPS_OVN=cPPS_OVN;				//oversampling period, in sec
 int32_t PID_Ix=cPID_Ix, PID_Px=cPID_Px, PID_Dx=cPID_Dx, PID_x = cPID_x;
volatile char ppsLocked=0;				//0->VCXO not locked, 1->VCXO locked

//other variables
static uint32_t tick0=0;				//for loop control

//functional prototypes
char ppsLock(void);

//pps isr - now support oversampling
void ppsISR(void) {
	static int ovn=0;					//oversampling counter
	static int inn=0;					//pps counter
	static uint32_t tks0=0;				//ticks for the previous rising edge
	uint32_t tks=ticks();				//stake a snapshot
	static int32_t tmp=0;
	uint32_t ppsTks;					//pps ticks in between two pulses

	//optional - anti-glitch
	if (tks - tks0 > TKS_MIN) {			//only pulse elapses longer than TKS_MIN are considered valid
		if (++inn == PPS_INN) {			//make sure that oversampling is done
			ppsTks=tks - tks0; 			//ppsTks = ticks expired
			tks0=tks;					//cupdate tks0
#if defined(PPSDO_DEBUG)
			tmp += ppsTks - (F_CPU + TKS_OFST);	//calculate measured tiks vs. expected ticks, with offset (for debugging)
#else
			tmp += ppsTks - (F_CPU           );	//calculate measured tiks vs. expected ticks
#endif	//tks_ofst
			inn = 0;					//reset inn counter
			if (++ovn == PPS_OVN) {		//enough oversampling?
				tksErr = tmp;			//save tmp to tksErr
				ppsFlg = 1;				//1->new data has arrived
				tmp = 0;				//reset tmp
				ovn = 0;				//reset oversample counter
				//tksErr = tks2Err();	//calculate frequency errors
				ppsLocked=ppsLock();	//determine lock status
			}
		}
	}
}

//pid timing error
//variable gain / oversampling window specified via macros
int32_t errPID(void) {
	static int32_t I_term=0;						//sum of all errors
	static int32_t P_term=0;						//current error
	static int32_t D_term=0;						//calculate the delta
	static int32_t errPrev=0;						//previous error term
	//int32_t pid=0;
	//static char initialized=0;					//0->first time running the code, not yet initialized

	//calculate the various terms
	I_term += tksErr;
	P_term  = tksErr;
	D_term  = tksErr - errPrev; errPrev = tksErr;

	//update gains: use fine pid parameters if vcxo is locked to pps input.
	//otherwise, use coarse pid parameters for faster convergence
	if (ppsLocked) {PID_Ix = fPID_Ix; PID_Px = fPID_Px; PID_Dx = fPID_Dx; PID_x = fPID_x; PPS_OVN = fPPS_OVN;}
	else {PID_Ix = cPID_Ix; PID_Px = cPID_Px; PID_Dx = cPID_Dx; PID_x = cPID_x; PPS_OVN = cPPS_OVN;}

#if defined(PPSDO_DEBUG)
	//for debugging only - comment out in actual runs
	//for fine tunning the pid
	u2Print("I_trm=                    ", I_term);
	//u2Print("P_trm=                    ", P_term);
	//u2Print("D_trm=                    ", D_term);
#endif	//ppsdo_debug
	return (I_term * PID_Ix + P_term * PID_Px + D_term * PID_Dx) / PID_x;						//fixed gain

}

//generating control signal
//accommodates both positive or negative slopes
//USE_PID: 16-bit PWM generator, return PWM duty cycle changes
//otherwise: use exponential smoothing
int32_t tksErr2Ctrl(void) {
	int32_t tmp=0;

	tmp = errPID();							//use PID
	//tmp = errSum();							//use simple summation

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
	//update pwm generator only if ctrl <> 0
	if (ctrl) pwm2SetDC(constrain(pwm2GetDC() + ctrl, 1, 0xffff));			//output timing errors on the pwm2 generator
	return pwm2GetDC();								//return pwm duty cycle
}

//lock status - to be called from ppsISR() only
//return 1/locked if frequency within range specified by PPB_LOCK for PPS_LOCKN times consecutively
//return 0/unlocked if the frequency error goes above PPB_LOCK on the first time
char ppsLock(void) {
	static uint32_t f_cpu = 0;
	static uint8_t cnt=0;
	char tmp;

	if (f_cpu==0) f_cpu = F_CPU;
	if (abs(tksErr) > f_cpu / 1000000ul * PPB_LOCK * PPS_OVN / 1000) tmp = 0; else tmp = 1;
	//only return 1 if we have lock for PPS_LOCKN times consecutively
	if (tmp == 0) cnt=0; else cnt+=(cnt==PPS_LOCKN)?0:1;
	//u2Print("cnt   =              ", cnt);
	return ppsLocked=(cnt==PPS_LOCKN);
}

//deprecated
//return measured frequency
//measure frequency in 0.01 resolution
//i.e., xyz reading = x.yz Hz
uint32_t Frqx100(void) {
	//return F_CPU * 100 + tks2Err() * 100/PPS_OVN;
	return F_CPU * 100 + tksErr * 100/PPS_OVN;
}

//deprecated - for debugging
//print frequency
void u2PrintFrq(uint32_t freq, int32_t frac) {
	u2Print("IkHz  =               ", (freq / 1000));
	u2Print("FmHz  =               ", (freq % 1000) * 1000 + frac * 1000 / 8);
}

//output the reference clock - not supported on dsPIC33
//hardware dependent.
//divider specified by REFCLK0_N
//using REFCO (PIC24/PIC32) or MCO (STM32/PY32)
void refClk0Out(void) {
	//general workflow
#if 0	//for pic24 - not available on dsPIC33
	REFOCONbits.ROEN = 0;				//1->enable refclk0 output, 0->disable refclk0 output
	//set up divider(s)
	REFOCONbits.RODIV= REFCLK0_N;		//divider = 2^N, N=0..15
	REFOCONbits.ROSEL = 0;				//0->sysclk, 1->Posc
	//enable the output pin
	REFOCONbits.ROEN = 1;				//1->enable refclk0 output, 0->disable refclk0 output
#endif	//0
}

//divider specified by REFCLK1_N
//using timer / PWM (50% dc)
void refClk1Out(void) {
	//general workflow
	//set up divider(s)
	//enable the output pin
}

//divider specified by REFCLK2_N
//using timer / PWM (50% dc)
void refClk2Out(void) {
	//general workflow
	//set up divider(s)
	//enable the output pin
}

//initialize pps
//PWM output pwm2 on RP5: timebase tmr2, 1:1 prescaler
//PPS output pwm4 on RP6: timebase tmr3, variable prescaler (256:1, 64:1, 8:1 or 1:1), based on F_CPU
void ppsInit(void) {
	char ps = TMR_PS256x;				//default prescaler

	ppsFlg=0;							//initialize ppsFlg to 0->no new data
	PPS_OVN = cPPS_OVN;					//start with coarse PID

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
	//tmr3 as pwm4 time base, 1% DC
	//determining prescaler: 256:1, 64:1 and 8:1 or 1:1
	if (F_CPU / 64 / (PPS_OUTN) > 65535) ps = TMR_PS256x; else {if (F_CPU / 8  / (PPS_OUTN) > 65535) ps = TMR_PS64x; else {if (F_CPU / 1  / (PPS_OUTN) > 65535) ps = TMR_PS8x; else ps = TMR_PS1x;}}
	tmr3Init(ps); TMR3 = -1;
	//set up pwm4, using tmr3 as time base
	pwm4Init(); OC4CONbits.OCTSEL = 1; pwm4SetPR(F_CPU / tmr3GetPS() / (PPS_OUTN)); pwm4SetDC(pwm4GetPR() / 100);
	//manually select precaler
	//{tmr3Init(TMR_PS64x); pwm3Init(); OC3CONbits.OCTSEL = 1; pwm3SetPR(F_CPU / 64 / (BPH_OUT / 3600)); pwm3SetDC(pwm3GetPR() / 100);}
#endif

#if defined(LED_LOCK)
	digitalWrite(LED_LOCK, LOW); pinMode(LED_LOCK, OUTPUT);	//led_lock as output, active low
#endif	//LED_LOCK

	//output reference signals - REFCLK0/1/2
#if defined(REFCLK0_N)
	refClk0Out();
#endif	//refclk0_out

#if defined(REFCLK1_N)
	refClk1Out();
#endif	//refclk1_out

#if defined(REFCLK2_N)
	refClk2Out();
#endif	//refclk2_out
}

//frequency conversion, from tksErr -> freq
FREQ_TypeDef *tks2freq(FREQ_TypeDef *frq) {
	int32_t tmp;									//temp variable

	//two ways to calculate freq
#if 0
	//this approach iterates
	frq->Hz = F_CPU;
	tmp = tksErr * 1000000l / PPS_OVN;				//calculate the temp variable
	do {
		//unlikely our error will be greater than 100Hz / PPS_OVN
		//so we will get the iterate only once
		frq->Hz -= 100;								//decrement freq, by 100
		tmp       += 100 * 1000000l;				//increment tmp, by 100
	} while (tmp < 0);
	frq->Hz   += tmp / 1000000l;
	frq->uHz = tmp % 1000000l;
#else
	//no iteration
	frq->Hz = F_CPU;								//Hz stores the nominal value
	//tmp = 0;
	if (tksErr < 0) {
		tmp = -tksErr / PPS_OVN; if (tksErr != -tmp * PPS_OVN) tmp += 1;	//find the number of Hz needed to borrower from frq->HZ
		frq->Hz -= tmp;								//borrow from frq->Hz
		frq->uHz = tmp * 1000000l + (tksErr % PPS_OVN) * 1000000l / PPS_OVN;	//update the frq->uHz
	} else {
		tmp =  tksErr / PPS_OVN;					//find the number of Hz to be given to frq->Hz: if tksErr > PPS_OVN
		frq->Hz += tmp;								//add to frq->Hz
		frq->uHz =                  (tksErr % PPS_OVN) * 1000000l / PPS_OVN;	//calculate the remaining uHz
	}
#endif	//0
	return frq;
}

//frequency conversion, from freq to string, in uHz
//supports up to 999,999,999.999999Hz display
char *freq2uHz(FREQ_TypeDef *frq) {
	unsigned char idx=21;							//must match the number of chars down below
	uint32_t tmp;

	frq->str[idx--]= 0;								//null terminated c-string
	frq->str[idx--] = 'z';
	frq->str[idx--] = 'H';
	//frq->str[idx--] = 'u';
	tmp = frq->uHz;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]='.';
	tmp = frq->Hz;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=',';
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=',';
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--] = ' ';
	return frq->str;
}

//frequency conversion, from freq to string, in mHz
//supports up to 999,999,999.999999Hz display
char *freq2mHz(FREQ_TypeDef *frq) {
	unsigned char idx=21-3;							//must match the number of chars down below
	uint32_t tmp;

	frq->str[idx--]= 0;								//null terminated c-string
	frq->str[idx--] = 'z';
	frq->str[idx--] = 'H';
	//frq->str[idx--] = 'm';
	tmp = frq->uHz / 1000;
	//frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	//frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	//frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]='.';
	tmp = frq->Hz;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=',';
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=',';
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--]=(tmp % 10) + '0'; tmp /= 10;
	frq->str[idx--] = ' ';
	return frq->str;
}

//flip led
void led_flp(void) {
#if defined(LED)
	pinFlip(LED);									//flip led
#endif	//led
}

//user defined set up code
void setup(void) {
	//switch clock - not all clock sources shown (LPRC)
	//SystemCoreClockFRC();							//switch to FRC
	//SystemCoreClockFRCPLL0x4();					//switch to FRCxPLL
	SystemCoreClockPOSC();							//switch to primary oscillator
	//SystemCoreClockPOSCPLL();						//run pll (@4x default)
	//SystemCoreClockPOSCPLL2x();					//switch to primary oscillator x PLL
	//FRCSetTrim(0);								//trim FRC

#if defined(LED)
	digitalWrite(LED, HIGH); pinMode(LED, OUTPUT);							//led as output pin
#endif	//led

	//initialize the uart
	//uart1Init(UART_BR9600);						//initialize uart1
	uart2Init(UART_BR38K4);							//initialize uart2

	//initialize the pps module
	ppsInit(); while (ppsFlg==0); ppsFlg=0;			//skip over the first measurement

	tick0=ticks();									//time stamp tick0

	ei();											//enable interrupts
}

//user defined main loop
void loop(void) {
	int32_t tmp0, tmp;
	static uint32_t cnt=0;
	int i;

	//if (tick0--==0) { tick0=0x10000ul;			//increment a counter
	//if (ticks() - tick0 > LED_DLY) { tick0 += LED_DLY;	//if enough time has passed
	if (ppsFlg) { ppsFlg = 0;						//if new data has arrived
		//pinFlip(LED);								//flip the led

		//measure timing
		tmp0=ticks();
		//put some here to benchmark
		tmp0=ticks() - tmp0;

		//measure timing error
		//update the pwm generator
		tmp = tksErr2Ctrl();						//calculate the control needed
		Ctrl2DC16(tmp);								//output timing errors on the pwm2 generator

#if defined(LED_LOCK)
		if (ppsLocked) digitalWrite(LED_LOCK, HIGH); else digitalWrite(LED_LOCK, LOW);
#endif	//led_lock

		//display something
		u2Print("cnt   =                  ", cnt++);							//display counter
		//u2Print("F_CPU =                  ", F_CPU);							//display cpu frequency
		//u2Print("ticks =                  ", ticks());						//display cpu frequency
		//u2Print("tmp0  =                  ", tmp0);
		//u2Print("ppsTks=                  ", ppsTks);
		//u2Print("tksErr=                  ", tksErr);
		//u2PrintFrq(100000, -2);
		tks2freq(&freq); uart2Puts("Freq  ="); uart2Puts(freq2uHz(&freq)); uart2Puts("   "); //display the frequency in string
		//u2Print("Frq100=                  ", Frqx100());
		//u2Print("ppsLck=                  ", ppsLock());
		uart2Puts(ppsLocked?"pps     locked...   ":"pps not locked...   ");
		//u2Print("DC_Chg=                  ", (tmp));								//display timing error, in ms per day
		u2Print("PWMDC%=                  ", pwm2GetDC() * 10000ul/pwm2GetPR()*1);	//display timing error, off 50.0%, 1% = 1 sec.
		//u2Print("PWMDC#=                  ", pwm2GetDC());						//display timing error, off 50.0%, 1% = 1 sec.
		//u2Print("u2bps =                  ", u2bps());
		u2Println();
	}
}

