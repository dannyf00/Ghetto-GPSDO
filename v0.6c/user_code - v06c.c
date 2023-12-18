//Ghetto GPSDO: dsPIC33
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
// - v0.6b,11/24/2023: implemented REFCLK1 output and minor bug fixes
// - v0.6c,12/17/2023: provided guidance for PID tuning.
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
//  PPB_LOCK:		the threshold to determine if the VCXO has locked to the pps input signal, in ppb. Use very small PPB_LOCK value to disable variable oversampling
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
//****important****: cPPS_OVN and fPPS_OVN must fully divide 1000000ul -> they need to be factors of 2 and 5
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
//
//PID tunning:
//  1. set all parameters to zero.
//  2. set PID_Px to a point where it starts to oscillate; set PID_Px to 1/2 of the gain;
//  3. set PID_Ix to a value that converges the control in a desired period of time - just be it overshoots;
//  4. set PID_Dx to a value that smooths out the overshoot

#include "dspic33duino.h"				//we use dsPIC33duino

//hardware configuration
//#define LED			PB5					//led pin, comment out if not used
#define LED_DLY		(F_CPU / 2)			//half a second

//pps lock indicator, active low
//#define LED_LOCK	PB8					//0->pps not locked, 1->pps locked (frequency error less than specified by PPB_LOCK, in ppb. comment out if not used
#define PPB_LOCK	100					//ppb error to determine if pps is locked: lock deemed if frequency error falls within the PPB_LOCK
#define PPS_LOCKN	4					//number of consecutive times that frequency error must fall below PPB_LOCK before a lock is deemed

//Pulse input: the software will automatically determine the likely BPH rate
#define PPS_IN		PB7					//pps pulse train input on EINT0 on pb7
#define PPS_INN		2					//pps pulses per second on EINT0 on pb7
#define TKS_MIN		5000				//minimum number of ticks to be considered a value input pulse
#define VTRLSLOPE_P						//if Vcontrl slope is positive (higher output voltage -> higher frequency). comment out if negative
//#define GPSDO_HEADER					//send header over uart, if defined.

//#define USE_PID						//deprecated: switch between errPID() or errSum()
//PID settings for coarse PID - prior to the OCXO has locked to the pps
//generally, shorter oversampling window (cPPS_OVN < fPPS_OVN) and higher gains
//cPPS_OVN must fully divide 1000000ul: factors of 2 and 5
#define cPPS_OVN	2					//oversample. gate time. practically unlimited
#define cPID_Ix		8					//PID gain for I_term, normalized by PID_x: normalized to 1
#define cPID_Px		256					//PID gain for P_term, normalized by PID_x: 64 .. 512 (for my system, the measured sensitivity is about 0.6hz -> 1 dc#)
#define cPID_Dx		16					//PID gain for D_term, normalized by PID_x: not used
#define cPID_x		1					//normalizer for PID_gains: may use it if the gain / timer period is too big (PPS_OVN >= 256)
//PID settings for fine PID - after the OCXO has locked to the pps
//generally, longer oversampling window (fPPS_OVN > cPPS_OVN) and lower gains
//good practice to answer fPID parameters to cPID parameters
//fPPS_OVN must fully divide 1000000ul: factors of 2 and 5
#define fPPS_OVN	10					//oversample. gate time. practically unlimited
#define fPID_Ix		(cPID_Ix)			//PID gain for I_term, normalized by PID_x: normalized to 1
#define fPID_Px		(cPID_Px * 1)		//PID gain for P_term, normalized by PID_x: 64 .. 512
#define fPID_Dx		(cPID_Dx)			//PID gain for D_term, normalized by PID_x: not used
#define fPID_x		(cPID_x * 1)		//normalizer for PID_gains: may use it if the gain / timer period is too big (PPS_OVN >= 256)

#define GPSDO_DEBUG						//use if we are debugging - enabling print out from errPID(). comment out if not used
#define TKS_OFST	(-100-8*1)			//tks offset - for debugging

//PWM output for timing errors - must have for the gpsdo to function
#define PWM_OUT							//pwm2/pb5 generator output pin. can be commented out - not recommended

//PPS output - optional
//generating Beats Per Hour pulses - to simulate a GPS 1pps output
#define PPS_OUT							//pps pulse output on pwm4/PB6. comment out if not needed
#define PPS_OUTN	1					//number of pulses per second. 1=1pps output, 2=2pps output

//reference frequency output - comment out if not needed
//user implemented -> may not be available on all chips
//#define REFCLK0_N	5					//divider for refclk0 - using REFCO (PIC24/PIC32) or MCO (STM32/PY32). output frequency = F_CPU / REFCLK0_N
#define REFCLK1_N	50000				//divider for refclk1 - using timer / pwm (50% dc). output frequency = F_CPU / REFCLK1_N. Available only if PPS_OUT is commented out
//#define REFCLK2_N	200					//divider for refclk2 - using timer / pwm (50% dc). output frequency = F_CPU / REFCLK2_N
//end hardware configuration

//global defines
//data type to hold frequency measurements in two pieces:
//1. Hz: frequency measured in hz
//2. uHz: the uHz portion of the frequency measured
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
#if defined(GPSDO_DEBUG)
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
				//ppsLocked=ppsLock();	//determine lock status - processed in main loop now
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
	int32_t tmp;

	//calculate the various terms
	I_term += tksErr;
	P_term  = tksErr;
	D_term  = tksErr - errPrev; errPrev = tksErr;

	//calculate the control on existing gain
	tmp = (I_term * PID_Ix + P_term * PID_Px + D_term * PID_Dx) / PID_x;						//fixed gain

#if defined(GPSDO_DEBUG)
	//for debugging only - comment out in actual runs
	//for fine tunning the pid
	u2Print("I_trm=                    ", I_term);
	//u2Print("P_trm=                    ", P_term);
	//u2Print("D_trm=                    ", D_term);
#endif	//ppsdo_debug

	return tmp;							//return the control
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

//output control signal
//option2: using dac1
uint16_t Ctrl2DAC16(int32_t ctrl) {
	//update dac only if ctrl<>0
	if (ctrl) dac1LSet(constrain(dac1LGet() + ctrl, 1, 0xffff));			//output timing errors on the dac1 L-channel generator
	return dac1LGet();
}

//lock status
//return 1/locked if frequency within range specified by PPB_LOCK for PPS_LOCKN times consecutively
//return 0/unlocked if the frequency error goes above PPB_LOCK on the first time
char ppsLock(void) {
	static uint32_t f_cpu = 0;				//reset f_cpu for the initial run
	static uint8_t cnt=0;					//count the number of consecutive runs when frequency error is less than PPB_LOCK
	char tmp;								//temp variable

	if (f_cpu==0) f_cpu = F_CPU;
	//tmp = 1 if in lock for this timing cycle. 0->not in lock
	if (abs(tksErr) > f_cpu / 1000000ul * PPB_LOCK * PPS_OVN / 1000) tmp = 0; else tmp = 1;
	//increment only if we are in lock (tmp == 1). otherwise, reset cnt
	if (tmp == 0) cnt=0; else cnt+=(cnt>PPS_LOCKN)?0:1;	//cnt maxies out at PPS_LOCKN, other than it increments to PPS_LOCKN
	//u2Print("cnt   =              ", cnt);	//print out cnt for debugging
	//only return 1 if we have lock for PPS_LOCKN times consecutively
	ppsLocked=(cnt>PPS_LOCKN)?1:0;		//return 1 if cnt == PPS_LOCKN, otherwise 0

	//update gains: use fine pid parameters if vcxo is locked to pps input.
	//otherwise, use coarse pid parameters for faster convergence
	if (ppsLocked) {PID_Ix = fPID_Ix; PID_Px = fPID_Px; PID_Dx = fPID_Dx; PID_x = fPID_x; PPS_OVN = fPPS_OVN;}
	else           {PID_Ix = cPID_Ix; PID_Px = cPID_Px; PID_Dx = cPID_Dx; PID_x = cPID_x; PPS_OVN = cPPS_OVN;}

	return ppsLocked;
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
//using timer3 / PWM4 (50% dc)
//only available if PPS_OUT is used
void refClk1Out(void) {
	//general workflow
	//set up divider(s)
	//enable the output pin
#if !defined(PPS_OUT) && defined(REFCLK1_N)		//only available if pps output is not used and REFCLK1_N is specified
	//tmr3 as pwm4 time base, 1:1 prescaler
	tmr3Init(TMR_PS1x); TMR3 = -1;
	//set up pwm4, using tmr3 as time base, 50% dc
	pwm4Init(); OC4CONbits.OCTSEL = 1; pwm4SetPR(REFCLK1_N); pwm4SetDC(pwm4GetPR() / 2);
#endif

}

//divider specified by REFCLK2_N
//using timer / PWM (50% dc)
void refClk2Out(void) {
	//general workflow
	//set up divider(s)
	//enable the output pin
}

//frequency conversion, from tksErr -> freq
FREQ_TypeDef *tks2freq(FREQ_TypeDef *frq) {
	int32_t tmp;									//temp variable

	//two ways to calculate freq
	//use of (1000000l / PPS_OVN) to avoid math error on C30 - likely a compiler bug
#if 0
	//this approach iterates
	frq->Hz = F_CPU;
	tmp = tksErr * (1000000l / PPS_OVN);				//calculate the temp variable - can overflow for large tksErr (>2100)
	//uart2Wait(); u2Print("PPS_OV=               ", PPS_OVN); u2Print("tksErr=               ", tksErr); u2Print("Hz0  =              ", frq->Hz); u2Print("tmp0  =               ", tmp); u2Println();
	do {
		//unlikely our error will be greater than 100Hz / PPS_OVN
		//so we will get the iterate only once
		frq->Hz -= 100;								//decrement freq, by 100hz
		tmp     += 100 * 1000000l;					//increment tmp, by 100hz(=100*1000000uHz)
		//uart2Wait(); u2Print("Hz1  =               ", frq->Hz); u2Print("tmp1  =               ", tmp); u2Println();
	} while (tmp < 0);
	frq->Hz += tmp / 1000000l;
	frq->uHz = tmp % 1000000l;
	//uart2Wait(); u2Print("Hz2  =               ", frq->Hz); u2Print("uHz2  =               ", frq->uHz); u2Println();
#else
	//no iteration - more reliable
	frq->Hz = F_CPU;								//Hz stores the nominal value
	//tmp = 0;
	if (tksErr < 0) {
		tmp = (-tksErr) / PPS_OVN; if ((-tksErr) % PPS_OVN) tmp += 1;	//find the number of Hz needed to borrower from frq->HZ
		frq->Hz -= tmp;								//borrow from frq->Hz
		frq->uHz = tmp * 1000000l + -((-tksErr) % PPS_OVN) * (1000000l / PPS_OVN);	//update the frq->uHz
	} else {
		tmp =  tksErr / PPS_OVN;					//find the number of Hz to be given to frq->Hz: if tksErr > PPS_OVN
		frq->Hz += tmp;								//add to frq->Hz
		frq->uHz =                  (tksErr % PPS_OVN) * (1000000l / PPS_OVN);	//calculate the remaining uHz
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

#if defined(PWM_OUT)
	//configure 16-bit PWM output for timing errors
#if defined(GPSDO_HEADER)
	uart2Wait(); uart2Puts("GPSDO starting TIM2/PWM2...\r\n");
#endif	//gpsdo
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
#if defined(GPSDO_HEADER)
	uart2Wait(); uart2Puts("GPSDO starting optional PPS output on TIM3/PWM4...\r\n");
#endif	//gpsdo
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

#if defined(GPSDO_HEADER)
	uart2Wait(); uart2Puts("GPSDO waiting for reference pulse ...\r\n");
#endif	//header
	//wait for the first cycle
	while (ppsFlg==0); ppsFlg=0;

#if defined(GPSDO_HEADER)
	//print out header
	uart2Wait();								//wait for uart2 to free up
	uart2Putline("Ghetto GPSDO, v0.6a. June 24, 2023");
	uart2Putline("key parameters:");
	u2Print("Fgpsdo=                  ", F_CPU);	u2Print("PPBLCK=                  ", PPB_LOCK); u2Println();
	u2Print("cOVN  =                  ", cPPS_OVN); u2Print("cPIDPx=                  ", cPID_Px); u2Print("cPID_x=                  ", cPID_x); u2Println();
	u2Print("fOVN  =                  ", fPPS_OVN); u2Print("fPIDPx=                  ", fPID_Px); u2Print("fPID_x=                  ", fPID_x); u2Println();
	uart2Puts("UART command module not supported"); u2Println();
#if defined(PPS_OUT)
	uart2Puts("PPS output enabled\r\n");
#else
	uart2Puts("PPS output disabled\r\n");
#endif	//pps_out

	uart2Wait();								//wait for uart2 to free up
#if defined(REFCLK0_N)
	uart2Puts("REFCLK0 output enabled: "); u2Print("F_ref0=               ", F_CPU / REFCLK0_N);
#else
	uart2Puts("REFCLK0 output disabled. "); //u2Print("F_ref1=               ", F_CPU / REFCLK1_N);
#endif	//refclk0
	u2Println();

	uart2Wait();								//wait for uart2 to free up
#if !defined(PPS_OUT) && defined(REFCLK1_N)
	uart2Puts("REFCLK1 output enabled: "); u2Print("F_ref1=               ", F_CPU / REFCLK1_N);
#else
	uart2Puts("REFCLK1 output disabled. "); //u2Print("F_ref1=               ", F_CPU / REFCLK1_N);
#endif	//refclk1
	u2Println();

	uart2Wait();								//wait for uart2 to free up
#if defined(REFCLK2_N)
	uart2Puts("REFCLK2 output enabled: "); u2Print("F_ref2=               ", F_CPU / REFCLK2_N);
#else
	uart2Puts("REFCLK2 output disabled. "); //u2Print("F_ref1=               ", F_CPU / REFCLK1_N);
#endif	//refclk1
	u2Println();

	uart2Wait();								//wait for uart2 to free up
	uart2Putline("GPSDO begins..."); u2Println();
#endif	//gpsdo_header
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
	ppsInit(); 										//skip over the first measurement

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

		//the main gpsdo control loop: measure timing error, update the pwm generator
		tmp = tksErr2Ctrl();						//calculate the control needed: 6.6k ticks
		Ctrl2DC16(tmp);								//output timing errors on the pwm2 generator: 57 ticks
		tks2freq(&freq); freq2uHz(&freq);			//update freq struct: 15K ticks (option 2), 14.9K ticks (option 1)
		ppsLock();									//determine pps lock status, update gains - last function to be executed
		//end gpsdo control loop
#if defined(LED_LOCK)
		if (ppsLocked) digitalWrite(LED_LOCK, HIGH); else digitalWrite(LED_LOCK, LOW);
#endif	//led_lock

		//measure timing
		tmp0=ticks();
		//put some here to benchmark
		tmp0=ticks() - tmp0;

		//display something
		u2Print("cnt   =                  ", cnt++);							//display counter
		uart2Puts("Fgpsdo ="); uart2Puts(freq.str); uart2Puts("   "); 			//display the frequency in string
		//u2Print("F_CPU =                  ", F_CPU);							//display cpu frequency
		//u2Print("DACR  =                  ", DAC1RDAT);						//display cpu frequency
		//u2Print("DAC EN=                  ", DAC1CONbits.DACEN);				//1->turn on dac1
		//DAC1LDAT = 12345; u2Print("DACL  =                  ", DAC1LDAT);							//display l-ch output
		//u2Print("DACLEN=                  ", DAC1STATbits.LOEN);				//1->turn on l-ch
		//u2Print("ticks =                  ", ticks());						//display cpu frequency
		//u2Print("tmp0  =                  ", tmp0);
		//u2Print("ppsTks=                  ", ppsTks);
		//u2Print("tksErr=                  ", tksErr);
		//u2Print("PPSOVN=                  ", PPS_OVN);							//print out oversampling horizon
		//u2PrintFrq(100000, -2);
		//u2Print("Frq100=                  ", Frqx100());
		//u2Print("ppsLck=                  ", ppsLock());
		uart2Puts(ppsLocked?"pps     locked...   ":"pps not locked...   ");
		//u2Print("DC_Chg=                  ", (tmp));								//display timing error, in ms per day
		//u2Print("PWMDC%=                  ", pwm2GetDC() * 10000ul/pwm2GetPR()*10);	//display timing error, off 50.000%, 1% = 1 sec.
		u2Print("PWMDC#=                  ", pwm2GetDC());						//display timing error, off 32768
		//u2Print("u2bps =                  ", u2bps());
		u2Println();
	}
}

