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
