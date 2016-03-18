
#include "IRRemotereceiver.h"
#include <wiring_analog.c>
#include <wiring_private.h>
#include <driverlib/interrupt.h>
#include <driverlib/timer.h>


generationLibrary generationlibrary;
ReceivingLibrary receivinglibrary;
decode_results results;
IRRemoteReceiver irremotereceiver;

TouchDetectData touchedData;
int isReceivedDataToBeProcessed;
int autoModeTimeOut;

volatile irparams_t irparams;                 /// used for receiving
volatile unsigned long ticksPerMicrosecond;  /// used to set the timer for 50us tick

/// parameters for generation of PWM
int PWMpin = 29;
int numberOfSteps = 100;
int dutyCycle = 50;
int frequency = 38000;

volatile unsigned long ticks; /// used to specify the ticks in ms for recv_send code

int setforSendEnable; /// used for providing delay in the recv_send code for proximity.

int timerOverFlowCounter; /// used for providing delay in the recv_send code for proximity.

/// generation code
/// This function is used to indicate ON time or the MARK
void generationLibrary::pwmOn(int time) {
	pinMode(PWMpin, OUTPUT);
	PWMWrite(PWMpin, numberOfSteps, dutyCycle, frequency); /// This function is used to generate the PWM signal for IR
	delayMicroseconds(time);
}
/// This function is used to indicate OFF time or the SPACE
void generationLibrary::pwmOff(int time) {
	uint8_t timer = digitalPinToTimer(PWMpin);
	uint32_t base = TIMERA0_BASE + ((timer / 2) << 12);
	uint16_t timerab = timer % 2 ? TIMER_B : TIMER_A;
	MAP_TimerDisable(base, timerab);
	pinMode(PWMpin, OUTPUT);
	digitalWrite(PWMpin, LOW);
	delayMicroseconds(time);
}

/// These functions are used for generation of signals according to different protocols
void generationLibrary::Sony(unsigned long data, int nbits)

{
	PWMWrite(29, 100, 50, 38000);
	pwmOn(2400);
	pwmOff(600);
	data = data << (32 - nbits);
	for (int i = 0; i < nbits; i++) {
		if (data & TOPBIT) {
			pwmOn(1200);
			pwmOff(600);
		} else {
			pwmOn(600);
			pwmOff(600);
		}
		data <<= 1;
	}
	pwmOff(0);
}

void generationLibrary::Sanyo(unsigned long data, int nbits)

{
	PWMWrite(29, 100, 50, 38000);
	pwmOn(3500);
	pwmOff(950);
	data = data << (32 - nbits);
	for (int i = 0; i < nbits; i++) {
		if (data & TOPBIT) {
			pwmOn(2400);
			pwmOff(950);
		} else {
			pwmOn(700);
			pwmOff(950);
		}
		data <<= 1;
	}
	pwmOff(0);
}
void generationLibrary::Samsung(unsigned long data, int nbits)

{
	PWMWrite(29, 100, 50, 38000);
	pwmOn(5000);
	pwmOff(5000);
	for (unsigned long mask = 1UL << (nbits - 1); mask; mask >>= 1) {
		if (data & mask) {
			pwmOn(560);
			pwmOff(1600);
		} else {
			pwmOn(560);
			pwmOff(560);
		}
	}
	pwmOn(560);
	pwmOff(0);
}

void generationLibrary::Raw(unsigned int buf[], int len) {
	PWMWrite(29, 100, 50, 38000);
	for (int i = 0; i < len; i++) {
		if (i & 1) {
			pwmOff(buf[i]);
		} else {
			pwmOn(buf[i]);
		}
	}
	pwmOff(0);
}

/// used for providing delay in recv_send code in proximity

void generationLibrary::initTimer() {
	ticks = 0;
	timerOverFlowCounter = 0;
	MAP_PRCMPeripheralClkEnable(PRCM_TIMERA3, PRCM_RUN_MODE_CLK);
	MAP_PRCMPeripheralReset (PRCM_TIMERA3);
	MAP_TimerConfigure(TIMERA3_BASE, TIMER_CFG_B_ONE_SHOT | TIMER_CFG_SPLIT_PAIR);
	MAP_TimerPrescaleSet(TIMERA3_BASE, TIMER_B,255);
	MAP_TimerPrescaleMatchSet(TIMERA3_BASE, TIMER_B, 255);
}
void generationLibrary::setTimerA3B() {
	TimerIntRegister(TIMERA3_BASE, TIMER_B, Switch_on_device);
	MAP_TimerIntEnable(TIMERA3_BASE, TIMER_TIMB_TIMEOUT);
	// load the timer
	MAP_TimerLoadSet(TIMERA3_BASE, TIMER_B,62500);
	// Enable the timer.
	MAP_TimerEnable(TIMERA3_BASE, TIMER_B);

}
void Switch_on_device() {
	MAP_TimerIntClear(TIMERA3_BASE, TIMER_TIMB_TIMEOUT);
	timerOverFlowCounter =  timerOverFlowCounter + 200;
	if ( timerOverFlowCounter >= 1000){
		setforSendEnable= 1;
		timerOverFlowCounter = 0;
	}
	// load the timer
	MAP_TimerLoadSet(TIMERA3_BASE, TIMER_B,62500);
	// Enable the timer.
	MAP_TimerEnable(TIMERA3_BASE, TIMER_B);
}

/// receiving code

int MATCH(int measured, int desired) {
	DBG_PRINT("Testing: ");DBG_PRINT(TICKS_LOW(desired), DEC);DBG_PRINT(" <= ");DBG_PRINT(measured, DEC);DBG_PRINT(" <= ");DBG_PRINTLN(TICKS_HIGH(desired), DEC);

	return ((measured >= TICKS_LOW(desired))
			&& (measured <= TICKS_HIGH(desired)));
}

// Due to sensor lag, when received, Marks tend to be 100us too long
int MATCH_MARK(int measured_ticks, int desired_us) {
	DBG_PRINT("Testing mark ");DBG_PRINT(measured_ticks * USECPERTICK, DEC);DBG_PRINT(" vs ");DBG_PRINT(desired_us, DEC);DBG_PRINT(": ");DBG_PRINT(TICKS_LOW(desired_us + MARK_EXCESS), DEC);DBG_PRINT(" <= ");DBG_PRINT(measured_ticks, DEC);DBG_PRINT(" <= ");DBG_PRINTLN(TICKS_HIGH(desired_us + MARK_EXCESS), DEC);

	return ((measured_ticks >= TICKS_LOW(desired_us + MARK_EXCESS))
			&& (measured_ticks <= TICKS_HIGH(desired_us + MARK_EXCESS)));
}

// Due to sensor lag, when received, Spaces tend to be 100us too short
int MATCH_SPACE(int measured_ticks, int desired_us) {
	DBG_PRINT("Testing space ");DBG_PRINT(measured_ticks * USECPERTICK, DEC);DBG_PRINT(" vs ");DBG_PRINT(desired_us, DEC);DBG_PRINT(": ");DBG_PRINT(TICKS_LOW(desired_us - MARK_EXCESS), DEC);DBG_PRINT(" <= ");DBG_PRINT(measured_ticks, DEC);DBG_PRINT(" <= ");DBG_PRINTLN(TICKS_HIGH(desired_us - MARK_EXCESS), DEC);

	return ((measured_ticks >= TICKS_LOW(desired_us - MARK_EXCESS))
			&& (measured_ticks <= TICKS_HIGH(desired_us - MARK_EXCESS)));
}

void ReceivingLibrary::Receiving(int recvpin) {
	irparams.recvpin = recvpin;
}

void ReceivingLibrary::enableIRIn() {

	ticksPerMicrosecond = 0;
	MAP_PRCMPeripheralClkEnable(PRCM_TIMERA1, PRCM_RUN_MODE_CLK);
	MAP_PRCMPeripheralReset (PRCM_TIMERA1);
	MAP_TimerConfigure(TIMERA1_BASE,TIMER_CFG_B_PERIODIC | TIMER_CFG_SPLIT_PAIR);
	MAP_TimerPrescaleSet(TIMERA1_BASE, TIMER_B, 0);
	MAP_TimerPrescaleMatchSet(TIMERA1_BASE, TIMER_B, 0);
	// initialize state machine variables
	irparams.rcvstate = STATE_IDLE;
	irparams.rawlen = 0;
	// set pin modes
	pinMode(irparams.recvpin, INPUT);
}

///used to set the timer for 50us tick
void ReceivingLibrary::setTimerA1B() {
	TimerIntRegister(TIMERA1_BASE, TIMER_B, perform_IRreceive);
	ticksPerMicrosecond = (80000000) / 1000000;
	// load the timer
	MAP_TimerLoadSet(TIMERA1_BASE, TIMER_B, ticksPerMicrosecond * 50);
	MAP_TimerIntEnable(TIMERA1_BASE, TIMER_TIMB_TIMEOUT);
	// Enable the timer.
	MAP_TimerEnable(TIMERA1_BASE, TIMER_B);
}

/// ISR
void perform_IRreceive() {
	MAP_TimerIntClear(TIMERA1_BASE, TIMER_TIMB_TIMEOUT);

	uint8_t irdata = (uint8_t) digitalRead(irparams.recvpin);

	irparams.timer++; // One more 50us tick
	if (irparams.rawlen >= RAWBUF) {
		// Buffer overflow
		irparams.rcvstate = STATE_STOP;
	}
	switch (irparams.rcvstate) {
	case STATE_IDLE: // In the middle of a gap
		if (irdata == MARK) {
			if (irparams.timer < GAP_TICKS) {
				// Not big enough to be a gap.
				irparams.timer = 0;
			} else {
				// gap just ended, record duration and start recording transmission
				irparams.rawlen = 0;
				irparams.rawbuf[irparams.rawlen++] = irparams.timer;
				irparams.timer = 0;
				irparams.rcvstate = STATE_MARK;
			}
		}
		break;
	case STATE_MARK: // timing MARK
		if (irdata == SPACE) {   // MARK ended, record time
			irparams.rawbuf[irparams.rawlen++] = irparams.timer;
			irparams.timer = 0;
			irparams.rcvstate = STATE_SPACE;
		}
		break;
	case STATE_SPACE: // timing SPACE
		if (irdata == MARK) { // SPACE just ended, record it
			irparams.rawbuf[irparams.rawlen++] = irparams.timer;
			irparams.timer = 0;
			irparams.rcvstate = STATE_MARK;
		} else { // SPACE
			if (irparams.timer > GAP_TICKS) {
				// big SPACE, indicates gap between codes
				// Mark current code as ready for processing
				// Switch to STOP
				// Don't reset timer; keep counting space width
				irparams.rcvstate = STATE_STOP;
			}
		}
		break;
	case STATE_STOP: // waiting, measuring gap
		if (irdata == MARK) { // reset gap timer
			irparams.timer = 0;
		}
		break;
	}
}

void ReceivingLibrary::resume() {
	irparams.rcvstate = STATE_IDLE;
	irparams.rawlen = 0;
}

// Decodes the received IR message
// Returns 0 if no data ready, 1 if data ready.
// Results of decoding are stored in results
int ReceivingLibrary::decode(decode_results *results) {
	results->rawbuf = irparams.rawbuf;
	results->rawlen = irparams.rawlen;
	if (irparams.rcvstate != STATE_STOP) {
		return ERR;
	}
#ifdef DEBUG
	if (decodeNEC(results)) {
		Serial.println("Attempting NEC decode");
		return DECODED;
	}
#endif
#ifdef DEBUG
	if (decodeSony(results)) {
		Serial.println("Attempting Sony decode");
		return DECODED;
	}
#endif
	if (decodeHash(results)) {
		return DECODED;
	}
	// Throw away and start over
	resume();
	return ERR;
}

// NECs have a repeat only 4 items long
long ReceivingLibrary::decodeNEC(decode_results *results) {
	long data = 0;
	int offset = 1; // Skip first space
	// Initial mark
	if (!MATCH_MARK(results->rawbuf[offset], NEC_HDR_MARK)) {
		return ERR;
	}
	offset++;
	// Check for repeat
	if (irparams.rawlen == 4
			&& MATCH_SPACE(results->rawbuf[offset], NEC_RPT_SPACE)
	&& MATCH_MARK(results->rawbuf[offset + 1], NEC_BIT_MARK)) {
		results->bits = 0;
		results->value = REPEAT;
		results->decode_type = NEC;
		ir_signal_counter++;
		return DECODED;
	}
	if (irparams.rawlen < 2 * NEC_BITS + 4) {
		return ERR;
	}
	// Initial space
	if (!MATCH_SPACE(results->rawbuf[offset], NEC_HDR_SPACE)) {
		return ERR;
	}
	offset++;
	for (int i = 0; i < NEC_BITS; i++) {
		if (!MATCH_MARK(results->rawbuf[offset], NEC_BIT_MARK)) {
			return ERR;
		}
		offset++;
		if (MATCH_SPACE(results->rawbuf[offset], NEC_ONE_SPACE)) {
			data = (data << 1) | 1;
		} else if (MATCH_SPACE(results->rawbuf[offset], NEC_ZERO_SPACE)) {
			data <<= 1;
		} else {
			return ERR;
		}
		offset++;
	}
	// Success
	results->bits = NEC_BITS;
	results->value = data;
	results->decode_type = NEC;
	ir_signal_counter = 0;
	receivedData = data;
	return DECODED;
}

long ReceivingLibrary::decodeSony(decode_results *results) {
	long data = 0;
	if (irparams.rawlen < 2 * SONY_BITS + 2) {
		return ERR;
	}
	int offset = 0; // Dont skip first space, check its size

	// Some Sony's deliver repeats fast after first
	// unfortunately can't spot difference from of repeat from two fast clicks
	if (results->rawbuf[offset] < SONY_DOUBLE_SPACE_USECS) {
		// Serial.print("IR Gap found: ");
		results->bits = 0;
		results->value = REPEAT;
		results->decode_type = SANYO;
		return DECODED;
	}
	offset++;

	// Initial mark
	if (!MATCH_MARK(results->rawbuf[offset], SONY_HDR_MARK)) {
		return ERR;
	}
	offset++;

	while (offset + 1 < irparams.rawlen) {
		if (!MATCH_SPACE(results->rawbuf[offset], SONY_HDR_SPACE)) {
			break;
		}
		offset++;
		if (MATCH_MARK(results->rawbuf[offset], SONY_ONE_MARK)) {
			data = (data << 1) | 1;
		} else if (MATCH_MARK(results->rawbuf[offset], SONY_ZERO_MARK)) {
			data <<= 1;
		} else {
			return ERR;
		}
		offset++;
	}

	// Success
	results->bits = (offset - 1) / 2;
	if (results->bits < 12) {
		results->bits = 0;
		return ERR;
	}
	results->value = data;
	results->decode_type = SONY;
	ir_signal_counter = 0;
	receivedData = data;
	return DECODED;
}

/* hashdecode - decode an arbitrary IR code.
 * Instead of decoding using a standard encoding scheme
 * (e.g. Sony, NEC, RC5), the code is hashed to a 32-bit value.
 *
 * The algorithm: look at the sequence of MARK signals, and see if each one
 * is shorter (0), the same length (1), or longer (2) than the previous.
 * Do the same with the SPACE signals.  Hszh the resulting sequence of 0's,
 * 1's, and 2's to a 32-bit value.  This will give a unique value for each
 * different code (probably), for most code systems.
 *
 * http://arcfn.com/2010/01/using-arbitrary-remotes-with-arduino.html
 */

// Compare two tick values, returning 0 if newval is shorter,
// 1 if newval is equal, and 2 if newval is longer
// Use a tolerance of 20%
int ReceivingLibrary::compare(unsigned int oldval, unsigned int newval) {
	if (newval < oldval * .8) {
		return 0;
	}
	else if (oldval < newval * .8) {
		return 2;
	}
	else {
		return 1;
	}
}

// Use FNV hash algorithm: http://isthe.com/chongo/tech/comp/fnv/#FNV-param
#define FNV_PRIME_32 16777619
#define FNV_BASIS_32 2166136261

/* Converts the raw code values into a 32-bit hash code.
 * Hopefully this code is unique for each button.
 * This isn't a "real" decoding, just an arbitrary value.
 */
long ReceivingLibrary::decodeHash(decode_results *results) {
	// Require at least 6 samples to prevent triggering on noise
	if (results->rawlen < 6) {
		return ERR;
	}
	long hash = FNV_BASIS_32;
	for (int i = 1; i+2 < results->rawlen; i++) {
		int value =  compare(results->rawbuf[i], results->rawbuf[i+2]);
		// Add value into the hash
		hash = (hash * FNV_PRIME_32) ^ value;
	}
	results->value = hash;
	results->bits = 32;
	results->decode_type = UNKNOWN;
	return DECODED;
}
//void IRRemoteReceiver::setIRDelegate(void (*irDataDelegate)(TouchDetectData)) {
//	irDataCallback = irDataDelegate;
//}
//
//void IRRemoteReceiver::detectInputDataDelegate(
//		void (*timerDataCallback)(TouchDetectData),
//		bool (*isSameInputDetectedCallback)()) {
//	timerDataDelegate = timerDataCallback;
//	isSameInputDetected = isSameInputDetectedCallback;
//}
//
//void IRRemoteReceiver::detectSetMode(bool (*isSameInputDetectedCallback)()){
//	isSameInputDetected = isSameInputDetectedCallback;
//}

void IRRemoteReceiver::initIR()
{
	codeType = -1;
	isReceivedDataToBeProcessed = 0;
	applianceIRCode[0] = 0x44BB49B6;	//1				//0xFF9867;
	applianceIRCode[1] = 0x44BBC936;	//2				//0xFFD827;
	applianceIRCode[2] = 0x44BB33CC;	//3				//0xFF8877;
	applianceIRCode[3] = 0x44BB718E;	//4			    //0xFFA857;

	increaseLevelIRCode[0] = 0x44BBF10E;	//5			//0xFF906F;
	increaseLevelIRCode[1] = 0x44BB13EC;	//6			//0xFFB847;

	decreaseLevelIRCode[0] = 0x44BB51AE;  //7		   //0xFFE817;
	decreaseLevelIRCode[1] = 0x44BBD12E;	//8		   //0xFF48B7;

	autoModeTimeOutIRCode[0] = 0xFF7A85;
	autoModeTimeOutIRCode[1] = 0xFF609F;
	autoModeTimeOutIRCode[2] = 0xFFA05F;
	autoModeTimeOutIRCode[3] = 0xFFC03F;


	autoTimeOutTimeValue[0] = 120; //2 minutes
	autoTimeOutTimeValue[1] = 300; // 5 minutes
	autoTimeOutTimeValue[2] = 900; // 15 minutes
	autoTimeOutTimeValue[3] = 1800; // infinite
}
void IRRemoteReceiver::monitorIR()
{
	if (receivinglibrary.decode(&results)) {
		if (isReceivedDataToBeProcessed == 0) {
			isReceivedDataToBeProcessed = 0;
			processIRCode(&results);
			int irRequestDetected = 0;
			TouchDetectData inputIRData;
			//make irData from the result and send it to irDataCallback
			//check for on/off or auto of the appliance
			for (int i = 0; i < NO_OF_APPLIANCES; i++) {
				if ( receivinglibrary.receivedData == applianceIRCode[i]) {
					Serial.println("appliance IR code received");
					irRequestDetected = 1;
					inputIRData.touchType = touchedData.touchType;
					inputIRData.touchedSwitch = i;
					if (touchedData.touchType == SINGLE_PRESS) {
						Serial.print("Single press : ");
					} else {
						Serial.print("Long press : ");
					}
					Serial.println(i);
					break;
				}
			}

			//check for the change in level of any appliance
			if (!irRequestDetected) {

				for (int i = 0; i < NO_OF_LEVEL_APPPLIANCES; i++) {
					if (receivinglibrary.receivedData == increaseLevelIRCode[i]) {
						Serial.println("increseLevelIRCode received");
						irRequestDetected = 1;
						//increase of level detected
						inputIRData.touchType = LEVEL_TYPE;
						if (i == 0) {
							inputIRData.touchedSwitch = 6;
						} else {
							inputIRData.touchedSwitch = 4;
						}
						Serial.print("increase level : ");
						Serial.println(i);
						break;
					}
					if (receivinglibrary.receivedData == decreaseLevelIRCode[i]) {
						Serial.println("decreaseLevelIRCode received");
						irRequestDetected = 1;
						//decrease of level detected
						inputIRData.touchType = LEVEL_TYPE;
						if (i == 0) {
							inputIRData.touchedSwitch = 7;
						} else {
							inputIRData.touchedSwitch = 5;
						}
						Serial.print("decrease level : ");
						Serial.println(i);
						break;
					}
				}
			}

			//check for the settings of different auto mode timeout
			if (!irRequestDetected) {
				for (int i = 0; i < NO_OF_AUTO_TIME_OUT; i++) {
					if (receivinglibrary.receivedData == autoModeTimeOutIRCode[i]) {
						Serial.println("automodeTimeoutIrcode Received");
						irRequestDetected = 1;
						//send the request to set auto mode time out
						inputIRData.touchType = AUTO_TIME_OUT;
						autoModeTimeOut = autoTimeOutTimeValue[i];
						Serial.print("auto time out : ");
						Serial.println(i);
						break;
					}
				}
			}
			//			if (irRequestDetected && results.value > 0) {
			//				irDataCallback(inputIRData);
			//				//make inputIRData to null
			//			}
		}
		//			detectInputDataDelegate(setIRreceivedData,
		//					isSameIRInputDetected);
		setforSendEnable=0;
		generationlibrary.setTimerA3B();
		receivinglibrary.resume();
		//			setTimer0(FOR_INPUT); /// set timer functions are included
	}
}


void IRRemoteReceiver::processIRCode(decode_results *results) {
	codeType = results->decode_type;
	int count = results->rawlen;
	long data = 0;
	if (codeType == UNKNOWN) {
		//	if(1){
		Serial.println("Received unknown code, saving as raw");
		codeLen = results->rawlen - 1;
		// To store raw codes:
		// Drop first value (gap)
		// Convert from ticks to microseconds
		// Tweak marks shorter, and spaces longer to cancel out IR receiver distortion
		for (int i = 1; i <= codeLen; i++) {
			if (i % 2) {
				// Mark
				rawCodes[i - 1] =
						results->rawbuf[i] * USECPERTICK - MARK_EXCESS;
				Serial.print(" m");
			} else {
				// Space
				rawCodes[i - 1] =
						results->rawbuf[i] * USECPERTICK + MARK_EXCESS;
				Serial.print(" s");
			}
			Serial.print(rawCodes[i - 1], DEC);
		}
		Serial.println("");
	} else {
		if (codeType == NEC) {
			if (results->value == REPEAT) {
				// Don't record a NEC repeat value as that's useless.
				Serial.println("repeat; ignoring.");
				return;
			}
		}
		else if (codeType == SONY) {
		}
		Serial.println(results->value, HEX);
		codeValue = results->value;
		codeLen = results->bits;
	}
}
//void setIRreceivedData(TouchDetectData data) {
//	isReceivedDataToBeProcessed = 1;
//	touchedData.touchType = data.touchType;
//}

//int ir_signal_counter = 0;
//bool isSameIRInputDetected() {
//	if (receivinglibrary.ir_signal_counter > ir_signal_counter) {
//		ir_signal_counter = receivinglibrary.ir_signal_counter;
//		Serial.print(ir_signal_counter);
//		return true;
//	} else {
//		ir_signal_counter = 0;
//	}
//	return false;
//}

// powerOn is an example used. The data can be changed.
unsigned int powerOn[] = {1000,500,1050,2450,1000,500,1000,2500,1000 ,500,1000,2450,1050,500,1000,2450,1000,2500,1000,2450,1050,2450,1000,500,1000,2500,1000,500,1000,500,1050,450,1050,2450,1000,500,1000,500,1050,450, 1050,500,1000,2450,1050 ,2450,1000,2450,1050,500,1000,500,1000,500,1000,2500,1000,500,1000,500,1000,2500,1000,2450,1050,450,1050,500,1000,500,1000,500,1000,500,1000,500,1050,450,1050,500,1000,500,1000,500,1000,500,1000,500,1000,500,1050,450,1050,500,1000,500,1000,2450,1050};
void IRLibrary::init_IR()
{
	setforSendEnable=1;
	receivinglibrary.Receiving (RECV_PIN); /// This is the pin used for receiving.
	receivinglibrary.enableIRIn();
	receivinglibrary.setTimerA1B();
	generationlibrary.initTimer();
	irremotereceiver.initIR();
}
// This function is used to generate the signals continuously for proximity.
void IRLibrary::monitor_IR()
{
	//if (Serial.read() != -1)
		 if (setforSendEnable==1)
	{
		for (int i =0;i<3;i++)
		{
			//generationlibrary.Sanyo(0x1234,16);
			//generationlibrary.Samsung(0x4049F,32);
			generationlibrary.Raw(powerOn,99);
			delay(100);
		}
		setforSendEnable=0;
		generationlibrary.setTimerA3B();
	}
	irremotereceiver.monitorIR();
}

