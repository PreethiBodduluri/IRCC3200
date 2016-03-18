
#ifndef IRREMOTERECEIVER_H_
#define IRREMOTERECEIVER_H_
#include "Energia.h"

#define TOPBIT 0x80000000

#define TIMER_TIMB_TIMEOUT  0x00000001

#define USECPERTICK 50  // microseconds per clock interrupt tick
#define RAWBUF 100 // Length of raw duration buffer

#define MARK  0
#define SPACE 1

// receiver states
#define STATE_IDLE     2
#define STATE_MARK     3
#define STATE_SPACE    4
#define STATE_STOP     5

#define _GAP 5000 // Minimum map between transmissions
#define GAP_TICKS (_GAP/USECPERTICK)

#define ERR 0
#define DECODED 1

#define UNKNOWN -1

#define MARK_EXCESS 100

#define TOLERANCE 30  // percent tolerance in measurements
#define LTOL (1.0 - TOLERANCE/100.)
#define UTOL (1.0 + TOLERANCE/100.)

#define TICKS_LOW(us) (int) (((us)*LTOL/USECPERTICK))
#define TICKS_HIGH(us) (int) (((us)*UTOL/USECPERTICK + 1))

// Set DEBUG to 1 for debug output

#define DEBUG  0

// Debug directives

#if DEBUG
#	define DBG_PRINT(...)    Serial.print(__VA_ARGS__)
#	define DBG_PRINTLN(...)  Serial.println(__VA_ARGS__)
#else
#	define DBG_PRINT(...)
#	define DBG_PRINTLN(...)
#endif

int MATCH(int measured, int desired);
int MATCH_MARK(int measured_ticks, int desired_us);
int MATCH_SPACE(int measured_ticks, int desired_us);

#define NEC_BITS 32
#define SONY_BITS 12

#define REPEAT 0xffffffff

#define NEC_HDR_MARK  9000
#define NEC_HDR_SPACE 4500
#define NEC_BIT_MARK  560
#define NEC_ONE_SPACE 1600
#define NEC_ZERO_SPACE  560
#define NEC_RPT_SPACE 2250

#define SONY_HDR_MARK 2400
#define SONY_HDR_SPACE  600
#define SONY_ONE_MARK 1200
#define SONY_ZERO_MARK  600
#define SONY_RPT_LENGTH 45000
#define SONY_DOUBLE_SPACE_USECS  500

// Values for decode_type
#define NEC 1
#define SONY 2
#define SANYO 9

#define RECV_PIN  31

#define NO_OF_APPLIANCES 4  // NO OF APPLIANCES CONTROLLED
#define NO_OF_LEVEL_APPPLIANCES 2 //no of appliances which has intensity level
#define NO_OF_AUTO_TIME_OUT 4 //no of settings for auto mode time out


class generationLibrary {

private:

public:
	void pwmOn(int time);
	void pwmOff(int time);
	void Sony(unsigned long data, int nbits);
	void Samsung(unsigned long data, int nbits);
	void Sanyo(unsigned long data, int nbits);
	void Raw(unsigned int buf[], int len);
	void initTimer();
	void setTimerA3B();
	void SendNec(unsigned long data, int nbits);
};

// information for the interrupt handler
typedef struct {
	uint8_t recvpin;           // pin for IR data from detector
	uint8_t rcvstate;          // state machine
	unsigned int timer;     // state timer, counts 50uS ticks.
	unsigned int rawbuf[RAWBUF]; // raw data
	uint8_t rawlen;         // counter of entries in rawbuf
} irparams_t;

extern volatile irparams_t irparams;

class decode_results {
public:
	int decode_type; // NEC, SONY, RC5, UNKNOWN
	unsigned long value; // Decoded value
	int bits; // Number of bits in decoded value
	volatile unsigned int *rawbuf; // Raw intervals in .5 us ticks
	int rawlen; // Number of records in rawbuf.
};

class ReceivingLibrary {

private:
	long decodeNEC(decode_results *results);
	long decodeSony(decode_results *results);

public:
	void Receiving(int recvpin);
	void setTimerA1B();
	int decode(decode_results *results);
	long decodeHash(decode_results *results);
	int compare(unsigned int oldval, unsigned int newval);
	void enableIRIn();
	void resume();
	int ir_signal_counter;
	unsigned long receivedData;
};

enum TouchType{
	SINGLE_PRESS,
	LONG_PRESS,
	LEVEL_TYPE,
	AUTO_TIME_OUT,
	LEVEL_TYPE_FAST
};

typedef  struct TouchDetectData{
	int touchedSwitch;
	TouchType touchType;
}touchDetectData;


class  IRRemoteReceiver {

public:

	void setIRDelegate(void (*irDataDelegate)(TouchDetectData));
	void initIR();
	void monitorIR();
//	void detectInputDataDelegate(void (*)(TouchDetectData), bool (*)(void));//
//	void detectSetMode(bool(*)(void));                                      //
//	void (*timerDataDelegate)(TouchDetectData); //
//	bool (*isSameInputDetected)();//

private:
	decode_results results;
    int codeType;
	int codeLen;
	unsigned long codeValue;
	unsigned int rawCodes[RAWBUF];
	unsigned long irData;
//	void (*irDataCallback)(TouchDetectData);
	void processIRCode(decode_results *results);
	unsigned long applianceIRCode[NO_OF_APPLIANCES];
	unsigned long increaseLevelIRCode[NO_OF_LEVEL_APPPLIANCES];
	unsigned long decreaseLevelIRCode[NO_OF_LEVEL_APPPLIANCES];
	unsigned long autoModeTimeOutIRCode[NO_OF_AUTO_TIME_OUT];
	int autoTimeOutTimeValue[NO_OF_AUTO_TIME_OUT];

};

class IRLibrary {
private:
public:
	void init_IR();
	void monitor_IR();
};

extern "C" void Switch_on_device(void);
extern  int setforSendEnable;
extern 	int isReceivedDataToBeProcessed;
extern "C" void perform_IRreceive(void);
//extern "C" void setIRreceivedData(TouchDetectData);
//extern "C" bool isSameIRInputDetected();
#endif /* IRREMOTERECEIVER_H_ */
