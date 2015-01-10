#include <Streaming.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <RFM69.h>
#include <SPI.h>
#include <SPIFlash.h>


#define DEBUG 0
#define MIN 60

#define TRESHOLD 200
#define MEASUREMENTS 7

#define NODEID      99
#define NETWORKID   100
#define GATEWAYID   1
#define FREQUENCY   RF69_915MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
#define KEY         "thisIsEncryptKey" //has to be same 16 characters/bytes on all nodes, not more not less!

#define SERIAL_BAUD 115200
#define ACK_TIME    30  // # of ms to wait for an ack

#define TEMP_CALIBRATION 0

const long scaleConst = 1156.300 * 1000 ; // internalRef * 1023 * 1000;

int           lcdA = 0, lcdB = 0, lcdC = 0;
int           scalePower = A0;
bool          lcd1, lcd2, lcd3, lcd4, lcd5, lcd6, lcd7, lcd8;
unsigned int  SAn, SBn, SCn;
unsigned int  SA[8], SB[8], SC[8];
bool          A[8], B[8], C[8];

int TRANSMITPERIOD = 300; //transmit a packet to gateway so often (in ms)
byte sendSize = 0;
boolean requestACK = false;
SPIFlash flash(8, 0xEF30); //EF40 for 16mbit windbond chip

RFM69 radio;


//########################################################################################################################
// Power management
//########################################################################################################################

// watchdog interrupt
ISR(WDT_vect)
{
	wdt_disable();  // disable watchdog
}

void myWatchdogEnable(const byte interval)
{
	MCUSR = 0;                          // reset various flags
	WDTCSR |= 0b00011000;               // see docs, set WDCE, WDE
	WDTCSR =  0b01000000 | interval;    // set WDIE, and appropriate delay

	wdt_reset();
	set_sleep_mode (SLEEP_MODE_PWR_DOWN);
	sleep_mode();            // now goes to Sleep and waits for the interrupt
}

//
void sleep(int seconds)
{
	// sleep bit patterns:
	//  1 second:  0b000110
	//  2 seconds: 0b000111
	//  4 seconds: 0b100000
	//  8 seconds: 0b100001

	int i;
	for (i = 0; i < seconds / 8; i++)
	{
		myWatchdogEnable (0b100001);  // 8 seconds
	}
	seconds = seconds % 8;

	for (i = 0; i < seconds / 4; i++)
	{
		myWatchdogEnable (0b100000);  // 4 seconds
	}
	seconds = seconds % 4;

	for (i = 0; i < seconds / 2; i++)
	{
		myWatchdogEnable (0b000111);  // 2 seconds
	}
	seconds = seconds % 2;

	for (i = 0; i < seconds; i++)
	{
		myWatchdogEnable (0b000110);  // 1 second
	}

}

//########################################################################################################################


// Function that printf and related will use to print
int serial_putchar(char c, FILE* f)
{
	if (c == '\n') serial_putchar('\r', f);
	Serial.write(c);
	return 0;
}

FILE serial_stdout;


//########################################################################################################################
//Data Structure to be sent
//########################################################################################################################

typedef struct
{
	int           nodeId; //store this nodeId
	unsigned long uptime; //uptime in ms
	int weight;    // Weight
	int supplyV;      // Supply voltage
	byte outTemp; // Temperature of RF module outside hive. Probably not ver accurate as it's the RF chip temp.
	int inTemp; // TH02 Temperature inside hive
	int humidity; // TH02 humidity inside hive
}
Payload;

Payload theData;

//########################################################################################################################



void setup()
{
	pinMode(scalePower, INPUT);
	digitalWrite(scalePower, LOW);

	Serial.begin(SERIAL_BAUD);
	radio.initialize(FREQUENCY, NODEID, NETWORKID);
	//radio.setHighPower(); //uncomment only for RFM69HW!
	radio.encrypt(KEY);
	char buff[50];
	sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY == RF69_433MHZ ? 433 : FREQUENCY == RF69_868MHZ ? 868 : 915);
	Serial.println(buff);

	if (flash.initialize())
		Serial.println("SPI Flash Init OK!");
	else
		Serial.println("SPI Flash Init FAIL! (is chip present?)");

	//#if DEBUG
	// Set up stdout for printf
	fdev_setup_stream(&serial_stdout, serial_putchar, NULL, _FDEV_SETUP_WRITE);
	stdout = &serial_stdout;
	Serial << "Initialized..." << endl;
	Serial.flush();

	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
}

int weight = 0;
long lastPeriod = -1;
void loop()
{

	//process any serial input
	if (Serial.available() > 0)
	{
		char input = Serial.read();
		if (input >= 48 && input <= 57) //[0,9]
		{
			TRANSMITPERIOD = 100 * (input - 48);
			if (TRANSMITPERIOD == 0) TRANSMITPERIOD = 1000;
			Serial.print("\nChanging delay to ");
			Serial.print(TRANSMITPERIOD);
			Serial.println("ms\n");
		}

		if (input == 'r') //d=dump register values
			radio.readAllRegs();
		//if (input == 'E') //E=enable encryption
		//  radio.encrypt(KEY);
		//if (input == 'e') //e=disable encryption
		//  radio.encrypt(null);

		if (input == 'd') //d=dump flash area
		{
			Serial.println("Flash content:");
			int counter = 0;

			while(counter <= 256)
			{
				Serial.print(flash.readByte(counter++), HEX);
				Serial.print('.');
			}
			while(flash.busy());
			Serial.println();
		}
		if (input == 'e')
		{
			Serial.print("Erasing Flash chip ... ");
			flash.chipErase();
			while(flash.busy());
			Serial.println("DONE");
		}
		if (input == 'i')
		{
			Serial.print("DeviceID: ");
			word jedecid = flash.readDeviceId();
			Serial.println(jedecid, HEX);
		}
	}

	//check for any received packets
	if (radio.receiveDone())
	{
		Serial.print('[');
		Serial.print(radio.SENDERID, DEC);
		Serial.print("] ");
		for (byte i = 0; i < radio.DATALEN; i++)
			Serial.print((char)radio.DATA[i]);
		Serial.print("   [RX_RSSI:");
		Serial.print(radio.readRSSI());
		Serial.print("]");

		if (radio.ACKRequested())
		{
			radio.sendACK();
			Serial.print(" - ACK sent");
			delay(10);
		}
		Serial.println();
	}

	int currPeriod = millis() / TRANSMITPERIOD;
	if (currPeriod != lastPeriod)
	{
		wakeScale();
		weight = 0;
		int i = 0;
		for (i = 0; i < 5; i++) // try 5 times to get a measurement then give up
		{
			weight = getweightnow();
			if(weight >= 0) break;
		}
		Serial << "i: " << i << " weight: " << weight << endl;
		Serial.flush();
		//fill in the struct with new values
		theData.nodeId = NODEID;
		theData.uptime = millis();
		theData.weight = weight;
		theData.supplyV = readVccMv();
		theData.outTemp = radio.readTemperature(TEMP_CALIBRATION);
		theData.inTemp = 0; // TODO: Add TH02 code later
		theData.humidity = 0; // TODO: Add TH02 code later

		Serial.print("Sending struct (");
		Serial.print(sizeof(theData));
		Serial.print(" bytes) ... ");
		if (radio.sendWithRetry(GATEWAYID, (const void*)(&theData), sizeof(theData)))
			Serial.print(" ok!");
		else
			Serial.print(" nothing...");
		Serial.println();
		lastPeriod = currPeriod;

		sleep(2 * MIN); // wait 20 min before next measurement

	}
}


//########################################################################################################################
// Utility functions
//########################################################################################################################

//calculate weight
int getDigit(int d)
{
	// d=0,1,2,3
	int i = 2 * d + 0;
	int j = 2 * d + 1;

	if ( A[i] &&  A[j] && !B[i] &&  B[j] &&  C[i] &&  C[j] ) return 0; //ok
	if (!A[i] && !A[j] && !B[i] &&  B[j] && !C[i] &&  C[j] ) return 1; //ok
	if (!A[i] &&  A[j] &&  B[i] &&  B[j] &&  C[i] && !C[j] ) return 2; //ok
	if (!A[i] &&  A[j] &&  B[i] &&  B[j] && !C[i] &&  C[j] ) return 3; //ok
	if ( A[i] && !A[j] &&  B[i] &&  B[j] && !C[i] &&  C[j] ) return 4; //ok
	if ( A[i] &&  A[j] &&  B[i] && !B[j] && !C[i] &&  C[j] ) return 5; //ok
	if ( A[i] &&  A[j] &&  B[i] && !B[j] &&  C[i] &&  C[j] ) return 6; //ok
	if (!A[i] &&  A[j] && !B[i] &&  B[j] && !C[i] &&  C[j] ) return 7; //ok
	if ( A[i] &&  A[j] &&  B[i] &&  B[j] &&  C[i] &&  C[j] ) return 8; //ok
	if ( A[i] &&  A[j] &&  B[i] &&  B[j] && !C[i] &&  C[j] ) return 9; //ok
	if ( !A[i] &&  !A[j] && !B[i] &&  !B[j] &&  !C[i] &&  !C[j] ) return 0; //empty
	return -1; // Wrong combination
}

void wakeScale()
{
	pinMode(scalePower, OUTPUT);
	pinMode(scalePower, INPUT);
	//wait 8 sec for scale to become active
	sleep(8);

}

int getweightnow()
{
#if DEBUG
	lcdA = analogRead(A3);
	lcdB = analogRead(A4);
	lcdC = analogRead(A5);
	printf("Idle Read: %i, %i, %i, %i\n", lcdA, lcdB, lcdC);
#endif
	//define i as 0..8
	SAn = SBn = SCn = 0;
	lcd1 = lcd2 = lcd3 = lcd4 = lcd5 = lcd6 = lcd7 = lcd8 = 0;
	for (int i = 0; i < 8; i++)
		SA[i] = SB[i] = SC[i] = 0;
	int i = 0;


	//  //run while backplanes are active
	//  while ((lcdA!=0 || lcdB!=0 || lcdC!=0 || lcdD!=0)) {
	//run while backplanes are active
	do
	{
		delay(random(30)); // don't read so fast
		lcdA = analogRead(A3);
		lcdB = analogRead(A4);
		lcdC = analogRead(A5);

		lcd1 = digitalRead(A1);
		lcd2 = digitalRead(A2);
		lcd3 = digitalRead(3);
		lcd4 = digitalRead(4);
		lcd5 = digitalRead(5);
		lcd6 = digitalRead(6);
		lcd7 = digitalRead(7);
		lcd8 = digitalRead(8);

		//   printf("Read: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\n", lcdA, lcdB, lcdC, lcdD, lcd1, lcd2, lcd3, lcd4, lcd5, lcd6, lcd7, lcd8);

		if ((lcd1 == 0 && lcd2 == 0 && lcd3 == 0 && lcd4 == 0 && lcd5 == 0 && lcd6 == 0 && lcd7 == 0 && lcd8 == 0))
		{
#if DEBUG
			printf("Invalid Read: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\n", lcdA, lcdB, lcdC, lcd1, lcd2, lcd3, lcd4, lcd5, lcd6, lcd7, lcd8);
#endif
			continue; // ignore invalid read all zero
		}

		if (lcdA < TRESHOLD)
		{
			SA[0] += lcd1;
			SA[1] += lcd2;
			SA[2] += lcd3;
			SA[3] += lcd4;
			SA[4] += lcd5;
			SA[5] += lcd6;
			SA[6] += lcd7;
			SA[7] += lcd8;
			SAn++;
#if DEBUG
			printf("ReadA: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\n", lcdA, lcdB, lcdC, lcd1, lcd2, lcd3, lcd4, lcd5, lcd6, lcd7, lcd8);
#endif
		}
		if (lcdB < TRESHOLD)
		{
			SB[0] += lcd1;
			SB[1] += lcd2;
			SB[2] += lcd3;
			SB[3] += lcd4;
			SB[4] += lcd5;
			SB[5] += lcd6;
			SB[6] += lcd7;
			SB[7] += lcd8;
			SBn++;
#if DEBUG
			printf("ReadB: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\n", lcdA, lcdB, lcdC, lcd1, lcd2, lcd3, lcd4, lcd5, lcd6, lcd7, lcd8);
#endif
		}
		if (lcdC < TRESHOLD)
		{
			SC[0] += lcd1;
			SC[1] += lcd2;
			SC[2] += lcd3;
			SC[3] += lcd4;
			SC[4] += lcd5;
			SC[5] += lcd6;
			SC[6] += lcd7;
			SC[7] += lcd8;
			SCn++;
#if DEBUG
			printf("ReadC: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\n", lcdA, lcdB, lcdC, lcd1, lcd2, lcd3, lcd4, lcd5, lcd6, lcd7, lcd8);
#endif
		}

		i++;


		//    if ((lcd1==0 && lcd2==0 && lcd3==0 && lcd4==0 && lcd5==0 && lcd6==0 && lcd7==0 && lcd8==0) && (SA[0]+SA[1]+SA[2]+SA[3]+SA[4]+SA[5]+SA[6]+SA[7]>0)) {
		if (SAn > MEASUREMENTS && SBn > MEASUREMENTS && SCn > MEASUREMENTS )
		{

			for (int i = 0; i < 8; i++)
			{
				A[i] = (1.0 * SA[i] / SAn > 0.5);
				B[i] = (1.0 * SB[i] / SBn > 0.5);
				C[i] = (1.0 * SC[i] / SCn > 0.5);
			}
#if DEBUG
			printf(" %c   %c     %c     %c \n", A[2 - 1] ? '-' : ' ', A[4 - 1] ? '-' : ' ', A[6 - 1] ? '-' : ' ', A[8 - 1] ? '-' : ' ');
			printf("%c %c %c %c %c %c %c   %c %c\n", A[1 - 1] ? '|' : ' ', B[2 - 1] ? '|' : ' ', A[3 - 1] ? '|' : ' ', B[4 - 1] ? '|' : ' ', ' ', A[5 - 1] ? '|' : ' ', B[6 - 1] ? '|' : ' ', A[7 - 1] ? '|' : ' ', B[8 - 1] ? '|' : ' ');
			printf(" %c   %c     %c     %c \n", B[1 - 1] ? '-' : ' ', B[3 - 1] ? '-' : ' ', B[5 - 1] ? '-' : ' ', B[7 - 1] ? '-' : ' ');
			printf("%c %c %c %c %c %c %c   %c %c\n", C[1 - 1] ? '|' : ' ', C[2 - 1] ? '|' : ' ', C[3 - 1] ? '|' : ' ', C[4 - 1] ? '|' : ' ', ' ', C[5 - 1] ? '|' : ' ', C[6 - 1] ? '|' : ' ', C[7 - 1] ? '|' : ' ', C[8 - 1] ? '|' : ' ');
			printf(" %c   %c     %c  %c  %c ", ' ', ' ', ' ', ' ', ' ');
#endif
			// decoding
			int digit0 = getDigit(0);
			int digit1 = getDigit(1);
			int digit2 = getDigit(2);
			int digit3 = getDigit(3);
			if(digit0 == -1 || digit1 == -1 || digit2 == -1 || digit3 == -1)
			{
				Serial << "Wrong Digit: " << digit0 << ", " << digit1 << ", " << digit2 << ", " << digit3 << endl;
				Serial.flush();

				return -1; // maybe break instead and discard measurement
			}
			int weight = 1000.0 * digit0 + 100.0 * digit1 + 10.0 * digit2 + 1.0 * digit3;

			//#if DEBUG
			unsigned long time = millis();
			Serial << "Time:" << time << " Weight:" << weight << endl;
			Serial.flush();

			return weight;

		} //end of if
	}
	while (lcdA + lcdB + lcdC < 2700); // Backplanes are pulled low
#if DEBUG
	printf("getWeightNow() Loop terminated %d, %d, %d\n", lcdA, lcdB, lcdC);
#endif

	return -1; // should normally never get here as we stop once we have a measurement
	// while(true);
	// while((lcd1!=0 || lcd2!=0 || lcd3!=0 || lcd4!=0 || lcd5!=0 || lcd6!=0 || lcd7!=0 || lcd8!=0)); //end of while

}

//--------------------------------------------------------------------------------------------------
// Get VCC
//-------------------------------------------------------------------------------------------------
long readVcc()
{
	long result;
	// Read 1.1V reference against AVcc
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	delay(2); // Wait for Vref to settle
	ADCSRA |= _BV(ADSC); // Convert
	while (bit_is_set(ADCSRA, ADSC));
	result = ADCL;
	result |= ADCH << 8;
	result = 1126400L / result; // Back-calculate AVcc in mV
	return result;
}

// Better ReadVcc method.
// Taken from http://www.rcgroups.com/forums/showthread.php?t=1874973
int readVccMv()
{
// Read 1.1V reference against AVcc
// set the reference to Vcc and the measurement to the internal 1.1V reference
// Leonardo, Micro, Esplora, Duemilanove, Mega 2560, Mega ADK
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
// No clue, not planning to look it up
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
	ADMUX = _BV(MUX5) | _BV(MUX0);
// No clue, not planning to look it up
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
	ADMUX = _BV(MUX3) | _BV(MUX2);
// All others
#else
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
	delay(2); // Wait for Vref to settle
	ADCSRA |= _BV(ADSC); // Start conversion
	while (bit_is_set(ADCSRA, ADSC)); // measuring
	uint8_t low = ADCL; // must read ADCL first - it then locks ADCH
	uint8_t high = ADCH; // unlocks both
	long result = (high << 8) | low;
//result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
	result = scaleConst / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
	return (int)result; // Vcc in millivolts
}








