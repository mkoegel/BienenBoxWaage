#include <Streaming.h>
#include <avr/sleep.h>
#include <avr/wdt.h>


#define DEBUG 0
#define MIN 60
#define ONESHEELD // comment if not pressent

#ifdef ONESHEELD
#include <OneSheeld.h>
#endif

int           lcdA=0,lcdB=0,lcdC=0,lcdD=0;
int           scalePower = 2;
bool          lcd1,lcd2,lcd3,lcd4,lcd5,lcd6,lcd7,lcd8,lcd9;
unsigned int  SAn, SBn, SCn, SDn;
unsigned int  SA[9], SB[9], SC[9], SD[9];
bool          A[9], B[9], C[9], D[9];

unsigned long button_time = 0;  
unsigned long last_button_time = 0;
int intPin = 1; //PIN 3 som interrupt
unsigned int _t;


#define myNodeID 25       // RF12 node ID in the range 1-30
#define network 210       // RF12 Network group
#define freq RF12_868MHZ  // Frequency of RFM12B module

#define USE_ACK // Enable ACKs, comment out to disable
#define RETRY_PERIOD 1 // How soon to retry (in seconds) if ACK didn't come in
#define RETRY_LIMIT 5 // Maximum number of times to retry
#define ACK_TIME 200 // Number of milliseconds to wait for an ack

#define TRESHOLD 200
#define MEASUREMENTS 7

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


// Function that printf and related will use to print
int serial_putchar(char c, FILE* f) {
  if (c == '\n') serial_putchar('\r', f);
#ifdef ONESHEELD
  Terminal.write(c);
#else
  Serial.write(c);
#endif
  return 0;
}

FILE serial_stdout;


//########################################################################################################################
//Data Structure to be sent
//########################################################################################################################

typedef struct {
  int weightNow;    // Switch state
  int supplyV;      // Supply voltage
} 
Payload;

Payload tinytx;

//########################################################################################################################

//
void sleep(int seconds)
{
  // sleep bit patterns:
  //  1 second:  0b000110
  //  2 seconds: 0b000111
  //  4 seconds: 0b100000
  //  8 seconds: 0b100001

  int i;
  for (i = 0; i < seconds/8; i++)
  {  
    myWatchdogEnable (0b100001);  // 8 seconds
  }
  seconds = seconds % 8;

  for (i = 0; i < seconds/4; i++)
  {  
    myWatchdogEnable (0b100000);  // 4 seconds
  }
  seconds = seconds % 4;

  for (i = 0; i < seconds/2; i++)
  {  
    myWatchdogEnable (0b000111);  // 2 seconds
  }
  seconds = seconds % 2;

  for (i = 0; i < seconds; i++)
  {  
    myWatchdogEnable (0b000110);  // 1 second
  }

}


//calculate weight
int getDigit(int d)
{
  // d=0,1,2,3
  int i = 2*d+0;
  int j = 2*d+1;

  if ( A[i] &&  A[j] && !B[i] &&  B[j] &&  C[i] &&  C[j] &&  D[i]) return 0; //ok
  if (!A[i] && !A[j] && !B[i] &&  B[j] && !C[i] &&  C[j] && !D[i]) return 1; //ok
  if (!A[i] &&  A[j] &&  B[i] &&  B[j] &&  C[i] && !C[j] &&  D[i]) return 2; //ok
  if (!A[i] &&  A[j] &&  B[i] &&  B[j] && !C[i] &&  C[j] &&  D[i]) return 3; //ok
  if ( A[i] && !A[j] &&  B[i] &&  B[j] && !C[i] &&  C[j] && !D[i]) return 4; //ok
  if ( A[i] &&  A[j] &&  B[i] && !B[j] && !C[i] &&  C[j] &&  D[i]) return 5; //ok
  if ( A[i] &&  A[j] &&  B[i] && !B[j] &&  C[i] &&  C[j] &&  D[i]) return 6; //ok
  if (!A[i] &&  A[j] && !B[i] &&  B[j] && !C[i] &&  C[j] && !D[i]) return 7; //ok
  if ( A[i] &&  A[j] &&  B[i] &&  B[j] &&  C[i] &&  C[j] &&  D[i]) return 8; //ok
  if ( A[i] &&  A[j] &&  B[i] &&  B[j] && !C[i] &&  C[j] &&  D[i]) return 9; //ok
  if ( !A[i] &&  !A[j] && !B[i] &&  !B[j] &&  !C[i] &&  !C[j] &&  !D[i]) return 0; //empty
  return -1; // Wrong combination
}


void setup() {
  pinMode(scalePower, INPUT);
  digitalWrite(scalePower, LOW);

#ifdef ONESHEELD
  /* Start communication. */
  OneSheeld.begin();
  /* Save any previous logged values. */
  Logger.stop();
  Logger.start("BienenBoxGewicht");
#else
  Serial.begin(57600);  //turn on serial communication
#endif  //  attachInterrupt(1, blink, CHANGE); //0=2, 1=3
  //  digitalWrite(3, LOW);

  //#if DEBUG
  // Set up stdout for printf
  fdev_setup_stream(&serial_stdout, serial_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &serial_stdout;
  printf("Initialised");
  Serial << "Initialized..." << endl;



  //#endif

  //rf12_initialize(myNodeID, freq, network);
  //rf12_control(0xC040); // set low-battery level to 2.2V i.s.o. 3.1V

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
}

void blink(){
}

bool on = false;

void scaleOn(){
  pinMode(scalePower, OUTPUT);
  pinMode(scalePower, INPUT);
  //wait 8 sec for scale to become active
  sleep(8);

}

void scaleOff(){
}

int weight = 0;
void loop()
{
  scaleOn();
  weight = 0;
  int i = 0;
  for (i=0; i<5; i++){ // try 5 times to get a measurement then give up
    weight = getweightnow();
    if(weight >= 0) break;
  }
  printf("i: %d, weight: %d\n", i, weight);
  /* Send an email. */

  scaleOff();
  // sleep(30); // wait 30 sec before next measurement
  sleep(20*MIN);
}


int getweightnow() {
  lcdA = analogRead(A0);
  lcdB = analogRead(A1);
  lcdC = analogRead(A2);
  lcdD = analogRead(A3);
#if DEBUG
  printf("Idle Read: %i, %i, %i, %i\n", lcdA, lcdB, lcdC, lcdD);
  printf("Stepped on scale\n");
#endif
  //define i as 0..8
  SAn = SBn = SCn = SDn = 0;
  lcd1=lcd2=lcd3=lcd4=lcd5=lcd6=lcd7=lcd8=lcd9=0;
  for (int i=0; i<9; i++)
    SA[i] = SB[i] = SC[i] = SD[i] = 0;
  int i=0;


  //  //run while backplanes are active
  //  while ((lcdA!=0 || lcdB!=0 || lcdC!=0 || lcdD!=0)) {
  //run while backplanes are active
  do  {
    delay(random(30)); // don't read so fast
    lcdA = analogRead(A0);
    lcdB = analogRead(A1);
    lcdC = analogRead(A2);
    lcdD = analogRead(A3);

    lcd1 = digitalRead(A4);
    lcd2 = digitalRead(A5);
    lcd3 = digitalRead(3);
    lcd4 = digitalRead(4);
    lcd5 = digitalRead(5);
    lcd6 = digitalRead(6);
    lcd7 = digitalRead(7);
    lcd8 = digitalRead(8);
    lcd9 = digitalRead(9);

    //   printf("Read: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\n", lcdA, lcdB, lcdC, lcdD, lcd1, lcd2, lcd3, lcd4, lcd5, lcd6, lcd7, lcd8, lcd9);

    if ((lcd1==0 && lcd2==0 && lcd3==0 && lcd4==0 && lcd5==0 && lcd6==0 && lcd7==0 && lcd8==0 && lcd9==0)) {
#if DEBUG
      printf("Invalid Read: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\n", lcdA, lcdB, lcdC, lcdD, lcd1, lcd2, lcd3, lcd4, lcd5, lcd6, lcd7, lcd8, lcd9);
#endif
      continue; // ignore invalid read all zero
    }

    if (lcdA<TRESHOLD)
    {
      SA[0] += lcd1;
      SA[1] += lcd2;
      SA[2] += lcd3;
      SA[3] += lcd4;
      SA[4] += lcd5;
      SA[5] += lcd6;
      SA[6] += lcd7;
      SA[7] += lcd8;
      SA[8] += lcd9;
      SAn++;
#if DEBUG
      printf("ReadA: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\n", lcdA, lcdB, lcdC, lcdD, lcd1, lcd2, lcd3, lcd4, lcd5, lcd6, lcd7, lcd8, lcd9);
#endif
    }
    if (lcdB<TRESHOLD)
    {
      SB[0] += lcd1;
      SB[1] += lcd2;
      SB[2] += lcd3;
      SB[3] += lcd4;
      SB[4] += lcd5;
      SB[5] += lcd6;
      SB[6] += lcd7;
      SB[7] += lcd8;
      SB[8] += lcd9;
      SBn++;
#if DEBUG
      printf("ReadB: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\n", lcdA, lcdB, lcdC, lcdD, lcd1, lcd2, lcd3, lcd4, lcd5, lcd6, lcd7, lcd8, lcd9);
#endif
    }
    if (lcdC<TRESHOLD)
    {
      SC[0] += lcd1;
      SC[1] += lcd2;
      SC[2] += lcd3;
      SC[3] += lcd4;
      SC[4] += lcd5;
      SC[5] += lcd6;
      SC[6] += lcd7;
      SC[7] += lcd8;
      SC[8] += lcd9;
      SCn++;
#if DEBUG
      printf("ReadC: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\n", lcdA, lcdB, lcdC, lcdD, lcd1, lcd2, lcd3, lcd4, lcd5, lcd6, lcd7, lcd8, lcd9);
#endif
    }
    if (lcdD<TRESHOLD)
    {
      SD[0] += lcd1;
      SD[1] += lcd2;
      SD[2] += lcd3;
      SD[3] += lcd4;
      SD[4] += lcd5;
      SD[5] += lcd6;
      SD[6] += lcd7;
      SD[7] += lcd8;
      SD[8] += lcd9;
      SDn++;
#if DEBUG
      printf("ReadD: %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i\n", lcdA, lcdB, lcdC, lcdD, lcd1, lcd2, lcd3, lcd4, lcd5, lcd6, lcd7, lcd8, lcd9);
#endif
    }

    i++;  


    //    if ((lcd1==0 && lcd2==0 && lcd3==0 && lcd4==0 && lcd5==0 && lcd6==0 && lcd7==0 && lcd8==0 && lcd9==0) && (SA[0]+SA[1]+SA[2]+SA[3]+SA[4]+SA[5]+SA[6]+SA[7]+SA[8]>0)) {
    if (SAn > MEASUREMENTS && SBn > MEASUREMENTS && SCn > MEASUREMENTS && SDn > MEASUREMENTS) {

      for (int i=0;i<9;i++)
      {
        A[i] = (1.0 * SA[i] / SAn >0.5);
        B[i] = (1.0 * SB[i] / SBn >0.5);
        C[i] = (1.0 * SC[i] / SCn >0.5);
        D[i] = (1.0 * SD[i] / SDn >0.5);
      }
#if DEBUG
      printf(" %c   %c     %c     %c \n", A[2-1]?'-':' ', A[4-1]?'-':' ', A[6-1]?'-':' ', A[8-1]?'-':' ');
      printf("%c %c %c %c %c %c %c   %c %c\n", A[1-1]?'|':' ', B[2-1]?'|':' ', A[3-1]?'|':' ', B[4-1]?'|':' ', D[4-1]?'*':' ', A[5-1]?'|':' ', B[6-1]?'|':' ', A[7-1]?'|':' ', B[8-1]?'|':' ');
      printf(" %c   %c     %c     %c \n", B[1-1]?'-':' ', B[3-1]?'-':' ', B[5-1]?'-':' ', B[7-1]?'-':' ');
      printf("%c %c %c %c %c %c %c   %c %c\n", C[1-1]?'|':' ', C[2-1]?'|':' ', C[3-1]?'|':' ', C[4-1]?'|':' ', D[4-1]?'*':' ', C[5-1]?'|':' ', C[6-1]?'|':' ', C[7-1]?'|':' ', C[8-1]?'|':' ');
      printf(" %c   %c     %c  %c  %c ", D[1-1]?'-':' ', D[3-1]?'-':' ', D[5-1]?'-':' ', D[6-1]?'*':' ', D[7-1]?'-':' ');
      if(A[8]==1) printf(" kg\n");
      if(B[8]==1) printf(" lb\n");
      if(C[8]==1) printf(" st\n");
#endif
      // decoding
      int digit0 = getDigit(0);
      int digit1 = getDigit(1);
      int digit2 = getDigit(2);
      int digit3 = getDigit(3);
      if(digit0 == -1 || digit1 == -1 || digit2 == -1 || digit3 == -1){
        Serial << "Wrong Digit: " << digit0 << ", "<< digit1 << ", "<< digit2 << ", "<< digit3 << endl;
        return -1; // maybe break instead and discard measurement
      }
      int weight = 1000.0*digit0 + 100.0*digit1 + 10.0*digit2 + 1.0*digit3;

      //#if DEBUG
      unsigned long time = millis();
      Serial << "Time:" << time << " Weight:" << weight << endl;
#ifdef ONESHEELD
      Logger.add("Time", time);
      Logger.add("Weight", weight);
      Logger.log();
      LCD.begin();
      LCD.print(weight);
      String message = String(weight);
      message += ", ";
      message += readVcc();
      char charBuf[message.length() + 1];
      message.toCharArray(charBuf, message.length() + 1);
      Email.send("<yourrobot>@robot.zapier.com","BeeScale", charBuf);
#endif

      return weight;

    } //end of if
  } 
  while (lcdA + lcdB + lcdC + lcdD < 2000); // Backplanes are pulled low
#if DEBUG
  printf("getWeightNow() Loop terminated %d, %d, %d, %d\n", lcdA, lcdB, lcdC, lcdD);
#endif

  return -1; // should normally never get here as we stop once we have a measurement
  // while(true);
  // while((lcd1!=0 || lcd2!=0 || lcd3!=0 || lcd4!=0 || lcd5!=0 || lcd6!=0 || lcd7!=0 || lcd8!=0 || lcd9!=0)); //end of while

}

//--------------------------------------------------------------------------------------------------
// Get VCC
//-------------------------------------------------------------------------------------------------
long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}






