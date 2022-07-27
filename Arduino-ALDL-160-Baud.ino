#include <Arduino.h>

#define ALDL_PIN 3

#define SERIAL_DEBUG

// Number of bytes per ALDL frame
//#define ALDL_FRAME_BUF_SIZE 50
//TODO: Make this adjustable/automatic
#define ALDL_FRAME_BUF_SIZE 20

// Number of bits per ALDL byte
#define ALDL_BYTE_SIZE 8

// Minimum microsecs for a complete packet
#define ALDL_MIN_PACKET_TIME 6000

// Maximum microsecs for a complete packet
#define ALDL_MAX_PACKET_TIME 6400

// Approx "start bit" max microsecs when transmitting a "0"
#define ALDL_0_MIN_LENGTH 100
#define ALDL_0_MAX_LENGTH 600

// Approx "start bit" min microsecs when transmitting a "1"
#define ALDL_1_MIN_LENGTH 4000
#define ALDL_1_MAX_LENGTH 6000

int frame[ALDL_FRAME_BUF_SIZE];

volatile unsigned int byteIndex = 0;
volatile unsigned int bitIndex = 0;
volatile unsigned int bitTime = 0;
volatile unsigned int curBit = 0;
volatile unsigned long curTime = micros();
volatile unsigned long prevTime = micros();

unsigned long interruptCount = 0;
int modCount = 0;
int lastModCount = 0;
unsigned long lastPrint = 0;

void setup() {
    // Clear the frame buffer
    for (int i = 0; i < ALDL_FRAME_BUF_SIZE; i++) {
        frame[i] = (byte)0x00;
    }

    pinMode(ALDL_PIN, INPUT);
    attachInterrupt( digitalPinToInterrupt( ALDL_PIN ) , interrupt, CHANGE);
    Serial.begin( 115200 );
    Serial.write( "Begin data stream...\r\n" );
}

void interrupt() {
  interruptCount++;
  readBit();
}

void readBit() {
  curTime = micros();
  bitTime = curTime - prevTime;
  if( !digitalRead( 3 ) ){
    prevTime = micros(); 
  }
  
  //Serial.println( bitTime );

  if (bitTime <= ALDL_0_MAX_LENGTH && bitTime >= ALDL_0_MIN_LENGTH) {
    nextBit( 0 );
  } else if (bitTime <= ALDL_1_MAX_LENGTH && bitTime >= ALDL_1_MIN_LENGTH) {
    nextBit( 1 );
  } else if (bitTime > ALDL_1_MAX_LENGTH ) {
    //nextBit( 0 );
  }
}

volatile uint16_t thisByte = 0;
int flagCount = 0;
int packetIndex = 0;
uint8_t packet[30];

void dumpPacket(){
  noInterrupts();
  Serial.print( "\r\nBegin Packet Dump: " );
  Serial.println( byteIndex );
  for( int i=0; i < byteIndex % 9; i++  ){
    Serial.write( packet[i] );
  }
  Serial.flush();
  interrupts();
}

void beginFlag( int bit ){
	if( bit == 1 ){
		flagCount++;
	}
	else{
		flagCount = 0;
	}
	if( flagCount == 9 ){
    dumpPacket();
		packetIndex = 0;
		byteIndex = 0;
	}
}

void nextBit( int latestBit ){
  beginFlag( latestBit );
	thisByte = thisByte << 1;
	thisByte = thisByte | latestBit;
	byteIndex++;
	if( byteIndex % 9 == 0 ){
		packet[ byteIndex % 9 ] = thisByte;
		thisByte = 0;
	}
}	

void loop(){
  /*
  if( millis() - lastPrint > 1000 ){
    Serial.println( interruptCount );
    lastPrint = millis();
  }
  */
  //Serial.println( digitalRead( 3 ) );
  //Serial.flush();
}

