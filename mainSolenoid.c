#include <p18cxxx.h>
#include <delays.h>
#include <adc.h>

// Password setup
#define PASSNUM 4
int password[PASSNUM] = {0x1, 0xb, 0x5, 0x0};

// General chip setup
#pragma config WDT = OFF
#pragma config OSC = HS
#pragma config LVP = OFF

// Seven segment light pins
// Split it up so everything was next to each other physically
/*
#define L1 PORTDbits.RD0
#define L2 PORTDbits.RD1
#define L3 PORTDbits.RD2
#define L4 PORTDbits.RD3
#define L5 PORTCbits.RC4
#define L6 PORTCbits.RC5
#define L7 PORTCbits.RC6
#define L8 PORTCbits.RC7
#define L9 PORTDbits.RD6
*/

/*  Seven segment layout
     -- L8 is a period 
     -- L9 is to switch the second display to open
     L1
   L2  L3
     L4  
   L5  L6
     L7

   L8   L9
*/


// Prototypes
void writeNum(int);
void InterruptHandlerHigh (void);

union
{
  struct
  {
    unsigned Timeout:1;         //flag to indicate a TMR0 timeout
    unsigned None:7;
  } Bit;
  unsigned char Byte;
} Flags;

// Global variables
int passDigit = 0;
int lockState = 0;
int analogInput = 0;
int analogPrevious = 0x1;
int hold = 0;
int lighton = 1; 
int acceptedNum = 0;
int secondRun = 0;

void main (void)
{
  int count = 0;
  int analogTemp = 0;  	// A temporary place holder is needed
						// since ADC will screw with the number
						// you are assigning to, and we could
						// switch threads during that time.
  TRISB = 0;  // Output for relay
  TRISD = 0;  // Output for seven segment display
  TRISC = 0;  // More output for seven segment display
  TRISAbits.TRISA0 = 1; // Analog input
  writeNum(0);

  // Enable timer interrupt
  Flags.Byte = 0;
  INTCON = 0x20;                //disable global and enable TMR0 interrupt
  INTCON2 = 0x84;               //TMR0 high priority
  RCONbits.IPEN = 1;            //enable priority levels
  TMR0H = 0;                    //clear timer
  TMR0L = 0;                    //clear timer
  T0CON = 0x84;                 //set up timer0 - prescaler 1:8
  INTCONbits.GIEH = 1;          //enable interrupts
		
  while(1){
    OpenADC(ADC_FOSC_8 & ADC_RIGHT_JUST & ADC_0_TAD,
       ADC_CH0 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS,
	   0b1011);
    SetChanADC(ADC_CH0);
    ConvertADC(); // Start conversion
    while( BusyADC() ); // Wait for ADC conversion
    analogTemp = ReadADC(); // Read result and put in temp
    CloseADC();
    analogInput = analogTemp >> 6; // Get only the most significant bits

	// If we didn't change values since last time, we are in a holding state
	if( analogPrevious == analogInput && !hold) hold = 1;
    else if (analogPrevious != analogInput){
 	    acceptedNum = 0;
		hold = 0;
    }
	
	// Turn off potential for activating the relay if we aren't on 0
    if(analogInput) PORTB = 0;


	// If we are allowing the second relay activation and we are on 0
    if(secondRun && !analogInput) {
      PORTB = 0xFF;
      secondRun = 0;
	  passDigit = 0;
	}

    analogPrevious = analogInput;
    writeNum(analogInput);
  }
}


#pragma code InterruptVectorHigh = 0x08
void
InterruptVectorHigh (void)
{
  _asm
    goto InterruptHandlerHigh //jump to interrupt routine
  _endasm
}

#pragma code
#pragma interrupt InterruptHandlerHigh

void
InterruptHandlerHigh ()
{
  if (INTCONbits.TMR0IF)
    {                                   //check for TMR0 overflow
      INTCONbits.TMR0IF = 0;            //clear interrupt flag
      Flags.Bit.Timeout = 1;            //indicate timeout
      if(hold && hold < 10) hold ++;
	  
	  // We have waited on the number long enough
      if(hold > 4 && !acceptedNum) {
		acceptedNum = 1;
		if(passDigit < PASSNUM) {
			if(password[passDigit] == analogInput) passDigit ++;
			else passDigit = 0;
		}
      }
  
	  if(passDigit >= PASSNUM) {
		if( !analogInput ) PORTB = 0xFF;
		else secondRun = 1;
	  }
	}      
}


// Brute force coding FTW!
// Originally had a much more elegant solution, but
// my original dying seven segment display tricked me into
// thinking it wasn't working
void writeNum(int x){
  if(!acceptedNum){
	switch(x){
		case 0:
			PORTD = 0b00000111;
			PORTC = 0b01110000;				
			break;
		case 1:
			PORTD = 0b00000100;
			PORTC = 0b00100000;					
			break;
		case 2:			
			PORTD = 0b00001101;
			PORTC = 0b01010000;	
			break;
		case 3:
			PORTD = 0b00001101;
			PORTC = 0b01100000;				
			break;
		case 4:
			PORTD = 0b00001110;
			PORTC = 0b00100000;
			break;
		case 5:
			PORTD = 0b00001011;
			PORTC = 0b01100000;
			break;
		case 6:
			PORTD = 0b00001011;
			PORTC = 0b01110000;
			break;
		case 7:
			PORTD = 0b00000101;
			PORTC = 0b00100000;					
			break;
		case 8:
			PORTD = 0b00001111;
			PORTC = 0b01110000;
			break;
		case 9:
			PORTD = 0b00001111;
			PORTC = 0b00100000;
			break;
		case 10: //A
			PORTD = 0b00001111;
			PORTC = 0b00110000;
			break;			
		case 11: //B
			PORTD = 0b00001010;
			PORTC = 0b01110000;
			break;		
		case 12: //C
			PORTD = 0b00000011;
			PORTC = 0b01010000;				
			break;
		case 13: // D
			PORTD = 0b00001100;
			PORTC = 0b01110000;
			break;
		case 14: // E
			PORTD = 0b00001011;
			PORTC = 0b01010000;
			break;
		case 15: // F
			PORTD = 0b00001011;
			PORTC = 0b00010000;
			break;
		case 99: // Debug pattern
			PORTD = 0b00001001;
			PORTC = 0b01000000;	
			break;
	}
  }else{
	switch(x){
		case 0:
			PORTD = 0b00000111;
			PORTC = 0b11110000;				
			break;
		case 1:
			PORTD = 0b00000100;
			PORTC = 0b10100000;					
			break;
		case 2:			
			PORTD = 0b00001101;
			PORTC = 0b11010000;	
			break;
		case 3:
			PORTD = 0b00001101;
			PORTC = 0b11100000;				
			break;
		case 4:
			PORTD = 0b00001110;
			PORTC = 0b10100000;
			break;
		case 5:
			PORTD = 0b00001011;
			PORTC = 0b11100000;
			break;
		case 6:
			PORTD = 0b00001011;
			PORTC = 0b11110000;
			break;
		case 7:
			PORTD = 0b00000101;
			PORTC = 0b10100000;					
			break;
		case 8:
			PORTD = 0b00001111;
			PORTC = 0b11110000;
			break;
		case 9:
			PORTD = 0b00001111;
			PORTC = 0b10100000;
			break;
		case 10: //A
			PORTD = 0b00001111;
			PORTC = 0b10110000;
			break;			
		case 11: //B
			PORTD = 0b00001010;
			PORTC = 0b11110000;
			break;		
		case 12: //C
			PORTD = 0b00000011;
			PORTC = 0b11010000;				
			break;
		case 13: // D
			PORTD = 0b00001100;
			PORTC = 0b11110000;
			break;
		case 14: // E
			PORTD = 0b00001011;
			PORTC = 0b11010000;
			break;
		case 15: // F
			PORTD = 0b00001011;
			PORTC = 0b10010000;
			break;	
		case 99: // Debug pattern
			PORTD = 0b00001001;
			PORTC = 0b11000000;	
			break;

	}
  }
}


