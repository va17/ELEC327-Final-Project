/////////MAIN PROJECT CODE
#include <msp430.h>
#include <stdint.h>

//mode variable
unsigned int running = 1; // 1 if running and operating ledstring, 0 if stopped and reading LCD

//LED SPI communication
uint8_t buffer[128];
unsigned int bufferCount = 0; //the index of the next byte to be sent

//timekeeping variables
unsigned int last_step = 0; //cycle count (VLO) of last interrupt
unsigned int time = 0; //cycle count (VLO) of current time
unsigned int period = 0; //cycle count since last step
unsigned int oldperiod = 0xFFFF;
unsigned int seconds_since = 0; //number of TA0 interrupts since last step

//run variables
unsigned int end = 0; //seconds this run has lasted
unsigned int steps = 0; //number of steps

///LED string cycle variables
int eq=0; //color variable
int led = 0;
int dir = 1; //whether pattern is rising of falling (1 = rising, -1 = falling)
int eqcount = 0;

//The Pins needed for LCD communication
#define RS BIT1 //of port 2!!! (Register Select)
#define EN BIT6 //(ENABLE)
#define PLUG BIT0  //plugin causes interrupts on falling edge
//DATA BUSES
#define DB4 BIT1
#define DB5 BIT2
#define DB6 BIT3
#define DB7 BIT4
unsigned int DB[] = {BIT1,BIT2,BIT3,BIT4}; //DB4, DB5, DB6, DB7;

///LCD COMMAND FUNCTIONS
void waitlcd(unsigned int x); //waits 2*x DCO cycles for LCD to process instructions
void prints(char *s);              //prints a string to the LCD
void lcdsend(unsigned char data);  //send either command or data, depending on RS, (0=command; 1=data)
void itoa(long unsigned int value, char* result, int base); ///integer to string conversion funtion that is part of the standard C libraries. We found it online at http://e2e.ti.com/support/microcontrollers/msp430/f/166/t/291574

unsigned int wdtCount = 0; //counts WDT interrupts up to 2 seconds


void main(void) {
    WDTCTL = WDTPW | WDTHOLD;    // Stop watchdog timer
    WDTCTL = WDTPW+WDTTMSEL+WDTCNTCL+WDTSSEL+WDTIS1; //interval mode with ~23 cycles/second
	BCSCTL3 |= LFXT1S_2; //select VLOCLK (12kHz) for the ACLK

	if (CALBC1_16MHZ == 0xFF || CALDCO_16MHZ == 0xff)
		while(1); // Erased calibration data? Trap!


	BCSCTL1 = CALBC1_16MHZ; // Set the DCO to 16 MHz
	DCOCTL = CALDCO_16MHZ; // And load calibration data

	//LCD interrupt pin
	P1IES = PLUG;
	P1REN = PLUG;
	P1OUT = PLUG; //pullup resistor
	P1IE = PLUG;

	//set all LCD pins to simple input/output
	P1SEL = 0;
	P2SEL = 0;
	P1SEL2 = 0;
	P2SEL2 = 0;

	P1DIR = DB4+DB5+DB6+DB7+EN;
	P2DIR = RS;

	///used for testing PCB connections
	//P1OUT |= DB4+DB5+DB6+DB7+EN;
	//P2OUT |= RS;

	//////////////REGULAR CODE SETUP
    //FSR interupt pin
    P2REN &= ~BIT0; //no pullup resistor
    P2DIR &= ~BIT0; //input
    P2IES = 0x00; //rising edge
    P2IE = BIT0;  //enable interrupt

    //set up TA0 for LED second counting and timing
    TA0CTL = TASSEL_1 + MC_1; //select ACLK; up mode
    TA0CCR0 = 12000; //1 second
    TA0CCTL0 = CCIE; // enable interrupts

    //set up TA1 for LEDs
    TA1CTL = TASSEL_1 + MC_0; //select ACLK; stop mode
    TA1CCTL0 = CCIE; // enable interrupts

    //configure USCI_B0 for LED string
	UCB0CTL1 |= UCSWRST; // prepare to initialize
	UCB0CTL0 |= UCMSB + UCMST + UCSYNC; //MSB first, master mode, synchronous mode
	UCB0CTL1 |= UCSSEL_2; // use the SMCLK

	//P1.5 is the clock; P1.7 is MOSI
	P1DIR |= BIT5 + BIT7; //set P1.5 and 1.7 to output
	P1SEL |= BIT5 + BIT7; //select USCI_B0 for these pins
	P1SEL2 |= BIT5 + BIT7;
	int i;
	//start bits
	for (i = 0; i<4; i++)
		buffer[i] = 0x00;
	for (;i<124;i+=4){
		buffer[i] = 0xE0; //off
		buffer[i+1] = 0x00; //blue
		buffer[i+2] = 0x00; //green
		buffer[i+3] = 0x00; //red
	}
	//end bits
	for (;i<128;i++)
		buffer[i] = 0xFF;

	UCB0CTL1 &= ~UCSWRST; // ready to go!
	IE2 |= UCB0TXIE; // enable transmit interrupts


    __bis_SR_register(LPM3_bits + GIE);
    while (1){
    	//////regular running code
    	if(running){
			if((steps>1) & (seconds_since<4)){  //if we've taken a step recently
				oldperiod=period; //save the old period
				period = 12000*seconds_since + time - last_step; //calculate new period
				if(period>500){ //if this is an actual step, initiate the cycle
					TA1CCR0 = period/59;
					buffer[4] = 0xE8; //turn new one on. (forewards)
					buffer[5] = 0; //blue
					buffer[6] = eq; //green
					buffer[7] = 255; //red

					buffer[120] = 0xE8; //turn new one on. (backwards)
					buffer[121] = eq; //blue
					buffer[122] = 255-eq; //green
					buffer[123] = eq; //red

					buffer[4+4*led] = 0xE0; //turn old one off.
					buffer[120-4*led] = 0xE0; //turn old one on. (backwards)

					eqcount = 0;
					led = 0;
					dir = 1;
					IE2 |= UCB0TXIE; // enable transmit interrupts
					TA1CTL = TASSEL_1 + MC_1; //select ACLK; up mode
				}
			}
			//for calculating the next period
			last_step = time;
			seconds_since = 0;

			__bis_SR_register(LPM3_bits);
    	}
    	/////LCD
    	else{
    		//Initialize the interface with the LCD
			waitlcd(3200); //wait
			P2OUT &=~RS;  //sending command
			lcdsend(0x33);
			P2OUT &=~RS;  //sending command
			lcdsend(0x32);
			P2OUT &=~RS;  //sending command
			lcdsend(0x28);   //set data length 4 bit; 2 line
			P2OUT &=~RS;  //sending command
			lcdsend(0x08);   // set display off
			P2OUT &=~RS;  //sending command
			lcdsend(0x01); // clear lcd
			P2OUT &=~RS;  //sending command
			waitlcd(0xAF00); //wait
			P2OUT &=~RS;  //sending command
			lcdsend(0x06);  // cursor shift direction
			P2OUT &=~RS;  //sending command
			lcdsend(0x0F);	//display on, cursor on, blink on
			P2OUT &=~RS;  //sending command
			lcdsend(0x80);  //start of first line
			P2OUT |= RS; //sending data
			prints("Steps: ");
			//convert steps to string and print it
			char stepstring[5];
			itoa(steps,stepstring,10);
			prints(stepstring);
			P2OUT &= ~RS; //sending command
			lcdsend(0xC0); //second line
			P2OUT |= RS; //sending data
			//convert time spent running to minutes:seconds string and print it
			int seconds = end%60;
			int minutes = end/60;
			prints("Runtime: ");
			char minstring[5];
			itoa(minutes,minstring,10);
			prints(minstring);
			prints(":");
			char secstring[5];
			itoa(seconds,secstring,10);
			prints(secstring);
			//reset the variables for the next run
			end = 0;
			steps = 0;
			P1IES = 0x00; //interrupt on LCD removal
			P1IE = PLUG;
			__bis_SR_register(LPM3_bits); //enter LPM3
    	}
    }
}

#pragma vector=PORT2_VECTOR
__interrupt void FSR(void){
	P2IFG = 0x00;
	if(TA0R-time>2000){ //if it's long enough to be a step, capture this step and its time and enter main loop
		time = TA0R+1;
		steps++;
		__bic_SR_register_on_exit(CPUOFF);  //return function to main()
	}
}


#pragma vector=TIMER1_A0_VECTOR
__interrupt void ledswitch(void)
{
	//if we're at the top, switch the direction
	if (led==29)
		dir = -1;
	buffer[4+4*led] = 0xE0; //turn old one off.
	buffer[120-4*led] = 0xE0; //turn old one on. (backwards)
	led += dir;
	eqcount++; //segment of the period we are now in (0-58)

	//if we're at the end of the cycle, stop TA1 and reset the LED cycle variables
	if (led<0){
		TA1CTL = TASSEL_1 + MC_0; //select ACLK; stop mode
		dir = 1;
		led=0;
		eqcount=0;
	}
	//otherwise, calculate the color and turn the next led on
	// NOTE: the color is calculated so that it changes each LED, moving smoothly from the old color to the new one.
	else{
		buffer[4+4*led] = 0xE8; //turn new one on. (forewards)
		buffer[120-4*led] = 0xE8; //turn new one on. (backwards)
		eq = (period/4500*eqcount + oldperiod/4500*(58-eqcount)); //color variable (0-255) scaled across 1 step period. higher eq = slower running speed.
		if (eq>255) eq = 255;
		else if (eq<0) eq = 0;

		//slow --> fast
		//yellow --> red
		buffer[5+4*led] = 0; //blue
		buffer[6+4*led] = eq; //green
		buffer[7+4*led] = 255; //red

		//purple --> green
		buffer[121-4*led] = eq; //blue
		buffer[122-4*led] = 255-eq; //green
		buffer[123-4*led] = eq; //red
	}
	IE2 |= UCB0TXIE; // enable transmit interrupts
}


#pragma vector=USCIAB0TX_VECTOR
__interrupt void transmit_next(void){
	//unless we're done, move the next byte into the buffer
	if (bufferCount==128){
		bufferCount=0;
		IE2 &= ~UCB0TXIE; // disable transmit interrupts; UCB0 will stop using SMCLK and automatically return to LMP3 powering
	}
	UCB0TXBUF = buffer[bufferCount];
	bufferCount++;
}






///////USED FOR LCD
//wait for x cycles
void waitlcd(unsigned int x)
{
	CCR0 = (TAR + x+x); // set interrupt for 2*x cycles from now
	TA0CCTL0 |= CCIE; // enable interrupts
	//
	if(TA0CCTL0 & CCIE){
		__bic_SR_register(SCG1); //make sure SMCLK is on for TA0
		__bis_SR_register(CPUOFF); // wait
	}
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TA0_0(void)
{
	if(running){ //count seconds; TA0R used for step periods
		end++;
		seconds_since++;
	}
	else{ //done waiting for LCD commands, return function to main
		TA0CCTL0 &= ~CCIE;
		__bic_SR_register_on_exit(CPUOFF);  //return function to main()
	}
}


void prints(char *s)
  {
  	P2OUT |= RS; //sending data
    while (*s)
    {
	 lcdsend(*s);
	 s++;
    }
  }



void lcdsend(unsigned char data)
{
	waitlcd(1600);
	P1OUT &= ~(DB4+DB5+DB6+DB7); //clear data busses
	//the high bits
	int i;
	for (i=0; i<4; i++){
		if((data>>(i+4))&0x01)
			P1OUT |= DB[i];
	}
	P1OUT  |=EN;
	P1OUT  &=~EN;
	waitlcd(1600);

	//send the low bits
	P1OUT &= ~(DB4+DB5+DB6+DB7); //clear data busses
	for (i=0; i<4; i++){
		if((data>>(i))&0x01)
			P1OUT |= DB[i];
	}
	P1OUT  |=EN;
	P1OUT  &=~EN;
}

#pragma vector=PORT1_VECTOR
__interrupt void P1(void){
	P1IFG = 0x00; //clear flags
	if(P1IES & PLUG){ //if we just plugged in
		IE1 |= WDTIE;// enable interrupts for the WDT
		P1IE = 0x00; //disable LCD interrupt
		running = 0;
		//reconfigure TA0 for LCD waiting
		TA0CTL = TASSEL_2 + MC_2; //select SMCLK; continuous mode
		TA0CCTL0 &= ~CCIE; // enable interrupts


		//turn everything but LCD off
		TA1CTL = TASSEL_1 + MC_0; // stop TA1
		P2IE = 0; //disable FSR interrupts
		buffer[4+4*led] = 0xE0; //turn old one off.
		buffer[120-4*led] = 0xE0; //turn old one off. (backwards)
		IE2 |= UCB0TXIE; // send the turn-off command
	}
	else{ //if we just unplugged
		P1IES = PLUG; //interrupt on plugin
		IE1 &= ~WDTIE;// disable interrupts for the WDT
		P2IE = BIT0; //enable FSR interrupts
		running = 1;
	    //set up TA0 for LED second counting and timing
	    TA0CTL = TASSEL_1 + MC_1; //select ACLK; up mode
	    TA0CCR0 = 12000; //1 second
	    TA0CCTL0 = CCIE; // enable interrupts
	}
}

#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void){
    if (wdtCount == 50){
		IE1 &= ~WDTIE;// disable interrupts for the WDT
		wdtCount = 0; //reset counter
    	__bic_SR_register_on_exit(CPUOFF);  //return function to main()
    }
    else
    	wdtCount++;

}

void itoa(long unsigned int value, char* result, int base)
{
  // check that the base if valid
  if (base < 2 || base > 36) { *result = '\0';}

  char* ptr = result, *ptr1 = result, tmp_char;
  int tmp_value;

  do {
	tmp_value = value;
	value /= base;
	*ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
  } while ( value );

  // Apply negative sign
  if (tmp_value < 0) *ptr++ = '-';
  *ptr-- = '\0';
  while(ptr1 < ptr) {
	tmp_char = *ptr;
	*ptr--= *ptr1;
	*ptr1++ = tmp_char;
  }

}
