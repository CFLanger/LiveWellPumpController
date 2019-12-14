//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                           Live Well Controller                                      //
//                                                                                                     //
//                                               MSP430G2553                                           //
//                                                                                                     //
//                                                                                                     //
// File              : lwc.c                                                                           //
// Software Engineer : Chris Langer                                                                    //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Project Revision Log (Latest Entries First)                                                         //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Date          Rev/Build      Coder        Status/Revision                                           //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// dd-mmm-yyyy   v.mm.bbbb       ...                ...                                                //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
// 27-Nov-2019   1.00.0000       CFL         Initial Development.                                      //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
#include <msp430.h>

#define SYS_STATUS_LED_ON       P2OUT |= BIT5
#define SYS_STATUS_LED_OFF      P2OUT &= ~BIT5

#define FLOAT_STATUS_LED_ON     P2OUT |= BIT3
#define FLOAT_STATUS_LED_OFF    P2OUT &= ~BIT3

#define SPRAY_FILL_LED_ON       P1OUT |= BIT0
#define SPRAY_FILL_LED_OFF      P1OUT &= ~BIT0

#define DRAIN_LED_ON            P1OUT |= BIT6
#define DRAIN_LED_OFF           P1OUT &= ~BIT6

#define SPRAY_FILL_RELAY_ON     P2OUT &= ~BIT0
#define SPRAY_FILL_RELAY_OFF    P2OUT |= BIT0

#define DRAIN_RELAY_ON          P2OUT &= ~BIT1
#define DRAIN_RELAY_OFF         P2OUT |= BIT1

#define ALL_STOP                    0
#define RAISE_LEVEL                 1
#define AERATE                      2
#define RAISE_LEVEL_IN_DURATION     3
#define LOWER_LEVEL                 4
#define RAISE_LEVEL_B4_ALL_STOP     5

#define INDICATES_EMPTY             0
#define INDICATES_FULL              1

#define ON                          0
#define OFF                         1

#define FLOAT_SWITCH                (P2IN & BIT4)

//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Variables                                                                                    //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
volatile unsigned long time;
volatile unsigned long CycleIntervalTime, CycleDurationTime, DrainDurationTime;
unsigned int Status_LED_cnt;
volatile unsigned int cdtmr0;
unsigned int ADC_ConversionResults[3];

int AerateStatus = 0;
int LiveWellState = ALL_STOP;
int Draining = 0;

unsigned long tAerate;
unsigned long tLower;

unsigned int auxsamp[3][16];
unsigned int auxindx[3];

//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function Prototypes                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void ReadPots( void );
void LiveWellAllStop( void );
void LiveWellRaiseLevel( void );
void LiveWellLowerLevel( void );
void LiveWellAerate( void );
void ManagePumps( void );
unsigned int AvgAuxAI( unsigned int newval, unsigned char ch );

//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
// M A I N,  (Program Entry Point)                                                                     //
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
int main( void ) {

    //
    // Stop Watchdog Timer
    //
    WDTCTL = WDTPW | WDTHOLD;
	
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;

    //
    // Setup Timer TA0, SMCLK/1, Cont Mode
    //
    TA0CTL = TASSEL_2 + MC_2 + TACLR;
    // Configure TA0CCR0 for 1 millisecond
    TA0CCR0 = 1000;
    TA0CCTL0 |= CCIE;

    //
    // Setup Timer TA1, ACLK/1, Cont Mode
    //
    TA1CTL = TASSEL_1 + MC_2 + TACLR;
    // Configure TA1CCR0 for 1 millisecond
    TA1CCR0 = 33;
    TA1CCTL0 |= CCIE;

    //
    // Configure Port Pins
    //
    P2DIR &= ~BIT4;                             // float switch input
    P2REN |= BIT4;                              // enable pullup

    SYS_STATUS_LED_OFF;
    P2DIR |= BIT5;

    FLOAT_STATUS_LED_OFF;
    P2DIR |= BIT3;

    SPRAY_FILL_LED_OFF;
    P1DIR |= BIT0;

    DRAIN_LED_OFF;
    P1DIR |= BIT6;

    SPRAY_FILL_RELAY_OFF;
    P2DIR |= BIT0;

    DRAIN_RELAY_OFF;
    P2DIR |= BIT1;

    ADC10CTL1 = INCH_3 + CONSEQ_1;              // A3/A2/A1, single sequence
    ADC10CTL0 = ADC10SHT_3 + MSC + ADC10ON;
    ADC10DTC1 = 0x03;                           // 3 conversions
    ADC10AE0 |= 0x0E;                           // P1.3,2,1 ADC10 option select

    // Can't run for more than 49 days continous
    time = 0L;

    ReadPots();

    //
    // Unused port save power
    //
    P3OUT = 0;
    P3DIR = 0xFF;

    //
    // Enable Interrupts
    //
    _EINT( );

    // Determine where to start
    if( FLOAT_SWITCH == INDICATES_EMPTY ) {
        cdtmr0 = 5; while( cdtmr0 );
        if( FLOAT_SWITCH == INDICATES_EMPTY ) {
            cdtmr0 = 5; while( cdtmr0 );
            if( FLOAT_SWITCH == INDICATES_EMPTY ) {
                LiveWellRaiseLevel();
                LiveWellState = RAISE_LEVEL;
            }
        }
    } else {
        LiveWellAerate();
        tLower = tAerate = time;
        AerateStatus = 2;
        LiveWellState = AERATE;
    }

	while(1) {

        ReadPots();

        if( !FLOAT_SWITCH ) FLOAT_STATUS_LED_ON;
        else FLOAT_STATUS_LED_OFF;

        ManagePumps();
	}
}



//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     //
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void ManagePumps( void ) {

    static unsigned long tNow;

    if( Draining ) return;

    switch( LiveWellState ) {

    default:
    case ALL_STOP:
        LiveWellAllStop();
        break;

    case RAISE_LEVEL:

        // WAIT FOR filled up
        if( FLOAT_SWITCH == INDICATES_EMPTY ) {
            cdtmr0 = 5; while( cdtmr0 );
            if( FLOAT_SWITCH == INDICATES_EMPTY ) {
                cdtmr0 = 5; while( cdtmr0 );
                if( FLOAT_SWITCH == INDICATES_EMPTY ) {
                    break;
                }
            }
        }
        tAerate = time;
        AerateStatus = 1;
        LiveWellAllStop();
        LiveWellState = AERATE;
        break;

    case RAISE_LEVEL_IN_DURATION:

        // WAIT FOR filled up
        if( FLOAT_SWITCH == INDICATES_EMPTY ) {
            cdtmr0 = 5; while( cdtmr0 );
            if( FLOAT_SWITCH == INDICATES_EMPTY ) {
                cdtmr0 = 5; while( cdtmr0 );
                if( FLOAT_SWITCH == INDICATES_EMPTY ) {
                    break;
                }
            }
        }
        LiveWellAerate();
        LiveWellState = AERATE;
        break;

    case RAISE_LEVEL_B4_ALL_STOP:

        // WAIT FOR filled up
        if( FLOAT_SWITCH == INDICATES_EMPTY ) {
            cdtmr0 = 5; while( cdtmr0 );
            if( FLOAT_SWITCH == INDICATES_EMPTY ) {
                cdtmr0 = 5; while( cdtmr0 );
                if( FLOAT_SWITCH == INDICATES_EMPTY ) {
                    break;
                }
            }
        }
        AerateStatus = 1;
        tAerate = time;
        LiveWellState = AERATE;
        LiveWellAllStop();
        break;

    case LOWER_LEVEL:
        tNow = time;
        if( tNow - tLower > DrainDurationTime ) {
            LiveWellRaiseLevel();
            LiveWellState = RAISE_LEVEL_IN_DURATION;
        }
        break;

    case AERATE:

        tNow = time;

        if( AerateStatus == 1 ) {
            if( tNow - tAerate > CycleIntervalTime ) {
                AerateStatus = 2;
                tAerate = time;
                LiveWellAerate();
            }
        } else {
            if( tNow - tAerate > CycleDurationTime ) {
                LiveWellRaiseLevel();
                LiveWellState = RAISE_LEVEL_B4_ALL_STOP;
            } else {
                //
                // Check for level FALLING, Pump out exceeds pump in
                //  because fill tube is 1/2 inch ID and pump out tube
                //  is 3/4" ID.
                //
                if( FLOAT_SWITCH == INDICATES_EMPTY ) {
                    cdtmr0 = 5; while( cdtmr0 );
                    if( FLOAT_SWITCH == INDICATES_EMPTY ) {
                        cdtmr0 = 5; while( cdtmr0 );
                        if( FLOAT_SWITCH == INDICATES_EMPTY ) {
                            tLower = time;
                            LiveWellState = LOWER_LEVEL;
                            LiveWellLowerLevel();
                        }
                    }
                }
            }
        }
        break;
    }
}



//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     //
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void ReadPots( void ) {

    unsigned long i;

    ADC10SA = (unsigned int)ADC_ConversionResults;      // Data buffer start
    ADC10CTL0 |= ENC + ADC10SC;                         // Sampling and conversion start

    while( ADC10CTL1 & ADC10BUSY )                      // Wait if ADC10 core is active
                 ;
    ADC10CTL0 &= ~ENC;

    i = AvgAuxAI( ADC_ConversionResults[2], 2 );
    // 0 to 10 minutes >> 0 to 600,000 ms
    // Calculated a slope and offset from a linear regression curve fit
    //  to make the potentiometer ADC counts match the time scale of the knob/dial
    i = ( i * 557074 ) + 28097185;
    i /= 1000;
    CycleIntervalTime = i;


    i = AvgAuxAI( ADC_ConversionResults[1], 1);
    // 0 to 10 minutes >> 0 to 600,000 ms
    // Calculated a slope and offset from a linear regression curve fit
    //  to make the potentiometer ADC counts match the time scale of the knob/dial
    i = ( i * 554866 ) + 42290131;
    i /= 1000;
    CycleDurationTime = i;


    i = AvgAuxAI( ADC_ConversionResults[0], 0);
    if( i < 6 ) {
        Draining = 1;
        DRAIN_RELAY_ON;
        DRAIN_LED_ON;
        SPRAY_FILL_RELAY_OFF;
        SPRAY_FILL_LED_OFF;
    } else {
        // 0 to 10 seconds >> 0 to 10,000 ms
        // Calculated a slope and offset from a linear regression curve fit
        //  to make the potentiometer ADC counts match the time scale of the knob/dial
        i = ( i * 9211 ) + 499849;
        i /= 1000;
        DrainDurationTime = i;
        if( Draining ) {
            Draining = 0;
            LiveWellAllStop();
        }
    }
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     //
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
void LiveWellAllStop( void ) {

    DRAIN_RELAY_OFF;
    DRAIN_LED_OFF;

    SPRAY_FILL_RELAY_OFF;
    SPRAY_FILL_LED_OFF;
}

void LiveWellRaiseLevel( void ) {

    DRAIN_RELAY_OFF;
    DRAIN_LED_OFF;

    SPRAY_FILL_RELAY_ON;
    SPRAY_FILL_LED_ON;
}

void LiveWellLowerLevel( void ) {

    DRAIN_RELAY_ON;
    DRAIN_LED_ON;

    SPRAY_FILL_RELAY_ON;
    SPRAY_FILL_LED_ON;
}

void LiveWellAerate( void ) {

    DRAIN_RELAY_ON;
    DRAIN_LED_ON;

    SPRAY_FILL_RELAY_ON;
    SPRAY_FILL_LED_ON;
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     //
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0( void ) {

    static unsigned char sled = 0;

    if( ++Status_LED_cnt == 100 ) {
        Status_LED_cnt = 0;
        sled ^= 1;
        if( sled ) SYS_STATUS_LED_ON;
        else SYS_STATUS_LED_OFF;
    }

    if( cdtmr0 ) cdtmr0--;

    // Interrupt every 1 Millisecond
    TA0CCR0 = TA0R + 1000;
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     //
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Timer A1 CCR0 interrupt service routine (DERIVED FROM 32.768 Khz ACLK XTAL)
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer1_A0( void ) {

    time++;

    // Interrupt every 1 millisecond
    TA1CCR0 = TA1R + 33;
}


//
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                     //
//                                                                                                     //
//                                                                                                     //
// Description:                                                                                        //
// Arguments:                                                                                          //
// Returns:                                                                                            //
//                                                                                                     //
// Notes/Warnings/Caveats:                                                                             //
//                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
// Rolling average, 8 terms, throw away highest and lowest
//
unsigned int AvgAuxAI( unsigned int newval, unsigned char ch ) {

    unsigned long tlwrk;
    unsigned long tmp, min, max;
    unsigned char u0;

    auxsamp[ ch ][ auxindx[ ch ]++ & 0x0F ] = newval;

    if( auxindx[ ch ] & 0x10 ) auxindx[ ch ] |= 0x80;
    auxindx[ ch ] &= 0x8F;

    if( auxindx[ ch ] & 0x80 ){
        min = 1024;
        max = 0;
        for( tlwrk = 0, u0 = 0; u0 < 16; ){
            tlwrk += tmp = auxsamp[ ch ][ u0++ ];
            if( tmp < min ) min = tmp;
            if( tmp > max ) max = tmp;
        }
        tlwrk -= min;
        tlwrk -= max;
        tlwrk /= 14;

        newval = ( unsigned int )tlwrk;
    }
    return( newval );
}
