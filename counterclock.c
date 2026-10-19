// COUNTER / CLOCK, PIC16F887, 4MHz, ABRAHAM TILBURY 30127103
// DIP1 active low logic: ON = 0, OFF = 1
// Four-digit 7-segment display; hexadecimal patterns control individual segments
// 7 segment display - Common anode configuration
// 
// Wiring
// PIN1           - MCLR, 10kOhm pullup to +5V
// PIN2 (RA0)     - DIP1, 10kOhm pullup to +5V, SW connects RA0 to GND
// PIN11,32       - +5V supply rail
// PIN12,31       - GND SUPPLY RAIL
// PIN13/14 4MHz  - Osc Crystal
// PIN15  (C0)    - DIG4 VCC
// PIN16  (C1)    - DIG3 VCC
// PIN17  (C2)    - DIG2 VCC
// PIN18  (C3)    - DIG1 VCC
// PIN19  (D0)    - pin a DIG1-4
// PIN20  (D1)    - pin b DIG1-4
// PIN21  (D2)    - pin c DIG1-4
// PIN22  (D3)    - pin d DIG1-4
// PIN27  (D4)    - pin e DIG1-4
// PIN28  (D5)    - pin f DIG1-4
// PIN29  (D6)    - pin g DIG1-4
// PIN30  (D7)    - pin dp DIG1-4

// MODES: DIP1 ON = COUNTER, DIP1 OFF = CLOCK

//Arrays
unsigned short seg[10] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90};      // Normal digits, decimal point OFF
unsigned short seg1[10] = {0x40, 0x79, 0x24, 0x30, 0x19, 0x12, 0x02, 0x78, 0x00, 0x10};     // Digits with decimal point on

volatile unsigned int counter = 0;       // Automatic counter starts at zero
volatile unsigned short hours = 12;      // Clock starts at 12 hours
volatile unsigned short minutes = 0;     // Clock starts at 00 minutes
volatile unsigned short seconds = 0;     // Tracks seconds for the clock
volatile unsigned short timerTicks = 0;  // Counts timer interrupts

void show_number(unsigned int value)     // multiplexes four digit display
{
  unsigned short d1, d2, d3, d4;
  unsigned short i;

  // obtain digits + segment code from array
  d1 = seg[value % 10];
  d2 = seg[ (value / 10) % 10];
  if(PORTA.F0 == 0)                       // DIP1 ON = Counter mode
    { d3 = seg[(value / 100) % 10];}      // Counter: decimal point OFF
  else                                    // DIP1 OFF = Clock mode
    { d3 = seg1[(value / 100) % 10];}     // Clock: decimal point ON

  d4 = seg[(value / 1000) % 10];
  
for(i = 0; i < 10; i++)                  // Repeat the four digits 10 times
{
    PORTC = 0;                           // Disable all digits
    PORTD = d1;                          // Load units segment pattern
    PORTC = 1;                           // Enable RC0 digit
    Delay_ms(1);                         // Display for 1 millisecond

    PORTC = 0;                           // Disable all digits
    PORTD = d2;                          // Load tens segment pattern
    PORTC = 2;                           // Enable RC1 digit
    Delay_ms(1);                         // Display for 1 millisecond

    PORTC = 0;                           // Disable all digits
    PORTD = d3;                          // Load hundreds segment pattern
    PORTC = 4;                           // Enable RC2 digit
    Delay_ms(1);                         // Display for 1 millisecond

    PORTC = 0;                           // Disable all digits
    PORTD = d4;                          // Load thousands segment pattern
    PORTC = 8;                           // Enable RC3 digit
    Delay_ms(1);                         // Display for 1 millisecond
}

PORTC = 0;                               // Disable digits after the loop
}                                        // End show_number function


void interrupt()                         // Timer interrupt function, maintains counter and clock time
{
    if(PIR1.F1 == 1)                    // Check if Timer2 triggered
    {
        PIR1.F1 = 0;                    // Clear interrupt flag
        timerTicks++;                   // Count another 4ms interval

        if(timerTicks >= 250)           // 250 x 4ms = 1 second
        {
            timerTicks = 0;             // Reset timer count
            counter++;                  // Increase counter by one

            if(counter > 9999)          // Check maximum counter value
                counter = 0;            // Restart counter at 0000

            seconds++;                  // Increase clock seconds

            if(seconds >= 60)           // Check if 60 seconds passed
            {
                seconds = 0;            // Reset seconds
                minutes++;              // Increase minutes

                if(minutes >= 60)       // Check if 60 minutes passed
                {
                    minutes = 0;        // Reset minutes
                    hours++;            // Increase hours

                    if(hours >= 24)     // Check if 24 hours passed
                        hours = 0;      // Restart clock at midnight
                }
            }
        }
    }
}


void main()                              // READS RA0 and chooses which value to display
{
    unsigned int displayValue;          // Number sent to display

    ANSEL = 0;                          // Configure analogue pins as digital
    ANSELH = 0;                         // Disable other analogue inputs

    TRISA = 0xFF;                       // PORTA inputs (DIP switch)
    TRISC = 0x00;                       // PORTC outputs (digit selection)
    TRISD = 0x00;                       // PORTD outputs (segments)

    PORTC = 0;                          // All digits OFF
    PORTD = 0xFF;                       // All segments OFF

    T2CON = 0;                          // Stop Timer2 for configuration
    PR2 = 249;                          // Set Timer2 period to 4ms at 4MHz
    TMR2 = 0;                           // Clear Timer2 counter
    PIR1.F1 = 0;                        // Clear Timer2 interrupt flag
    PIE1.F1 = 1;                        // Enable Timer2 interrupt
    T2CON = 0x07;                       // Start Timer2, prescaler 1:16

    INTCON.F6 = 1;                      // Enable peripheral interrupts
    INTCON.F7 = 1;                      // Enable global interrupts

    while(1)                            // Repeat forever
    {
        if(PORTA.F0 == 0)               // DIP1 ON = Counter mode
        {
            INTCON.F7 = 0;              // Pause interrupts while reading
            displayValue = counter;     // Read counter value
            INTCON.F7 = 1;              // Resume interrupts
        }
        else                            // DIP1 OFF = Clock mode
        {
            INTCON.F7 = 0;              // Pause interrupts while reading
            displayValue = hours * 100 + minutes;
                                         // Combine hours and minutes (HHMM)
            INTCON.F7 = 1;              // Resume interrupts
        }

        show_number(displayValue);      // Display selected value
    }
}

