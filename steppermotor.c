// =====================================================
// STEPPER MOTOR CONTROL - PIC16F887
// =====================================================


// ===============================
// VARIABLES
// ===============================


unsigned int adc_value;
unsigned int delay_time;
unsigned int i;
unsigned char mode 0;
unsigned char last_mode 0xFF;

// ===============================
// FULL STEP SEQUENCE
// ===============================

unsigned abort full_step_CW[4] *(1, 2, 4, 8);


// ===============================
// LCD CONNECTIONS
// ===============================

sbit LCD_RS at RB4_bit;
sbit LCD_EN at RB5_bit;
sbit LCD_D4 at RB0_bit;
sbit LCD_D5 at RB1_bit;
sbit LCD_D6 at RB2_bit;
sbit LCD_D7 at RB3_bit;

sbit LCD_RS_Direction at TRISB4_bit;
sbit LCD_EN_Direction at TRISB5_bit;
sbit LCD_D4_Direction at TRISB0_bit;
sbit LCD_D5_Direction at TRISB1_bit;
sbit LCD_D6_Direction at TRISB2_bit;
sbit LCD_D7_Direction at TRISB3_bit;


// ===============================
// SPEED CONTROL
// ===============================

void UpdateSpeed()
{
    // Normal motor speed
    delay_time = 200;


    // SW8 - Potentiometer controls the base speed
    if(PORTC.F7 == 1)
    {
        adc_value = ADC_Read(2);

        // ADC 0     -> 500 ms -> slow
        // ADC 1023  -> 50 ms  -> fast
        delay_time =
            500 - (((unsigned long)adc_value * 450) / 1023);
    }


    // SW6 - Speed up
    // Reduce delay between steps
    if(PORTC.F5 == 1)
    {
        if(delay_time > 100)
            delay_time = delay_time - 50;
    }


    // SW7 - Speed down
    // Increase delay between steps
    if(PORTC.F6 == 1)
    {
        if(delay_time < 450)
            delay_time = delay_time + 50;
    }
}


// ===============================
// LCD DELAY DISPLAY
// ===============================

void ShowDelay()
{
    WordToStr(delay_time, delay_txt);

    // Clear line 2 before rewriting delay
    Lcd_Out(2,1,"                ");

    Lcd_Out(2,1,"DELAY:");
    Lcd_Out(2,7,delay_txt);
    Lcd_Out(2,13,"ms");
}


// ===============================
// RUN STEPPER MOTOR
// ===============================

// halfStepMode:
// 0 = Full Step
// 1 = Half Step
//
// reverse:
// 0 = Clockwise
// 1 = Counter-Clockwise

void RunMotor(unsigned char halfStepMode,
              unsigned char reverse)
{
    unsigned char i;
    unsigned char numberOfSteps;


    // Select the number of patterns required
    if(halfStepMode == 0)
        numberOfSteps = 4;
    else
        numberOfSteps = 8;


    for(i = 0; i < numberOfSteps; i++)
    {
        // SW5 - Halt immediately
        if(PORTC.F4 == 1)
        {
            PORTD = 0x00;
            return;
        }


        // Update speed so the potentiometer
        // can affect the motor while running
        UpdateSpeed();


        // FULL STEP
        if(halfStepMode == 0)
        {
            if(reverse == 0)
                PORTD = fullStep[i];
            else
                PORTD = fullStep[numberOfSteps - 1 - i];
        }


        // HALF STEP
        else
        {
            if(reverse == 0)
                PORTD = halfStep[i];
            else
                PORTD = halfStep[numberOfSteps - 1 - i];
        }


        Vdelay_ms(delay_time);
    }
}


// ===============================
// MAIN PROGRAM
// ===============================

void main()
{
    // ---------------------------
    // PORT CONFIGURATION
    // ---------------------------

    ANSEL  = 0x04;      // RA2 / AN2 = analogue input
    ANSELH = 0x00;      // Remaining analogue channels digital

    C1ON_bit = 0;       // Disable comparator 1
    C2ON_bit = 0;       // Disable comparator 2

    TRISA = 0xFF;       // RA2 potentiometer input
    TRISB = 0x00;       // LCD outputs
    TRISC = 0xFF;       // SW1-SW8 inputs
    TRISD = 0x00;       // Stepper motor outputs

    PORTD = 0x00;


    // ---------------------------
    // INITIALISE ADC AND LCD
    // ---------------------------

    ADC_Init();

    Lcd_Init();
    Lcd_Cmd(_LCD_CLEAR);
    Lcd_Cmd(_LCD_CURSOR_OFF);

    delay_time = 200;


    // ===========================
    // MAIN CONTROL LOOP
    // ===========================

    while(1)
    {
        // Read speed controls before
        // determining motor operation
        UpdateSpeed();


        // ---------------------------
        // SW5 - HALT
        // ---------------------------

        if(PORTC.F4 == 1)
        {
            PORTD = 0x00;

            Lcd_Out(1,1,"MOTOR HALT      ");
            Lcd_Out(2,1,"                ");
        }


        // ---------------------------
        // SW1 - FULL STEP CW
        // ---------------------------

        else if(PORTC.F0 == 1)
        {
            Lcd_Out(1,1,"FULL STEP CW    ");

            ShowDelay();

            RunMotor(0, 0);
        }


        // ---------------------------
        // SW2 - HALF STEP CW
        // ---------------------------

        else if(PORTC.F1 == 1)
        {
            Lcd_Out(1,1,"HALF STEP CW    ");

            ShowDelay();

            RunMotor(1, 0);
        }


        // ---------------------------
        // SW3 - FULL STEP CCW
        // ---------------------------

        else if(PORTC.F2 == 1)
        {
            Lcd_Out(1,1,"FULL STEP CCW   ");

            ShowDelay();

            RunMotor(0, 1);
        }


        // ---------------------------
        // SW4 - HALF STEP CCW
        // ---------------------------

        else if(PORTC.F3 == 1)
        {
            Lcd_Out(1,1,"HALF STEP CCW   ");

            ShowDelay();

            RunMotor(1, 1);
        }


        // ---------------------------
        // NO MOTOR MODE SELECTED
        // ---------------------------

        else
        {
            PORTD = 0x00;

            Lcd_Out(1,1,"MOTOR OFF       ");
            ShowDelay();
        }


        Delay_ms(20);
    }
}
