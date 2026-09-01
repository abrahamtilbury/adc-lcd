// ============================================================
// PIC16F887 STEPPER CIRCUIT - SIMPLE FULL DIAGNOSTIC
// ============================================================
//
// AUTOMATIC TEST SEQUENCE:
//
// 1. LCD / PIC startup test
// 2. 30 seconds: RAW RC0-RC7 + potentiometer ADC
// 3. RD0 individually
// 4. RD1 individually
// 5. RD2 individually
// 6. RD3 individually
// 7. Teacher's exact forward step sequence
// 8. Teacher's exact reverse step sequence
// 9. Repeat forever
//
// IMPORTANT:
// The DIP switches DO NOT control this diagnostic.
// This means a broken switch cannot stop another test.
//
// ============================================================


// ===============================
// LCD CONNECTIONS
// EXACTLY SAME AS TEACHER'S CODE
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
// VARIABLES
// ===============================

unsigned int adc_value;
unsigned int count;
unsigned short step;


// ===============================
// TEACHER'S KNOWN-WORKING
// MOTOR SEQUENCES
// ===============================

unsigned short full_step[4] = {
    0x01,       // 0001
    0x02,       // 0010
    0x04,       // 0100
    0x08        // 1000
};

unsigned short CCfull_step[4] = {
    0x08,       // 1000
    0x04,       // 0100
    0x02,       // 0010
    0x01        // 0001
};


// ============================================================
// DISPLAY RAW RC7-RC0
//
// NO inversion.
//
// LCD shows EXACTLY what PIC sees.
//
// Example:
//
// 11111111 A:0512
//
// First 8 digits = RC7 RC6 RC5 RC4 RC3 RC2 RC1 RC0
// A = ADC value from RA0 potentiometer
// ============================================================

void showInputsAndADC() {

    adc_value = ADC_Read(0);


    // -------------------------
    // RC7 -> RC0
    // -------------------------

    Lcd_Chr(2, 1, PORTC.F7 ? '1' : '0');
    Lcd_Chr(2, 2, PORTC.F6 ? '1' : '0');
    Lcd_Chr(2, 3, PORTC.F5 ? '1' : '0');
    Lcd_Chr(2, 4, PORTC.F4 ? '1' : '0');

    Lcd_Chr(2, 5, PORTC.F3 ? '1' : '0');
    Lcd_Chr(2, 6, PORTC.F2 ? '1' : '0');
    Lcd_Chr(2, 7, PORTC.F1 ? '1' : '0');
    Lcd_Chr(2, 8, PORTC.F0 ? '1' : '0');


    // -------------------------
    // ADC display
    // Always displayed as 4 digits:
    //
    // 0000
    // 0512
    // 1023
    // -------------------------

    Lcd_Out(2, 9, " A:");

    Lcd_Chr(2, 12, ((adc_value / 1000) % 10) + '0');
    Lcd_Chr(2, 13, ((adc_value / 100)  % 10) + '0');
    Lcd_Chr(2, 14, ((adc_value / 10)   % 10) + '0');
    Lcd_Chr(2, 15, ( adc_value         % 10) + '0');
}


// ============================================================
// DISPLAY THE CURRENT RD3-RD0 OUTPUT
//
// Example:
//
// RD=0001
// ============================================================

void showRD(unsigned short value) {

    Lcd_Out(2, 1, "RD=");

    Lcd_Chr(2, 4, (value & 0x08) ? '1' : '0');
    Lcd_Chr(2, 5, (value & 0x04) ? '1' : '0');
    Lcd_Chr(2, 6, (value & 0x02) ? '1' : '0');
    Lcd_Chr(2, 7, (value & 0x01) ? '1' : '0');
}


// ============================================================
// MAIN PROGRAM
// ============================================================

void main() {


    // ========================================================
    // PORT CONFIGURATION
    // ========================================================

    ANSEL  = 0x01;      // RA0 / AN0 analogue
    ANSELH = 0x00;      // Remaining analogue pins digital

    TRISA = 0x01;       // RA0 = potentiometer input

    TRISB = 0x00;       // LCD output

    TRISC = 0xFF;       // RC0-RC7 ALL inputs
                        // SW1-SW8

    TRISD = 0x00;       // RD0-RD7 outputs
                        // RD0-RD3 -> ULN2003


    PORTD = 0x00;


    // ========================================================
    // INITIALISE ADC + LCD
    // ========================================================

    ADC_Init();

    Lcd_Init();

    Lcd_Cmd(_LCD_CLEAR);
    Lcd_Cmd(_LCD_CURSOR_OFF);


    // ========================================================
    // TEST 1
    // PIC + LCD STARTUP
    // ========================================================

    Lcd_Out(1, 1, "PIC DIAGNOSTIC");
    Lcd_Out(2, 1, "PIC + LCD OK");

    Delay_ms(2000);



    // ========================================================
    // REPEAT ALL TESTS FOREVER
    // ========================================================

    while(1) {


        // ====================================================
        // TEST 2
        //
        // RAW DIP SWITCHES + POTENTIOMETER
        //
        // Runs for 30 seconds.
        //
        // During this time:
        // - toggle SW1
        // - toggle SW2
        // - ...
        // - toggle SW8
        // - rotate potentiometer
        //
        // PORTD stays OFF.
        // ====================================================

        PORTD = 0x00;

        Lcd_Cmd(_LCD_CLEAR);
        Lcd_Out(1, 1, "RAW RC7-RC0");


        for(count = 0; count < 300; count++) {

            showInputsAndADC();

            Delay_ms(100);
        }



        // ====================================================
        // TEST 3
        //
        // RD0 / ULN2003 CHANNEL 1
        // ====================================================

        Lcd_Cmd(_LCD_CLEAR);
        Lcd_Out(1, 1, "RD0 / ULN IN1");

        PORTD = 0x01;

        showRD(0x01);

        Delay_ms(2000);

        PORTD = 0x00;

        Delay_ms(500);



        // ====================================================
        // TEST 4
        //
        // RD1 / ULN2003 CHANNEL 2
        // ====================================================

        Lcd_Cmd(_LCD_CLEAR);
        Lcd_Out(1, 1, "RD1 / ULN IN2");

        PORTD = 0x02;

        showRD(0x02);

        Delay_ms(2000);

        PORTD = 0x00;

        Delay_ms(500);



        // ====================================================
        // TEST 5
        //
        // RD2 / ULN2003 CHANNEL 3
        // ====================================================

        Lcd_Cmd(_LCD_CLEAR);
        Lcd_Out(1, 1, "RD2 / ULN IN3");

        PORTD = 0x04;

        showRD(0x04);

        Delay_ms(2000);

        PORTD = 0x00;

        Delay_ms(500);



        // ====================================================
        // TEST 6
        //
        // RD3 / ULN2003 CHANNEL 4
        // ====================================================

        Lcd_Cmd(_LCD_CLEAR);
        Lcd_Out(1, 1, "RD3 / ULN IN4");

        PORTD = 0x08;

        showRD(0x08);

        Delay_ms(2000);

        PORTD = 0x00;

        Delay_ms(1000);



        // ====================================================
        // TEST 7
        //
        // TEACHER'S EXACT FULL-STEP FORWARD SEQUENCE
        //
        // 0001
        // 0010
        // 0100
        // 1000
        //
        // VERY SLOW: 1 second per step
        // Two complete cycles
        // ====================================================

        Lcd_Cmd(_LCD_CLEAR);
        Lcd_Out(1, 1, "FULL STEP FWD");


        for(count = 0; count < 2; count++) {

            for(step = 0; step < 4; step++) {

                PORTD = full_step[step];

                showRD(full_step[step]);

                Delay_ms(1000);
            }
        }


        PORTD = 0x00;

        Delay_ms(1000);



        // ====================================================
        // TEST 8
        //
        // TEACHER'S EXACT REVERSE SEQUENCE
        //
        // 1000
        // 0100
        // 0010
        // 0001
        //
        // Two complete cycles
        // ====================================================

        Lcd_Cmd(_LCD_CLEAR);
        Lcd_Out(1, 1, "FULL STEP REV");


        for(count = 0; count < 2; count++) {

            for(step = 0; step < 4; step++) {

                PORTD = CCfull_step[step];

                showRD(CCfull_step[step]);

                Delay_ms(1000);
            }
        }


        PORTD = 0x00;



        // ====================================================
        // END OF CYCLE
        // ====================================================

        Lcd_Cmd(_LCD_CLEAR);

        Lcd_Out(1, 1, "CYCLE COMPLETE");
        Lcd_Out(2, 1, "RESTARTING...");

        Delay_ms(2000);
    }
}