// SW1-SW4 direction/step selection reference
// SW1 / RC0 ON -> 0001 -> 0x01
// SW2 / RC1 ON -> 0010 -> 0x02
// SW3 / RC2 ON -> 0100 -> 0x04
// SW4 / RC3 ON -> 1000 -> 0x08

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
// VARIABLES
// ===============================

unsigned int adc_value;
unsigned int delay_time;
char txt[7];
unsigned int i;
unsigned char direction;
unsigned char last_action = 0xFF;

// ===============================
// FULL STEP SEQUENCE - 4 positions
// ===============================

unsigned short full_step[4] = {0b0001, 0b0010, 0b0100, 0b1000};
unsigned short CCfull_step[4] = {0b1000, 0b0100, 0b0010, 0b0001};

// ===============================
// HALF STEP SEQUENCE - twice as many steps
// ===============================
unsigned short half_step[8] = {0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001};
unsigned short CChalf_step[8] = {0b1001, 0b1000, 0b1100, 0b0100, 0b0110, 0b0010, 0b0011,0b0001};

// ===============================
//           Functions 
// ===============================

// ===========================================
// Display message on LCD only when mode changes
// ===========================================

void updateLCD(unsigned char action, char *message) {
   if(action != last_action) {
      Lcd_Cmd(_LCD_CLEAR);
      Lcd_Out(1, 1, message);
      last_action = action;
   }
}

// ===========================================
// Run motor in full step mode with assigned delay
// ===========================================

void runFullStep(unsigned int step_delay) {
   for(i=0; i<4; i++) {

      // SW5 - Halt
      if(PORTC.F4 == 1) {
         PORTD = 0x00;
         return;
      }

      PORTD = full_step[i];
      VDelay_ms(step_delay);
   }
}

// ===============================================================================
// Run stepper in counter clockwise full step mode with assigned delay
// ===============================================================================

void runCCFullStep(unsigned int step_delay) {
   for(i=0; i<4; i++) {

      // SW5 - Halt
      if(PORTC.F4 == 1) {
         PORTD = 0x00;
         return;
      }

      PORTD = CCfull_step[i];
      VDelay_ms(step_delay);
   }
}

// ===========================================================
// Run stepper in half step mode with assigned delay
// ========================================================

void runHalfStep(unsigned int step_delay) {
   for(i=0; i<8; i++) {

      // SW5 - Halt
      if(PORTC.F4 == 1) {
         PORTD = 0x00;
         return;
      }

      PORTD = half_step[i];
      VDelay_ms(step_delay);
   }
}

// ======================================================================
// Run stepper in counter clockwise half step mode with custom delay
// ======================================================================

void runCCHalfStep(unsigned int step_delay) {
   for(i=0; i<8; i++) {

      // SW5 - Halt
      if(PORTC.F4 == 1) {
         PORTD = 0x00;
         return;
      }

      PORTD = CChalf_step[i];
      VDelay_ms(step_delay);
   }
}

// =========================
// Stop the stepper motor
// ========================

void stopMotor() {
   PORTD = 0x00;
}

// =========================================
// Read potentiometer and calculate delay
// =========================================

unsigned int readPotDelay() {
   adc_value = ADC_Read(0);
   return 2 + (adc_value / 4);  // ADC 0-1023 gives approximately 2-257 ms
}

// ===============================
// MAIN PROGRAM
// ===============================

void main()
{
    // ---------------------------
    // PORT CONFIGURATION
    // ---------------------------

    ANSEL  = 0x01;     // AN0 analog
    ANSELH = 0x00;      // Remaining analogue channels digital

    C1ON_bit = 0;       // Disable comparator 1
    C2ON_bit = 0;       // Disable comparator 2

    TRISA = 0x01;     // RA0 input (pot)
    TRISB = 0x00;       // LCD outputs
    TRISC = 0xFF;       // SW1-SW8 inputs on RC0-RC7
    TRISD = 0x00;       // Stepper motor control outputs

    PORTD = 0x00;

    // ---------------------------
    // INITIALISE ADC AND LCD
    // ---------------------------

    ADC_Init();
    Lcd_Init();
    Lcd_Cmd(_LCD_CLEAR);
    Lcd_Cmd(_LCD_CURSOR_OFF);

    delay_time = 50;

    // ===========================
    // MAIN CONTROL LOOP - 8 DIP switch control using if-else
    // ===========================

    while(1) {

            // =====================================
            // READ DIRECTION / STEP MODE SWITCHES
            // SW1-SW4 = RC0-RC3
            // =====================================
        
            direction = PORTC & 0x0F;
        
        
            // =====================================
            // SPEED CONTROL
            // =====================================
        
            // Normal/default speed
            delay_time = 50;
        
        
            // SW8 - Potentiometer controls speed
            // SW8 has priority over SW6/SW7 when analog control is selected
            if(PORTC.F7 == 1) {
                delay_time = readPotDelay();
            }
        
        
            // SW6 - Speed up
            else if((PORTC.F5 == 1) && (PORTC.F6 == 0)) {
                delay_time = 25;
            }
        
        
            // SW7 - Speed down
            else if((PORTC.F6 == 1) && (PORTC.F5 == 0)) {
                delay_time = 100;
            }
        
        
            // =====================================
            // MOTOR CONTROL
            // =====================================
        
            // SW5 - Halt
            if(PORTC.F4 == 1) {
                updateLCD(5, "Mode: HALT");
                stopMotor();
        
                Lcd_Out(2, 1, "Motor stopped");
            }
        
        
            // SW1 - Full Step Clockwise
            else if(direction == 0x01) {     //only SW1 is selected
                updateLCD(1, "Mode: FULL CW");
        
                IntToStr(delay_time, txt);
                Lcd_Out(2, 1, "Delay:");
                Lcd_Out(2, 8, txt);
        
                runFullStep(delay_time);
            }
        
        
            // SW2 - Half Step Clockwise
            else if(direction == 0x02) {         // only SW2 selected
                updateLCD(2, "Mode: HALF CW");
        
                IntToStr(delay_time, txt);
                Lcd_Out(2, 1, "Delay:");
                Lcd_Out(2, 8, txt);
        
                runHalfStep(delay_time);
            }
        
        
            // SW3 - Full Step Counter Clockwise
            else if(direction == 0x04) {         //only sw3 is selected
                updateLCD(3, "Mode: FULL CCW");
        
                IntToStr(delay_time, txt);
                Lcd_Out(2, 1, "Delay:");
                Lcd_Out(2, 8, txt);
        
                runCCFullStep(delay_time);
            }
        
        
            // SW4 - Half Step Counter Clockwise
            else if(direction == 0x08) {        // only SW4 selected
                updateLCD(4, "Mode: HALF CCW");
        
                IntToStr(delay_time, txt);
                Lcd_Out(2, 1, "Delay:");
                Lcd_Out(2, 8, txt);
        
                runCCHalfStep(delay_time);
            }
        
        
            // No direction switch selected
            else if(direction == 0x00) {
                updateLCD(0, "Mode: IDLE");
                stopMotor();
        
                Lcd_Out(2, 1, "Motor stopped");
            }
        
        
            // More than one SW1-SW4 selected
            else {
                updateLCD(6, "ONE DIR ONLY");
                stopMotor();
        
                Lcd_Out(2, 1, "Motor stopped");
            }
        }
}

            
