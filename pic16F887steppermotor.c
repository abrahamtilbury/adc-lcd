// SW1-SW8 are connected to PORTC and operate as active-low inputs:
// DIP OFF = logic 1, DIP ON = logic 0
//
// SW1-SW4 select the motor direction and stepping mode.
// Because these switches are active-low, PORTC is inverted in
// software before SW1-SW4 are checked.
// After inversion:
// SW1 / RC0 ON -> 0001 -> 0x01 -> Full-Step Clockwise
// SW2 / RC1 ON -> 0010 -> 0x02 -> Half-Step Clockwise
// SW3 / RC2 ON -> 0100 -> 0x04 -> Full-Step Counter-Clockwise
// SW4 / RC3 ON -> 1000 -> 0x08 -> Half-Step Counter-Clockwise
//
// SW5 / RC4 -> Halt motor
// SW6 / RC5 -> Increase motor speed
// SW7 / RC6 -> Decrease motor speed
// SW8 / RC7 -> Potentiometer controls motor speed
//
// RA0 / AN0 -> Potentiometer analogue input
// PORTB     -> LCD interface
// PORTD     -> Stepper motor driver outputs

sbit LCD_D4 at RB0_bit;          // LCD CONNECTIONS, data lines D4-D7 connected to RB0-RB3
sbit LCD_D5 at RB1_bit;
sbit LCD_D6 at RB2_bit;
sbit LCD_D7 at RB3_bit;
sbit LCD_RS at RB4_bit;     // Register Select
sbit LCD_EN at RB5_bit;       // Enable
sbit LCD_RS_Direction at TRISB4_bit;  // Configure the corresponding LCD pins as outputs using TRISB
sbit LCD_EN_Direction at TRISB5_bit;  
sbit LCD_D4_Direction at TRISB0_bit;
sbit LCD_D5_Direction at TRISB1_bit;
sbit LCD_D6_Direction at TRISB2_bit;
sbit LCD_D7_Direction at TRISB3_bit;
// ===============================
// VARIABLES
// ===============================

unsigned int delay_time;       // Stores delay between motor steps; smaller delay = faster speed
char txt[7];                     // Character array used to convert delay_time into text for LCD
unsigned char direction;               // Stores inverted state of SW1-SW4 for mode selection
unsigned char last_action = 0xFF; // Stores previous mode; 0xFF ensures LCD updates on startup

// 8-state sequence: half-step uses every state; full-step uses even states 0,2,4,6; reverse order gives CCW
unsigned short half_step[8] = {0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001};


// ========       Functions  =========
void updateLCD(unsigned char action, char *message) { // Display message on LCD when mode changes
   if(action != last_action) {
      Lcd_Cmd(_LCD_CLEAR);
      Lcd_Out(1, 1, message);
      last_action = action;
   }
}

void displayDelay() { // Convert current delay to text and display it on LCD row 2
   IntToStr(delay_time, txt);
   Lcd_Out(2, 1, "Delay:");
   Lcd_Out(2, 8, txt);
}

// half: 0=full/1=half; reverse: 0=CW/1=CCW
void runStepper(unsigned short half, unsigned short reverse, unsigned int step_delay) {
   unsigned short i, steps = half ? 8 : 4;

   for(i=0; i<steps; i++) {
      if(PORTC.F4 == 0) { // Check SW5 before each step
         PORTD = 0x00;
         return;
      }

      PORTD = half_step[(half ? 1 : 2) * (reverse ? steps-1-i : i)]; // Half-step uses every index; full-step skips to every second state
   }
}

// =================== Read potentiometer and calculate delay ===================
unsigned int readPotDelay() {
   return 257 - (ADC_Read(0) / 4); // AN0 ADC 0-1023 -> approx. 257-2 ms; higher voltage = faster motor
}                        

//  =========== MAIN PROGRAM ==========
void main()
{
    // -------------- PORT CONFIGURATION --------------
    ANSEL = 0x01; // Configure AN0 as analogue input for potentiometer
    ANSELH = 0x00; // Configure remaining analogue-capable channels as digital
    C1ON_bit = 0;       // Disable comparator 1
    C2ON_bit = 0;       // Disable comparator 2
    TRISA = 0x01; // RA0 input for potentiometer; remaining PORTA pins outputs
    TRISB = 0x00;       // LCD outputs
    TRISC = 0xFF;       // SW1-SW8 inputs on RC0-RC7
    TRISD = 0x00;       // Stepper motor control outputs
    PORTD = 0x00; // Ensure motor outputs are OFF at startup

    // ------------ INITIALISE ADC AND LCD --------------
    ADC_Init();
    Lcd_Init();
    Lcd_Cmd(_LCD_CLEAR);
    Lcd_Cmd(_LCD_CURSOR_OFF);

    //  ================== MAIN CONTROL LOOP - 8 DIP switch control using if-else ===========
    while(1) {
            //  ========== READ DIRECTION / STEP MODE SWITCHES - SW1-SW4 = RC0-RC3  ===================
            direction = (~PORTC) & 0x0F; // Invert active-low inputs then mask RC4-RC7 so only SW1-4 remain
      
            //  ================ SPEED CONTROL =============
            delay_time = 50;  // Reset to normal 50 ms delay unless SW6, SW7 or SW8 overrides it
        
            // Active-low SW8: RC7 = 0; checked first so potentiometer overrides SW6/SW7
            if(PORTC.F7 == 0) {
                delay_time = readPotDelay();  // SW8 - Potentiometer controls speed
            }

            // SW6 speed-up; if SW6+SW7 are both ON, neither overrides the default 50 ms
            else if((PORTC.F5 == 0) && (PORTC.F6 == 1)) {
                delay_time = 25;                        // 25 ms delay = faster than default
            }
        
            // SW7 - Speed down
            else if((PORTC.F6 == 0) && (PORTC.F5 == 1)) {
                delay_time = 100;                                          // Slower speed
            }
        
            //  ================== MOTOR CONTROL ==================
            if(PORTC.F4 == 0) { // SW5 has motor-control priority and stops motor at the next halt check
                updateLCD(5, "Mode: HALT");
                PORTD = 0x00; // De-energise motor outputs
                Lcd_Out(2, 1, "Motor stopped");
            }
            // SW1 - Full-Step Clockwise
            else if(direction == 0x01) {                  
                updateLCD(1, "Mode: FULL CW");
                displayDelay();
                runStepper(0, 0, delay_time);
            }
        
            // SW2 - Half-Step Clockwise
            else if(direction == 0x02) {      
                updateLCD(2, "Mode: HALF CW");      
                displayDelay();
                runStepper(1, 0, delay_time);
            }
        
            // SW3 - Full-Step Counter-Clockwise
            else if(direction == 0x04) {     
                updateLCD(3, "Mode: FULL CCW");
                displayDelay();
                runStepper(0, 1, delay_time);
            }
            // SW4 - Half-Step Counter-Clockwise
            else if(direction == 0x08) {  
                updateLCD(4, "Mode: HALF CCW");
                displayDelay();
                runStepper(1, 1, delay_time);
            }
        
            // No SW1-SW4 selected: enter idle state
            else if(direction == 0x00) {
                updateLCD(0, "Mode: IDLE");
                PORTD = 0x00; // De-energise motor outputs
                Lcd_Out(2, 1, "Motor stopped");
            }
        
            // Invalid selection: more than one SW1-SW4 active
            else {
                updateLCD(6, "ONE DIR ONLY");
                PORTD = 0x00; // De-energise motor outputs
                Lcd_Out(2, 1, "Motor stopped");
            }
        }
}       
