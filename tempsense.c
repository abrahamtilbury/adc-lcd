
/*
 * Practical Assessment 7
 * Part A - LM35 Temperature Fan Control
 * PIC16F887 | MikroC PRO for PIC
 */

// 1. LCD CONNECTIONS (4-bit mode)
// Control pins
sbit LCD_RS at RB4_bit;  // PIC pin 37 -> LCD RS
sbit LCD_EN at RB5_bit;  // PIC pin 38 -> LCD E

// Data pins
sbit LCD_D4 at RB0_bit;  // PIC pin 33 -> LCD D4
sbit LCD_D5 at RB1_bit;  // PIC pin 34 -> LCD D5
sbit LCD_D6 at RB2_bit;  // PIC pin 35 -> LCD D6
sbit LCD_D7 at RB3_bit;  // PIC pin 36 -> LCD D7

// Pin directions (0 = output)
sbit LCD_RS_Direction at TRISB4_bit;
sbit LCD_EN_Direction at TRISB5_bit;
sbit LCD_D4_Direction at TRISB0_bit;
sbit LCD_D5_Direction at TRISB1_bit;
sbit LCD_D6_Direction at TRISB2_bit;
sbit LCD_D7_Direction at TRISB3_bit;

// 2. PROGRAM VARIABLES
unsigned int adcValue;
float temperatureC;       // Measured temperature in Celsius
float displayTemperature; // Celsius or Fahrenheit for LCD
unsigned short ledOn = 0;
char temperatureText[15];

// 3. MAIN PROGRAM
void main() {

    // External 4 MHz ceramic resonator on OSC1/OSC2
    // Oscillator mode: XT (configured in MikroC project)

    // Enable AN1 (LM35) and AN2 (LM335)
        ANSEL = 0x06;
        ANSELH = 0x00;

    // Disable comparators
    C1ON_bit = 0;
    C2ON_bit = 0;

    // Configure temperature sensor inputs
    TRISA1_bit = 1;  // PIC pin 3: RA1/AN1, LM35 output
    TRISA2_bit = 1;  // PIC pin 4: RA2/AN2, LM335 output

    // LED output
    TRISD0_bit = 0;  // PIC pin 19: RD0 output
    RD0_bit = 0;     // LED initially OFF
    
    // DIP switch input
    TRISD7_bit = 1;  // PIC pin 30: RD7 input
    
    // Additional hardware
    // RD0, PIC pin 19: LED
    // RD7, PIC pin 30: DIP switch

    // Initialise ADC and LCD
    ADC_Init();
    Lcd_Init();
    Lcd_Cmd(_LCD_CLEAR);
    Lcd_Cmd(_LCD_CURSOR_OFF);

    // 4. CONTINUOUS CONTROL LOOP
    while(1) {

        // Read LM35 on AN0
        adcValue = ADC_Read(1);    // Read LM35 on AN1
        
        // Convert LM35 ADC reading to Celsius
        temperatureC = (adcValue * 500.0) / 1023.0;


       // 5. TEMPERATURE INDICATOR WITH HYSTERESIS
        // LED ON above 28 C, OFF below 26 C
        
        if (temperatureC > 28.0) {
            ledOn = 1;
        }
        else if (temperatureC < 26.0) {
            ledOn = 0;
        }
        
        // Update LED connected to RD0 (pin 19)
        RD0_bit = ledOn;


        // 6. LCD TEMPERATURE DISPLAY (C / F SELECTION)

        // RD7 is connected to the DIP switch.
        // Assumes external pull-up: LOW = switch ON.
        
        // Select display units
        if (RD7_bit == 0) {
            // DIP switch ON: Fahrenheit
            displayTemperature = (temperatureC * 9.0 / 5.0) + 32.0;
            Lcd_Out(1, 13, "F");
        }
        else {
            // DIP switch OFF: Celsius
            displayTemperature = temperatureC;
            Lcd_Out(1, 13, "C");
        }
        
        // Convert selected temperature into display text
        FloatToStr_FixLen(displayTemperature, temperatureText, 5);
        
        // Update LCD
        Lcd_Out(1, 1, "Temp:       ");
        Lcd_Out(1, 7, temperatureText);
        
        // 7. LCD LED STATUS 
        // DIP OFF (open), RD7 HIGH (1), temp .00 C 
        // DIP ON (CLOSED), RD7 LOW (0), temp .00 F
        if (ledOn == 1) {
            Lcd_Out(2, 1, "LED: ON         ");
        }
        else {
            Lcd_Out(2, 1, "LED: OFF        ");
        }

        Delay_ms(250);
    }
}
