
/*
 * Practical Assessment 7
 * Part A - LM35 Temperature Fan Control
 * PIC16F887 | MikroC PRO for PIC
 */

// 1. LCD CONNECTIONS
sbit LCD_RS at RD0_bit;
sbit LCD_EN at RD1_bit;

sbit LCD_D4 at RD4_bit;
sbit LCD_D5 at RD5_bit;
sbit LCD_D6 at RD6_bit;
sbit LCD_D7 at RD7_bit;

sbit LCD_RS_Direction at TRISD0_bit;
sbit LCD_EN_Direction at TRISD1_bit;

sbit LCD_D4_Direction at TRISD4_bit;
sbit LCD_D5_Direction at TRISD5_bit;
sbit LCD_D6_Direction at TRISD6_bit;
sbit LCD_D7_Direction at TRISD7_bit;


// 2. PROGRAM VARIABLES
unsigned int adcValue;
float temperatureC;
unsigned short fanOn = 0;
char temperatureText[15];


// 3. MAIN PROGRAM
void main() {

    // Configure 8 MHz internal oscillator
    OSCCON = 0x71;

    // Configure AN0 as analogue input
    ANSEL = 0x01;
    ANSELH = 0x00;

    // Disable comparators
    C1ON_bit = 0;
    C2ON_bit = 0;

    // Configure sensor and fan pins
    TRISA0_bit = 1;
    TRISC0_bit = 0;
    RC0_bit = 0;

    // Initialise ADC and LCD
    ADC_Init();
    Lcd_Init();
    Lcd_Cmd(_LCD_CLEAR);
    Lcd_Cmd(_LCD_CURSOR_OFF);


    // 4. CONTINUOUS CONTROL LOOP
    while(1) {

        // Read LM35 on AN0
        adcValue = ADC_Read(0);

        // Convert ADC reading to Celsius
        temperatureC = (adcValue * 500.0) / 1023.0;


        // 5. FAN CONTROL WITH HYSTERESIS
        if (temperatureC > 28.0) {
            fanOn = 1;
        }
        else if (temperatureC < 26.0) {
            fanOn = 0;
        }

        // Apply fan state to output
        RC0_bit = fanOn;


        // 6. LCD TEMPERATURE DISPLAY
        FloatToStr_FixLen(temperatureC,
                          temperatureText, 4);

        Lcd_Out(1, 1, "Temp:       C   ");
        Lcd_Out(1, 7, temperatureText);


        // 7. LCD FAN STATUS
        if (fanOn == 1) {
            Lcd_Out(2, 1, "Fan: ON         ");
        }
        else {
            Lcd_Out(2, 1, "Fan: OFF        ");
        }

        Delay_ms(250);
    }
}
