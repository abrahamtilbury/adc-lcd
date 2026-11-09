// PracAss 7 - 30127103 - Temperature Fan Control
// RC0 fan control, RD0 status LED and C/F selection
// 1. LCD CONNECTIONS (4-bit mode)
sbit LCD_RS at RB4_bit;      // PIC pin 37 -> LCD RS
sbit LCD_EN at RB5_bit;      // PIC pin 38 -> LCD E

// Data
sbit LCD_D4 at RB0_bit;      // PIC pin 33 -> LCD D4
sbit LCD_D5 at RB1_bit;      // PIC pin 34 -> LCD D5
sbit LCD_D6 at RB2_bit;      // PIC pin 35 -> LCD D6
sbit LCD_D7 at RB3_bit;      // PIC pin 36 -> LCD D7

// Pin directions (0 = output)
sbit LCD_RS_Direction at TRISB4_bit;
sbit LCD_EN_Direction at TRISB5_bit;
sbit LCD_D4_Direction at TRISB0_bit;
sbit LCD_D5_Direction at TRISB1_bit;
sbit LCD_D6_Direction at TRISB2_bit;
sbit LCD_D7_Direction at TRISB3_bit;

// 2. PROGRAM VARIABLES
unsigned int adcValue;
unsigned int temperatureC;           // Measured temperature in Celsius
unsigned int displayTemperature;     // Celsius or Fahrenheit for LCD
unsigned short fanOn = 0;      // Fan control state
unsigned short ledOn = 0;      // LED indicator state
char temperatureText[15];

// 3. MAIN
void main() {

    // 4MHz OSC1/OSC2
    // Oscillator mode: XT

    // Enable AN1 and AN2
    ANSEL = 0x06;   // AN1 = LM35, AN2 = LM335
    ANSELH = 0x00;  // Disable analogue inputs AN8-AN13

    // Disable comparators
    C1ON_bit = 0;
    C2ON_bit = 0;

    // Configure temperature sensor inputs
    TRISA1_bit = 1;  // PIC pin 3: RA1/AN1, LM35 output
    TRISA2_bit = 1;  // PIC pin 4: RA2/AN2, LM335 output

    // Fan-driver output (RC0, pin 15)
    TRISC0_bit = 0;
    RC0_bit = 0;
    
    // LED indicator output (RD0, pin 19)
    TRISD0_bit = 0;
    RD0_bit = 0;
    
    // DIP switch input
    TRISD7_bit = 1;  // PIC pin 30: RD7 input

    // Init ADC and LCD
    ADC_Init();
    Lcd_Init();
    Lcd_Cmd(_LCD_CLEAR);
    Lcd_Cmd(_LCD_CURSOR_OFF);

    // 4. CONTINUOUS CONTROL LOOP
    while(1) {

        // Read LM35 on AN1 (RA1, PIC pin 3)
        adcValue = ADC_Read(1);
        
        // Convert LM35 ADC reading to Celsius
        temperatureC = ((unsigned long)adcValue * 5000) / 1023;

       // 5. TEMP INDICATOR WITH HYSTERESIS
        // LED ON above 28 C, OFF below 26 C
       // 5. FAN CONTROL WITH HYSTERESIS
        if (adcValue >= 58) {
            fanOn = 1;
        }
        else if (adcValue <= 53) {
            fanOn = 0;
        }
        
        // LED mirrors the fan command
        ledOn = fanOn;
        
        // Update both outputs
        RC0_bit = fanOn;  // Fan-driver command, pin 15
        RD0_bit = ledOn;  // LED indicator, pin 19

        // 6. LCD TEMP DISPLAY (C/F SELECT)
         // DIP OFF (open), RD7 HIGH (1), temp .00 C 
        // DIP ON (CLOSED), RD7 LOW (0), temp .00 F
        // RD7 is connected to the DIP switch.
        // Assumes external pull-up: LOW = switch ON.
        
        // Select display units
        if (RD7_bit == 0) {
            // DIP switch ON: Fahrenheit
            displayTemperature = (temperatureC * 9 / 5) + 320;
            Lcd_Out(1, 13, "F");
        }
        else {
            // DIP switch OFF: Celsius
            displayTemperature = temperatureC;
            Lcd_Out(1, 13, "C");
        }
        
        // Update LCD
        Lcd_Out(1, 1, "Temp:       ");
        Lcd_Out(1, 7, temperatureText);
        
        // 7. LCD LED STATUS 
       
        if (fanOn == 1) {
            Lcd_Out(2, 1, "Fan: ON         ");
        }
        else {
            Lcd_Out(2, 1, "Fan: OFF        ");
        }

        Delay_ms(250);
    }
}
