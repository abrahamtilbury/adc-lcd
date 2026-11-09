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
unsigned int temperatureC;          // Celsius temperature multiplied by 10 (e.g. 283 = 28.3 C)
unsigned int displayTemperature;    // Selected C/F temperature multiplied by 10 for LCD
unsigned int adcLM335;              // 	Raw 10-bit reading from AN2
signed int temperatureLM335C;       // LM335 temperature in Celsius × 10
signed int displayLM335;            // LM335 temperature in the selected display unit × 10
unsigned short fanOn = 0;           // Fan control state
unsigned short ledOn = 0;           // LED indicator state
char temperatureText[15];
unsigned short lm335Fault = 0;

void FormatTenths(signed int value, char *out) {
    unsigned int magnitude;

    magnitude = (value < 0) ? (unsigned int)(-value) : (unsigned int)value;

    out[0] = (value < 0) ? '-' : ' ';
    out[1] = (magnitude >= 1000) ? ((magnitude / 1000) % 10) + '0' : ' ';
    out[2] = (magnitude >= 100) ? ((magnitude / 100) % 10) + '0' : ' ';
    out[3] = ((magnitude / 10) % 10) + '0';
    out[4] = '.';
    out[5] = (magnitude % 10) + '0';
    out[6] = '\0';
}

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
    TRISA1_bit = 1;      // PIC pin 3: RA1/AN1, LM35 output
    TRISA2_bit = 1;      // PIC pin 4: RA2/AN2, LM335 output

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
    Lcd_Out(1, 1, "35:");
    Lcd_Out(2, 1, "335:");

    // 4. CONTINUOUS CONTROL LOOP
    while(1) {

        // Read both sensors: LM35 on AN1, LM335 on AN2
        adcValue = ADC_Read(1);     // LM35 on AN1
        adcLM335 = ADC_Read(2);     // LM335 on AN2
        
        // Convert LM35 ADC reading to Celsius
        temperatureC = ((unsigned long)adcValue * 5000) / 1023;

        // Convert LM335 ADC reading from Kelvin to Celsius x 10
        temperatureLM335C =
            (signed int)(((unsigned long)adcLM335 * 5000) / 1023) - 2732;

        // Check LM335 reading against its operating range
        lm335Fault = (adcLM335 < 477) || (adcLM335 > 764);
        
        // 5. FAN CONTROL WITH HYSTERESIS

        // Sensor fault: command fan ON
        if (lm335Fault == 1) {
            fanOn = 1;
        }
        // Either sensor reaches 28.0 C
        else if ((temperatureC >= 280) || (temperatureLM335C >= 280)) {
            fanOn = 1;
        }
        // Both sensors fall below 26.0 C
        else if ((temperatureC < 260) && (temperatureLM335C < 260)) {
            fanOn = 0;
        }
        
        // LED mirrors the fan command
        ledOn = fanOn;
        
        // Update both outputs
        RC0_bit = fanOn;  // Fan-driver command, pin 15
        RD0_bit = ledOn;  // LED indicator, pin 19

        // 6. LCD TEMP DISPLAY (C/F SELECT)
        // DIP OFF (open), RD7 HIGH (1), display temperature as 0.0 C
        // DIP ON (closed), RD7 LOW (0), display temperature as 0.0 F
        // RD7 is connected to the DIP switch.
        // Assumes external pull-up: LOW = switch ON.
        
        // Select display units for BOTH sensors
        if (RD7_bit == 0) {
            displayTemperature = (temperatureC * 9 / 5) + 320;
            displayLM335 = (temperatureLM335C * 9 / 5) + 320;
        
            Lcd_Chr(1, 11, 'F');
            Lcd_Chr(2, 12, 'F');
        }
        else {
            displayTemperature = temperatureC;
            displayLM335 = temperatureLM335C;
        
            Lcd_Chr(1, 11, 'C');
            Lcd_Chr(2, 12, 'C');
        }

        // Display LM35 temperature on row 1
        FormatTenths((signed int)displayTemperature, temperatureText);
        Lcd_Out(1, 5, temperatureText);
        
        // Display LM335 temperature or fault
        if (lm335Fault == 1) {
            Lcd_Out(2, 6, "FAULT  ");
        }
        else {
            FormatTenths(displayLM335, temperatureText);
            Lcd_Out(2, 6, temperatureText);
        }
        
        // 7. LCD FAN STATUS 
    
        // Fan status at the end of LCD row 1
        if (fanOn == 1) {
            Lcd_Out(1, 13, "ON ");
        }
        else {
            Lcd_Out(1, 13, "OFF");
        }

        Delay_ms(250);
    }
}
