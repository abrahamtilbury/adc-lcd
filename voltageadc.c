unsigned int temp_res;
float voltage;
char txt[16];

// LCD module connections
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
// End LCD module connections

void main() {
  ANSEL  = 0x04;              // Configure AN2 pin as analog
  ANSELH = 0;                 // Configure other AN pins as digital I/O
  C1ON_bit = 0;               // Disable comparators
  C2ON_bit = 0;
  
  TRISA  = 0xFF;              // PORTA is input
  TRISC  = 0;                 // PORTC is output
  TRISB  = 0;                 // PORTB is output

  ADC_Init();
  
  Lcd_Init();
  Lcd_Cmd(_LCD_CLEAR);
  Lcd_Cmd(_LCD_CURSOR_OFF);

  Lcd_Out(1,1,"Voltage");
  
  do {
    Delay_us(20);

    temp_res = ADC_Read(2);   // Read ADC value (0-1023)

    voltage = (temp_res * 5.0) / 1023.0;   // Convert to volts

    FloatToStr(voltage, txt);

    Lcd_Out(2,1,"                ");   // Clear old value
    Lcd_Out(2,1,txt);                 // Display voltage
    Lcd_Out(2,8,"V");                 // Display unit

    Delay_ms(200);

  } while(1);
}
