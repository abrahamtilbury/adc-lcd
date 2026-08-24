// ===============================
// VARIABLES
// ===============================

unsigned int adc_value;
unsigned int delay_time;
char delay_txt[8];

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
// FULL STEP - CLOCKWISE
// ===============================

void FullStepCW()
{
    PORTD = 0b00000011;
    Vdelay_ms(delay_time);
    if(PORTC.F4 == 1) return;

    PORTD = 0b00000110;
    Vdelay_ms(delay_time);
    if(PORTC.F4 == 1) return;

    PORTD = 0b00001100;
    Vdelay_ms(delay_time);
    if(PORTC.F4 == 1) return;

    PORTD = 0b00001001;
    Vdelay_ms(delay_time);
}


// ===============================
// FULL STEP - COUNTER CLOCKWISE
// ===============================

void FullStepCCW()
{
    PORTD = 0b00001001;
    Vdelay_ms(delay_time);
    if(PORTC.F4 == 1) return;

    PORTD = 0b00001100;
    Vdelay_ms(delay_time);
    if(PORTC.F4 == 1) return;

    PORTD = 0b00000110;
    Vdelay_ms(delay_time);
    if(PORTC.F4 == 1) return;

    PORTD = 0b00000011;
    Vdelay_ms(delay_time);
}

// ===============================
// HALF STEP - CLOCKWISE
// ===============================

void HalfStepCW()
{
    PORTD = 0b00000001;
    Vdelay_ms(delay_time);
    if(PORTC.F4 == 1) return;

    PORTD = 0b00000011;
    Vdelay_ms(delay_time);
    if(PORTC.F4 == 1) return;

    PORTD = 0b00000010;
    Vdelay_ms(delay_time);
    if(PORTC.F4 == 1) return;

    PORTD = 0b00000110;
    Vdelay_ms(delay_time);
    if(PORTC.F4 == 1) return;

    PORTD = 0b00000100;
    Vdelay_ms(delay_time);
    if(PORTC.F4 == 1) return;

    PORTD = 0b00001100;
    Vdelay_ms(delay_time);
    if(PORTC.F4 == 1) return;

    PORTD = 0b00001000;
    Vdelay_ms(delay_time);
    if(PORTC.F4 == 1) return;

    PORTD = 0b00001001;
    Vdelay_ms(delay_time);
}


// ===============================
// HALF STEP - COUNTER CLOCKWISE
// ===============================

void HalfStepCCW()
{
    PORTD = 0b00001001;
    Vdelay_ms(delay_time);
    if(PORTC.F4 == 1) return;

    PORTD = 0b00001000;
    Vdelay_ms(delay_time);
    if(PORTC.F4 == 1) return;

    PORTD = 0b00001100;
    Vdelay_ms(delay_time);
    if(PORTC.F4 == 1) return;

    PORTD = 0b00000100;
    Vdelay_ms(delay_time);
    if(PORTC.F4 == 1) return;

    PORTD = 0b00000110;
    Vdelay_ms(delay_time);
    if(PORTC.F4 == 1) return;

    PORTD = 0b00000010;
    Vdelay_ms(delay_time);
    if(PORTC.F4 == 1) return;

    PORTD = 0b00000011;
    Vdelay_ms(delay_time);
    if(PORTC.F4 == 1) return;

    PORTD = 0b00000001;
    Vdelay_ms(delay_time);
}


// ===============================
// MAIN
// ===============================

void main()
{
    // ---------------------------
    // PORT CONFIGURATION
    // ---------------------------

    ANSEL = 0x04;       // AN2 = analog
    ANSELH = 0x00;      // Other analogue channels = digital

    C1ON_bit = 0;       // Disable comparator 1
    C2ON_bit = 0;       // Disable comparator 2

    TRISA = 0xFF;       // PORTA inputs
    TRISB = 0x00;       // LCD outputs
    TRISC = 0xFF;       // PORTC inputs - switches
    TRISD = 0x00;       // PORTD outputs - stepper

    PORTC = 0x00;
    PORTD = 0x00;

    ADC_Init();

    Lcd_Init();
    Lcd_Cmd(_LCD_CLEAR);
    Lcd_Cmd(_LCD_CURSOR_OFF);

    delay_time = 200;

    while(1)
    {
        // ===========================
        // READ ADC
        // ===========================
    
        adc_value = ADC_Read(2);
     
        // ===========================
        // SPEED CONTROL
        // ===========================
    
        if(PORTC.F7 == 1)
        {
            // Analog speed control
            delay_time = 500 - (((unsigned long)adc_value * 450) / 1023);
    
            if(delay_time < 50)
                delay_time = 50;
        }
        else
        {
            // Manual speed control
            if(PORTC.F5 == 1)
            {
                if(delay_time > 50)
                    delay_time = delay_time - 25;
            
                while(PORTC.F5 == 1);
                Delay_ms(20);
            }
            
            if(PORTC.F6 == 1)
            {
                if(delay_time < 500)
                    delay_time = delay_time + 25;
            
                while(PORTC.F6 == 1);
                Delay_ms(20);
            }
        }
    
    
        // ===========================
        // MOTOR CONTROL
        // ===========================
    
        if(PORTC.F4 == 1)
        {
            // SW5 - Halt
            PORTD = 0x00;
    
            Lcd_Cmd(_LCD_CLEAR);
            Lcd_Out(1,1,"MOTOR");
            Lcd_Out(2,1,"HALT");
        }
    
        else if(PORTC.F0 == 1)
        {
            // SW1 - Full step CW
            Lcd_Cmd(_LCD_CLEAR);
            Lcd_Out(1,1,"FULL STEP CW");
            
            WordToStr(delay_time, delay_txt);
            Lcd_Out(2,1,"DELAY:");
            Lcd_Out(2,7,delay_txt);
            Lcd_Out(2,13,"ms");
            
            FullStepCW();
        }
    
        else if(PORTC.F1 == 1)
        {
            // SW2 - Half step CW
            Lcd_Cmd(_LCD_CLEAR);
            Lcd_Out(1,1,"HALF STEP CW");
            
            WordToStr(delay_time, delay_txt);
            Lcd_Out(2,1,"DELAY:");
            Lcd_Out(2,7,delay_txt);
            Lcd_Out(2,13,"ms");
            
            HalfStepCW();
        }
    
        else if(PORTC.F2 == 1)
        {
            // SW3 - Full step CCW
          Lcd_Cmd(_LCD_CLEAR);
          Lcd_Out(1,1,"FULL STEP CCW");
          
          WordToStr(delay_time, delay_txt);
          Lcd_Out(2,1,"DELAY:");
          Lcd_Out(2,7,delay_txt);
          Lcd_Out(2,13,"ms");
          
          FullStepCCW();
        }
    
        else if(PORTC.F3 == 1)
        {
            // SW4 - Half step CCW
            Lcd_Cmd(_LCD_CLEAR);
            Lcd_Out(1,1,"HALF STEP CCW");
            
            WordToStr(delay_time, delay_txt);
            Lcd_Out(2,1,"DELAY:");
            Lcd_Out(2,7,delay_txt);
            Lcd_Out(2,13,"ms");
            
            HalfStepCCW();
        }
    
        else
        {
            // No motor mode selected
            PORTD = 0x00;
        }
    
    
        Delay_ms(50);
    }
}
