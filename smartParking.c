/*
 * Practical Assessment 8 - Smart Car Parking
 * mikroC PRO for PIC
 *
 * Create a project for your PIC and set the clock to match your board.
 * Select the Lcd library in Library Manager and disable the watchdog.
 * Power up with an empty car park. The count starts at 10 on reset.
 */

#ifndef USE_PIC16F877A
#define USE_PIC16F877A 0      // 0 = PIC16F887, 1 = PIC16F877A
#endif
#ifndef ENTRY_ONLY
#define ENTRY_ONLY 0          // 1 = Part A, 0 = Parts A and B
#endif
#ifndef ENTRY_BLOCKED_LEVEL
#define ENTRY_BLOCKED_LEVEL 1 // RA0 logic level when the beam is blocked
#endif
#ifndef EXIT_BLOCKED_LEVEL
#define EXIT_BLOCKED_LEVEL 1  // RA1 logic level when the beam is blocked
#endif

#define CAPACITY 10
#define STABLE_SAMPLES 3

// HD44780-compatible 16x2 LCD in 4-bit mode. Connect LCD R/W to GND.
sbit LCD_RS at RD0_bit;
sbit LCD_EN at RD1_bit;
sbit LCD_D4 at RD2_bit;
sbit LCD_D5 at RD3_bit;
sbit LCD_D6 at RD4_bit;
sbit LCD_D7 at RD5_bit;

sbit LCD_RS_Direction at TRISD0_bit;
sbit LCD_EN_Direction at TRISD1_bit;
sbit LCD_D4_Direction at TRISD2_bit;
sbit LCD_D5_Direction at TRISD3_bit;
sbit LCD_D6_Direction at TRISD4_bit;
sbit LCD_D7_Direction at TRISD5_bit;

typedef struct {
    unsigned short stableBlocked;
    unsigned short samples;
} BeamState;

// Treat each beam as blocked at startup so it must clear before it counts.
BeamState entryBeam = {1, 0};
#if !ENTRY_ONLY
BeamState exitBeam = {1, 0};
#endif

char countLine[17] = "Available:      ";

// Return 1 only when the beam changes from stably clear to stably blocked.
unsigned short beamInterrupted(BeamState *beam, unsigned short blocked) {
    if (blocked == beam->stableBlocked) {
        beam->samples = 0;
        return 0;
    }

    beam->samples++;
    if (beam->samples < STABLE_SAMPLES) {
        return 0;
    }

    beam->stableBlocked = blocked;
    beam->samples = 0;
    return blocked;
}

void displayAvailability(unsigned short available) {
    if (available >= 10) {
        countLine[11] = '0' + available / 10;
        countLine[12] = '0' + available % 10;
    } else {
        countLine[11] = '0' + available;
        countLine[12] = ' ';
    }

    if (available == 0) {
        PORTC = 0x02; // RC1 red on, RC0 green off
        Lcd_Out(2, 1, "Parking is Full ");
    } else {
        PORTC = 0x01; // RC0 green on, RC1 red off
        Lcd_Out(2, 1, "Spaces available");
    }
    Lcd_Out(1, 1, countLine);
}

void main() {
    unsigned short available;
    unsigned short previousAvailable;
    unsigned short sensorPins;
    unsigned short entryEvent;
#if !ENTRY_ONLY
    unsigned short exitEvent;
#endif

    // RA0 and RA1 are analog-capable pins. Configure them as digital inputs.
#if USE_PIC16F877A
    ADCON1 = 0x06;
    CMCON = 0x07;
#else
    ANSEL = 0;
    ANSELH = 0;
    CM1CON0 = 0;
    CM2CON0 = 0;
#endif
    TRISA = 0xFF;
    PORTC = 0;
    TRISC = 0xFC; // RC0 and RC1 are outputs
    PORTD = 0;
    TRISD = 0xC0; // RD0 through RD5 are outputs

    Delay_ms(50);
    Lcd_Init();
    Lcd_Cmd(_LCD_CLEAR);
    Lcd_Cmd(_LCD_CURSOR_OFF);

    available = CAPACITY;
    displayAvailability(available);

    while (1) {
        sensorPins = PORTA;
        previousAvailable = available;
        entryEvent = beamInterrupted(&entryBeam,
            (sensorPins & 0x01) == ENTRY_BLOCKED_LEVEL);

#if !ENTRY_ONLY
        exitEvent = beamInterrupted(&exitBeam,
            ((sensorPins >> 1) & 0x01) == EXIT_BLOCKED_LEVEL);

        // If both events arrive together, process the exit first.
        if (exitEvent && available < CAPACITY) {
            available++;
        }
#endif
        if (entryEvent && available > 0) {
            available--;
        }

        if (available != previousAvailable) {
            displayAvailability(available);
        }
        Delay_ms(10);
    }
}
