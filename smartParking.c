/* Practical Assessment 8 - Smart Car Parking
 * PIC16F887, mikroC PRO for PIC, 4 MHz clock
 * Enable the LCD library; disable the watchdog.
 * Disable LVP if using high-voltage ICSP (RB3 is LCD D7).
 * Start with an empty car park.
 */

#define ENTRY_ONLY 0          // 1 = Part A, 0 = Parts A and B
#define CAPACITY 10
#define STABLE_SAMPLES 3      // Consecutive readings before accepting a change
#define ENTRY_BLOCKED_LEVEL 1 // Confirm with voltage measurements on RA0
#define EXIT_BLOCKED_LEVEL 1  // Confirm with voltage measurements on RA1

// LCD connections on the lecturer's circuit (4-bit mode).
// LCD R/W is connected to ground.
sbit LCD_RS at RB4_bit;
sbit LCD_EN at RB5_bit;
sbit LCD_D4 at RB0_bit;
sbit LCD_D5 at RB1_bit;
sbit LCD_D6 at RB2_bit;
sbit LCD_D7 at RB3_bit;

// Direction registers for the LCD pins.
sbit LCD_RS_Direction at TRISB4_bit;
sbit LCD_EN_Direction at TRISB5_bit;
sbit LCD_D4_Direction at TRISB0_bit;
sbit LCD_D5_Direction at TRISB1_bit;
sbit LCD_D6_Direction at TRISB2_bit;
sbit LCD_D7_Direction at TRISB3_bit;

// Keep a separate detection state for each sensor.
typedef struct {
    unsigned short stableBlocked; // Last confirmed beam state
    unsigned short samples;       // Consecutive changed readings
} BeamState;

// Require a clear beam before counting the first vehicle.
BeamState entryBeam = {1, 0};
#if !ENTRY_ONLY
BeamState exitBeam = {1, 0};
#endif

char countLine[17] = "Available:      "; // 16 LCD characters plus terminator

// Return 1 only for a confirmed clear-to-blocked transition.
unsigned short beamInterrupted(BeamState *beam, unsigned short blocked) {
    if (blocked == beam->stableBlocked) {
        beam->samples = 0; // No change, or a brief disturbance
        return 0;
    }

    beam->samples++;
    if (beam->samples < STABLE_SAMPLES) {
        return 0; // Wait for a stable reading
    }

    beam->stableBlocked = blocked;
    beam->samples = 0;
    return blocked; // Clearing the beam does not count a vehicle
}

// Update the LCD and parking status LEDs.
void displayAvailability(unsigned short available) {
    // Convert the count to LCD characters and erase any old second digit.
    if (available >= 10) {
        countLine[11] = '0' + available / 10;
        countLine[12] = '0' + available % 10;
    } else {
        countLine[11] = '0' + available;
        countLine[12] = ' ';
    }

    Lcd_Out(1, 1, countLine);
    if (available == 0) {
        PORTD = 0x01; // RD0 red on, RD1 yellow off
        Lcd_Out(2, 1, "Parking is Full ");
    } else {
        PORTD = 0x02; // RD0 red off, RD1 yellow on
        Lcd_Out(2, 1, "Spaces available");
    }
}

// Configure the PIC16F887 for the supplied circuit.
void setupHardware() {
    ANSEL = 0;   // Disable analogue inputs AN0-AN7
    ANSELH = 0;  // Disable analogue inputs AN8-AN13
    CM1CON0 = 0; // Disable comparator 1
    CM2CON0 = 0; // Disable comparator 2

    TRISA = 0xFF; // RA0 and RA1 are sensor inputs
    PORTB = 0;
    TRISB = 0xC0; // RB0-RB5 outputs; RB6/RB7 left as inputs
    PORTD = 0;
    TRISD = 0xFC; // RD0 and RD1 outputs; other pins inputs

    Delay_ms(50); // Allow the LCD to power up
    Lcd_Init();
    Lcd_Cmd(_LCD_CLEAR);
    Lcd_Cmd(_LCD_CURSOR_OFF);
}

void main() {
    unsigned short available = CAPACITY;
    unsigned short previousAvailable;
    unsigned short sensorPins;
    unsigned short entryEvent;
#if !ENTRY_ONLY
    unsigned short exitEvent;
#endif

    setupHardware();
    displayAvailability(available);

    while (1) {
        sensorPins = PORTA; // Read both sensors from the same port sample
        previousAvailable = available;

        // RA0 is the entry sensor (bit 0 of PORTA).
        entryEvent = beamInterrupted(&entryBeam,
            (sensorPins & 0x01) == ENTRY_BLOCKED_LEVEL);

#if !ENTRY_ONLY
        // RA1 is the exit sensor (bit 1 of PORTA).
        exitEvent = beamInterrupted(&exitBeam,
            ((sensorPins >> 1) & 0x01) == EXIT_BLOCKED_LEVEL);

        // Process exits first; never exceed the car park capacity.
        if (exitEvent && available < CAPACITY) {
            available++;
        }
#endif
        // Count an entry only if a space remains.
        if (entryEvent && available > 0) {
            available--;
        }

        // Refresh the display only when the number changes.
        if (available != previousAvailable) {
            displayAvailability(available);
        }
        Delay_ms(10); // Pause between sensor readings
    }
}
