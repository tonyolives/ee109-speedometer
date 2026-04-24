/********************************************
 *
 *  Name: Tony Olivares
 *  Email: tolivare@usc.edu
 *  Section: Friday 12:30PM
 *  Assignment: EE109 Speedometer – Checkpoint 1
 *
 ********************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdio.h>

#include "lcd.h"
#include "adc.h"

// ADC button thresholds (lab 7)
#define BTN_RIGHT 0
#define BTN_UP 51
#define BTN_DOWN 102
#define BTN_LEFT 156
#define BTN_SELECT 205
#define BTN_TOL 20 // ±tolerance for all comparisons

// EEPROM one byte at address 0 stores the speed threshold across power cycles.
#define EE_THRESH_ADDR ((uint8_t *)0x00)

// speed threshold limits - 1-99 cm/sec
#define THRESH_MIN 1
#define THRESH_MAX 99

/* LCD layout
   Row 0 (16 chars):  [0-4]  range1    [6-10] range2    [12-15] elapsed time
   Row 1 (16 chars):  [0-5]  speed     [7-8]  threshold [12-15] remote speed */
#define COL_R1 0
#define COL_R2 6
#define COL_TIME 12
#define COL_SPEED 0
#define COL_THRESH 7
#define COL_REMOTE 12

// encoder
volatile uint8_t enc_old_state;
volatile uint8_t enc_changed; // set to 1 by ISR; cleared by main loop

// threshold – uint8_t is naturally atomic on 8-bit AVR
volatile uint8_t threshold; // cm/sec, always clamped to 1-99

// speed measurement (set in later phases)
volatile int16_t speed_mm_sec; // signed mm/sec
volatile uint8_t speed_valid;  // 0 = no measurement yet / failed

// prototypes
void timer2_init(void);
void led_off_all(void);
void led_blue(void);
void led_green(void);
void led_red(void);
void update_led(void);
void show_threshold(void);

// LED helper functions (to make my life easier leater) - pull cathode LOW to light, HIGH to turn off
void led_off_all(void)
{
    PORTC |= (1 << PC1) | (1 << PC2) | (1 << PC3);
}

void led_blue(void)
{
    led_off_all();
    PORTC &= ~(1 << PC3);
}

void led_green(void)
{
    led_off_all();
    PORTC &= ~(1 << PC2);
}

void led_red(void)
{
    led_off_all();
    PORTC &= ~(1 << PC1);
}

// cmopare abs speed to threshold without division, speed is mm/sec and threshold is cm/sec to need 10 * threshold = mm/sec
void update_led(void)
{
    if (!speed_valid)
    {
        led_blue();
        return;
    }
    uint16_t mag = (speed_mm_sec < 0) ? (uint16_t)(-speed_mm_sec)
                                      : (uint16_t)(speed_mm_sec);
    if (mag > (uint16_t)threshold * 10)
        led_red();
    else
        led_green();
}

// threshold display
void show_threshold(void)
{
    char buf[4];
    snprintf(buf, 4, "%2d", (int)threshold);
    lcd_moveto(1, COL_THRESH);
    lcd_stringout(buf);
}

// TIMER2 - fast pwm for servo dial indidcator (OC2A = PB3), prescalar 1024 -> period = 16.4ms
// OCR2A controls pulse width = 12 -> 0.75ms
// OCR2A = 35 -> 2.25ms
void timer2_init(void)
{
    TCCR2A |= (0b11 << WGM20);  // fast PWM, modulus = 256
    TCCR2A |= (0b10 << COM2A0); // Clear OC2A at compare, set at BOTTOM
    OCR2A = 12;                 // Start at full-CW (no time elapsed yet)
    TCCR2B |= (0b111 << CS20);  // Prescaler = 1024
}

// encoder ISR - PCINT1 = port c (pin change interrupt), same state machine as lab 6, encoder now on PC4 and PC5
// only modifies threshold
ISR(PCINT1_vect)
{
    uint8_t bits = PINC;
    uint8_t a = bits & (1 << PC4);
    uint8_t b = bits & (1 << PC5);
    uint8_t new_state = enc_old_state;

    if (enc_old_state == 0)
    {
        if (a)
        {
            new_state = 1;
            threshold++;
        }
        else if (b)
        {
            new_state = 2;
            threshold--;
        }
    }
    else if (enc_old_state == 1)
    {
        if (!a)
        {
            new_state = 0;
            threshold--;
        }
        else if (b)
        {
            new_state = 3;
            threshold++;
        }
    }
    else if (enc_old_state == 2)
    {
        if (a)
        {
            new_state = 3;
            threshold--;
        }
        else if (!b)
        {
            new_state = 0;
            threshold++;
        }
    }
    else
    { // state 3
        if (!a)
        {
            new_state = 2;
            threshold++;
        }
        else if (!b)
        {
            new_state = 1;
            threshold--;
        }
    }

    if (new_state != enc_old_state)
    {
        // clamp inside ISR so threshold is always valid when main reads ite
        if (threshold < THRESH_MIN)
            threshold = THRESH_MIN;
        if (threshold > THRESH_MAX)
            threshold = THRESH_MAX;
        enc_old_state = new_state;
        enc_changed = 1; // tell main loop to update LCD/EEPROM
    }
}

// main
int main(void)
{
    uint8_t bits, a, b;

    // DDR: set outputs
    DDRB |= (1 << PB3) | (1 << PB4) | (1 << PB5);
    DDRC |= (1 << PC1) | (1 << PC2) | (1 << PC3);
    DDRD |= (1 << PD3);  // trigger = output
    DDRD &= ~(1 << PD2); // echo    = input

    // initial output states
    PORTB |= (1 << PB4);              // tri-state buffer disabled
    PORTB &= ~(1 << PB5);             // buzzer off
    PORTD &= ~(1 << PD3);             // trigger idle-low
    PORTC |= (1 << PC4) | (1 << PC5); // encoder pull-ups on
    led_off_all();                    // LEDs off during init

    // module initialisation
    lcd_init();
    adc_init();
    timer2_init();

    // splash screen
    lcd_writecommand(1);
    lcd_moveto(0, 1);
    lcd_stringout("Tony Olivares");
    lcd_moveto(1, 2);
    lcd_stringout("EE109 Project");
    _delay_ms(1500);
    lcd_writecommand(1);

    // static lcd labels
    // "T:" sits just left of where the 2-digit threshold number goes
    lcd_moveto(1, COL_THRESH - 2);
    lcd_stringout("T:");

    // laod threshold from EEPROM
    {
        uint8_t stored = eeprom_read_byte(EE_THRESH_ADDR);
        threshold = (stored >= THRESH_MIN && stored <= THRESH_MAX)
                        ? stored
                        : 20; // default 20 cm/sec on first use
    }
    show_threshold();

    // initial LED: blue = no measurement yet
    speed_valid = 0;
    update_led();

    // encoder: read initial physical state
    bits = PINC;
    a = bits & (1 << PC4);
    b = bits & (1 << PC5);
    if (!b && !a)
        enc_old_state = 0;
    else if (!b && a)
        enc_old_state = 1;
    else if (b && !a)
        enc_old_state = 2;
    else
        enc_old_state = 3;

    // enable encoder pin-change interrupts (PC4 and PC5)
    PCICR |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT4) | (1 << PCINT5);

    sei();

    // main loop
    while (1)
    {

        // encoder: update threshold display and EEPROM
        if (enc_changed)
        {
            enc_changed = 0;
            eeprom_write_byte(EE_THRESH_ADDR, threshold);
            show_threshold();
            update_led();
        }

        // read LCD buttons (placeholder - rangefinder added in Phase 2)
        uint8_t btn = adc_sample(0);

        // LEFT button (≈156) = "Start" initiates first range measurement
        if (btn > BTN_LEFT - BTN_TOL && btn < BTN_LEFT + BTN_TOL)
        {
            // phase 2 will trigger the rangefinder here
            _delay_ms(200); // debounce
        }

        // RIGHT button (≈0) = "Stop" – initiates second range measurement
        if (btn <= BTN_RIGHT + BTN_TOL)
        {
            // phase 2 will trigger the rangefinder here
            _delay_ms(200); // debounce
        }

        _delay_ms(20);
    }

    return 0; // never reached
}