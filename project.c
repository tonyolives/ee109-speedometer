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

// TIMER1 prescaler and watchdog for pulse-width measurement
// prescaler 8: ticks at 16MHz/8 = 2MHz (0.5us per tick)
// max echo = ~23.5ms (400cm round trip) = ~47059 ticks - watchdog set above that
#define T1_PRESCALER_BITS ((1 << CS11)) // prescaler = 8
#define T1_WATCHDOG 50000               // ~25ms at 2MHz, safely above max echo

// state machine states
#define STATE_IDLE 0       // waiting for START (LEFT button)
#define STATE_MEASURING1 1 // first echo in progress
#define STATE_TIMING 2     // stopwatch running, waiting for STOP (RIGHT button)
#define STATE_MEASURING2 3 // second echo in progress

// encoder
volatile uint8_t enc_old_state;
volatile uint8_t enc_changed; // set to 1 by ISR; cleared by main loop

// threshold – uint8_t is naturally atomic on 8-bit AVR
volatile uint8_t threshold; // cm/sec, always clamped to 1-99

// speed measurement
volatile int16_t speed_mm_sec; // signed mm/sec
volatile uint8_t speed_valid;  // 0 = no measurement yet / failed

// rangefinder
volatile uint8_t echo_state; // 0 = idle, 1 = waiting for falling edge
volatile uint16_t range1_mm;
volatile uint8_t range1_valid;
volatile uint16_t range2_mm;
volatile uint8_t range2_valid;
volatile uint8_t meas_done; // set by ISR when echo measurement finishes or fails

// stopwatch (TIMER1 in CTC mode between measurements)
volatile uint8_t elapsed_tenths;    // counts 0.1s ticks, max 100 (= 10.0s)
volatile uint8_t time_changed;      // set by stopwatch ISR each tick
volatile uint8_t stopwatch_timeout; // set by stopwatch ISR at 10.0s

// overall state
volatile uint8_t state = STATE_IDLE;

// prototypes
void timer1_setup_pulse(void);
void timer1_start_stopwatch(void);
void timer2_init(void);
void trigger_sensor(void);
void show_range1(void);
void show_range2(void);
void show_elapsed(void);
void show_speed(void);
void clear_display_fields(void);
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

// display range1 in cm with one decimal at upper-left (e.g. " 27.6")
void show_range1(void)
{
    char buf[7];
    uint16_t cm_int = range1_mm / 10;
    uint16_t cm_frac = range1_mm % 10;
    snprintf(buf, 7, "%3u.%1u", cm_int, cm_frac);
    lcd_moveto(0, COL_R1);
    lcd_stringout(buf);
}

// display range2 in cm with one decimal in the middle of the top line (e.g. " 27.6")
void show_range2(void)
{
    char buf[7];
    uint16_t cm_int = range2_mm / 10;
    uint16_t cm_frac = range2_mm % 10;
    snprintf(buf, 7, "%3u.%1u", cm_int, cm_frac);
    lcd_moveto(0, COL_R2);
    lcd_stringout(buf);
}

// display elapsed time in seconds with one decimal at upper-right (e.g. " 7.5")
void show_elapsed(void)
{
    char buf[5];
    uint8_t secs = elapsed_tenths / 10;
    uint8_t frac = elapsed_tenths % 10;
    snprintf(buf, 5, "%2u.%1u", secs, frac);
    lcd_moveto(0, COL_TIME);
    lcd_stringout(buf);
}

// display speed in cm/sec with one decimal at lower-left (e.g. "  3.6" or " -3.6")
// speed_mm_sec / 10 gives cm/sec integer part (sign preserved)
void show_speed(void)
{
    char buf[8];
    int16_t cm_int = speed_mm_sec / 10;
    uint16_t cm_frac = (speed_mm_sec < 0 ? (uint16_t)(-speed_mm_sec)
                                         : (uint16_t)(speed_mm_sec)) %
                       10;
    snprintf(buf, 8, "%4d.%1u", cm_int, cm_frac);
    lcd_moveto(1, COL_SPEED);
    lcd_stringout(buf);
}

// clear range2, elapsed time, and speed fields for a fresh measurement cycle
void clear_display_fields(void)
{
    lcd_moveto(0, COL_R2);
    lcd_stringout("     ");
    lcd_moveto(0, COL_TIME);
    lcd_stringout("    ");
    lcd_moveto(1, COL_SPEED);
    lcd_stringout("      ");
}

// configure TIMER1 for free-running pulse-width measurement (normal mode)
// does NOT start the timer - echo ISR starts it on rising edge
void timer1_setup_pulse(void)
{
    TCCR1B = 0; // stop timer, clear all mode bits
    TCCR1A = 0; // normal mode
    TCNT1 = 0;
    OCR1A = T1_WATCHDOG; // watchdog fires if echo never arrives
    TIMSK1 |= (1 << OCIE1A);
}

// configure and start TIMER1 as a 0.1s stopwatch (CTC mode, prescaler 64)
void timer1_start_stopwatch(void)
{
    TCCR1B = 0; // stop any running timer first
    TCCR1A = 0;
    TCNT1 = 0;
    OCR1A = 24999; // 16MHz / 64 / 25000 = 10Hz = 0.1s interval
    elapsed_tenths = 0;
    time_changed = 0;
    stopwatch_timeout = 0;
    TIMSK1 |= (1 << OCIE1A);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC, prescaler 64, starts now
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

// send a 10us trigger pulse to start a rangefinder measurement
void trigger_sensor(void)
{
    PORTD |= (1 << PD3);
    _delay_us(10);
    PORTD &= ~(1 << PD3);
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

// echo pin ISR - PCINT2 = port d (pin change interrupt on PD2)
// rising edge: start TIMER1 free-running to measure pulse width
// falling edge: stop TIMER1, convert count to mm, signal done
// uses state to know which range variable to write to
ISR(PCINT2_vect)
{
    if (PIND & (1 << PD2))
    {
        // rising edge - start of echo pulse
        TCNT1 = 0;
        TCCR1B = T1_PRESCALER_BITS; // start free-running, prescaler 8
        echo_state = 1;
    }
    else
    {
        // falling edge - end of echo pulse
        if (echo_state == 1)
        {
            TCCR1B = 0; // stop timer
            uint16_t count = TCNT1;

            // convert count to distance in mm
            // prescaler 8 -> 2MHz -> 0.5us per tick
            // distance_mm = count * 0.5us * 340000mm/s / 2 = count * 17 / 200
            uint16_t dist_mm = (uint32_t)count * 17 / 200;

            if (state == STATE_MEASURING1)
            {
                range1_mm = dist_mm;
                range1_valid = 1;
            }
            else if (state == STATE_MEASURING2)
            {
                range2_mm = dist_mm;
                range2_valid = 1;
            }

            echo_state = 0;
            meas_done = 1;
        }
    }
}

// TIMER1 compare-A ISR - two roles depending on state:
// STATE_TIMING:    stopwatch tick (increment elapsed_tenths, check 10s timeout)
// all other states: watchdog (echo took too long, abort measurement)
ISR(TIMER1_COMPA_vect)
{
    if (state == STATE_TIMING)
    {
        // stopwatch tick
        elapsed_tenths++;
        time_changed = 1;
        if (elapsed_tenths >= 100)
        {
            // 10.0s elapsed without a second measurement - give up
            TCCR1B = 0;
            stopwatch_timeout = 1;
        }
    }
    else
    {
        // watchdog - echo pulse never fell, abort this measurement
        TCCR1B = 0;
        echo_state = 0;
        meas_done = 1; // range_valid was already cleared before trigger
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
    timer1_setup_pulse(); // TIMER1 ready for first echo measurement

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

    // enable encoder pin-change interrupts on PC4 and PC5
    PCICR |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT4) | (1 << PCINT5);

    // enable echo pin-change interrupt on PD2
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT18);

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

        // stopwatch timeout: 10s elapsed with no second measurement
        if (stopwatch_timeout)
        {
            stopwatch_timeout = 0;
            state = STATE_IDLE;
            speed_valid = 0;
            update_led(); // back to blue
            OCR2A = 12;   // servo back to start
            lcd_moveto(0, COL_TIME);
            lcd_stringout("    "); // clear elapsed time
        }

        // update elapsed time display and servo dial while stopwatch is running
        if (state == STATE_TIMING && time_changed)
        {
            time_changed = 0;
            show_elapsed();
            // servo rotates from full-CCW (35) to full-CW (12) over 10 seconds
            // OCR2A = 35 - elapsed_tenths * 23 / 100
            OCR2A = (uint8_t)(35 - (uint16_t)elapsed_tenths * 23 / 100);
        }

        // read LCD buttons
        uint8_t btn = adc_sample(0);

        // LEFT button (~156) = "Start" - only valid in IDLE state
        if (state == STATE_IDLE &&
            btn > BTN_LEFT - BTN_TOL && btn < BTN_LEFT + BTN_TOL)
        {

            // clear stale display fields from previous cycle
            clear_display_fields();

            // fire first measurement
            state = STATE_MEASURING1;
            meas_done = 0;
            range1_valid = 0;
            echo_state = 0;
            timer1_setup_pulse();
            trigger_sensor();

            // wait for ISR to finish (software timeout ~60ms as backup)
            uint16_t timeout = 0;
            while (!meas_done && timeout < 60000)
            {
                timeout++;
                _delay_us(1);
            }

            if (range1_valid)
            {
                show_range1();
                // start the stopwatch and move to TIMING state
                state = STATE_TIMING;
                timer1_start_stopwatch();
            }
            else
            {
                // measurement failed - show dashes and return to idle
                lcd_moveto(0, COL_R1);
                lcd_stringout(" ---");
                state = STATE_IDLE;
                speed_valid = 0;
                update_led();
            }

            _delay_ms(200); // debounce
        }

        // RIGHT button (~0) = "Stop" - only valid while stopwatch is running
        if (state == STATE_TIMING && btn <= BTN_RIGHT + BTN_TOL)
        {

            // stop the stopwatch and save elapsed time
            TCCR1B = 0;
            uint8_t et = (elapsed_tenths > 0) ? elapsed_tenths : 1; // avoid /0

            // fire second measurement
            state = STATE_MEASURING2;
            meas_done = 0;
            range2_valid = 0;
            echo_state = 0;
            timer1_setup_pulse();
            trigger_sensor();

            // wait for ISR to finish
            uint16_t timeout = 0;
            while (!meas_done && timeout < 60000)
            {
                timeout++;
                _delay_us(1);
            }

            if (range2_valid)
            {
                show_range2();

                // calculate speed in mm/sec then display as cm/sec
                // speed = (range2 - range1) * 10 / elapsed_tenths  (mm/sec)
                // positive = moving away, negative = moving toward
                int16_t diff = (int16_t)range2_mm - (int16_t)range1_mm;
                speed_mm_sec = (int16_t)((int32_t)diff * 10 / et);
                speed_valid = 1;
                show_speed();
                update_led();
            }
            else
            {
                // second measurement failed
                lcd_moveto(0, COL_R2);
                lcd_stringout(" ---");
                speed_valid = 0;
                update_led();
            }

            OCR2A = 12; // servo back to start position
            state = STATE_IDLE;
            _delay_ms(200); // debounce
        }

        _delay_ms(20);
    }

    return 0; // never reached
}