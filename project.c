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

// TIMER1 prescaler and watchdog
// using prescaler 8: timer ticks at 16MHz/8 = 2MHz (0.5us per tick)
// sensor max range = 400cm, max echo = ~23.2ms -> max ticks = 46400 (fits in 16-bit)
// watchdog: if TCNT1 reaches this value something went wrong, abort measurement
#define T1_PRESCALER_BITS ((1 << CS11)) // prescaler = 8
#define T1_WATCHDOG 50000               // ~25ms at 2MHz, safely above max echo

// encoder
volatile uint8_t enc_old_state;
volatile uint8_t enc_changed; // set to 1 by ISR; cleared by main loop

// threshold – uint8_t is naturally atomic on 8-bit AVR
volatile uint8_t threshold; // cm/sec, always clamped to 1-99

// speed measurement (set in later phases)
volatile int16_t speed_mm_sec; // signed mm/sec
volatile uint8_t speed_valid;  // 0 = no measurement yet / failed

// rangefinder state
// range values stored in mm for precision (displayed as cm with 1 decimal)
volatile uint8_t echo_state;   // 0 = waiting for rising edge, 1 = waiting for falling
volatile uint16_t range1_mm;   // first range measurement in mm
volatile uint8_t range1_valid; // 1 = range1 has a good reading
volatile uint8_t meas_done;    // set by ISR when measurement completes or fails

// prototypes
void timer1_init_pulse(void);
void timer2_init(void);
void trigger_sensor(void);
void show_range1(void);
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

// display range1 in cm with one decimal at upper-left of LCD
// range is stored in mm, e.g. 276mm -> "27.6" at row 0 col 0
void show_range1(void)
{
    char buf[7];
    uint16_t cm_int = range1_mm / 10;  // integer part of cm
    uint16_t cm_frac = range1_mm % 10; // fractional digit
    snprintf(buf, 7, "%3u.%1u", cm_int, cm_frac);
    lcd_moveto(0, COL_R1);
    lcd_stringout(buf);
}

// init TIMER1 for free-running pulse-width measurement
// does NOT start the timer - timer starts in the echo ISR on rising edge
void timer1_init_pulse(void)
{
    TCCR1A = 0; // normal mode (no output compare pins)
    TCCR1B = 0; // timer stopped for now
    TCNT1 = 0;

    // watchdog: interrupt if count reaches T1_WATCHDOG (runaway echo)
    OCR1A = T1_WATCHDOG;
    TIMSK1 |= (1 << OCIE1A); // enable compare-A interrupt
}

// send a 10us trigger pulse to the rangefinder to start a measurement
void trigger_sensor(void)
{
    PORTD |= (1 << PD3); // trigger high
    _delay_us(10);
    PORTD &= ~(1 << PD3); // trigger low
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

// echo pin ISR - PCINT2 = port d (pin change interrupt on PD2)
// rising edge: start TIMER1 free-running
// falling edge: stop TIMER1, convert count to mm, signal done
ISR(PCINT2_vect)
{
    if (PIND & (1 << PD2))
    {
        // rising edge - start of echo pulse, reset and start TIMER1
        TCNT1 = 0;
        TCCR1B = T1_PRESCALER_BITS; // start timer, normal mode
        echo_state = 1;
    }
    else
    {
        // falling edge - end of echo pulse
        if (echo_state == 1)
        {
            TCCR1B = 0; // stop timer
            uint16_t count = TCNT1;

            // convert count to mm
            // at prescaler 8: each tick = 0.5us
            // sound speed = 340 m/s = 0.034 cm/us = 0.34 mm/us
            // distance = (count * 0.5us * 0.34 mm/us) / 2  (divide by 2 for round trip)
            // = count * 0.5 * 0.34 / 2 = count * 0.085 mm
            // avoid floats: distance_mm = count * 17 / 200
            range1_mm = (uint32_t)count * 17 / 200;
            range1_valid = 1;
            meas_done = 1;
            echo_state = 0;
        }
    }
}

// TIMER1 compare-A ISR - watchdog: echo pulse took too long, abort measurement
ISR(TIMER1_COMPA_vect)
{
    TCCR1B = 0; // stop timer
    echo_state = 0;
    range1_valid = 0;
    meas_done = 1; // signal main loop that measurement ended (failed)
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
    timer1_init_pulse();

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

        // read LCD buttons
        uint8_t btn = adc_sample(0);

        // LEFT button (~156) = "Start" - trigger first range measurement
        if (btn > BTN_LEFT - BTN_TOL && btn < BTN_LEFT + BTN_TOL)
        {
            // reset measurement state and fire the sensor
            meas_done = 0;
            range1_valid = 0;
            echo_state = 0;
            TCNT1 = 0;
            trigger_sensor();

            // wait for ISR to signal measurement done (with a timeout in case
            // the echo interrupt never fires at all)
            uint16_t timeout = 0;
            while (!meas_done && timeout < 60000)
            {
                timeout++;
                _delay_us(1);
            }

            // display result or clear the field on failure
            if (range1_valid)
            {
                show_range1();
            }
            else
            {
                lcd_moveto(0, COL_R1);
                lcd_stringout(" --- ");
            }

            _delay_ms(200); // debounce
        }

        // RIGHT button (~0) = "Stop" – second measurement added in Phase 3
        if (btn <= BTN_RIGHT + BTN_TOL)
        {
            // phase 3 will trigger the second rangefinder measurement here
            _delay_ms(200); // debounce
        }

        _delay_ms(20);
    }

    return 0; // never reached
}