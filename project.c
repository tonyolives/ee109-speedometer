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

#define FOSC 16000000UL               // 16mhz arduino clock
#define BAUD 9600                     // baud rate from project spec
#define MYUBRR (FOSC / 16 / BAUD - 1) // compile-time computed: 103 for 9600

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

// remote serial receive - state for the @<digits>$ ===
// keeping all of these volatile because the rx isr writes them and main reads them
volatile char rx_buf[6];        // up to 5 chars (- and 4 digits) + null terminator
volatile uint8_t rx_count;      // how many valid chars in rx_buf so far
volatile uint8_t rx_started;    // 1 once '@' has been seen, 0 otherwise
volatile uint8_t rx_valid;      // 1 once a complete '@...$' packet is in rx_buf
volatile uint8_t remote_valid;  // we have ever received at least one valid speed
volatile int16_t remote_mm_sec; // last received remote speed in mm/sec

// buzzer state : the timer0 isr reads these to know what tone to play
// each note has a "pitch" value (ocr) and a "duration" value (cnt = number of toggles)
// these arrays hold the 3 notes of the current sequence; main loop fills them.

// numbers below were computed once with these formulas, at 16mhz with prescaler 64:
// ocr value for freq f hz  = 250000 / (2 * f) - 1
// toggle count for 350 ms  = 350 * 2 * f / 1000

// c5 = 523 hz = ocr 238 = 366 toggles
// e5 = 659 hz = ocr 188 = 461 toggles
// g5 = 784 hz = ocr 158 = 548 toggles

// frequencies chosen because lab 6 used multiples of 50 hz (400-1600), so musical notes c5/e5/g5 are clearly different and form a c major triad and my music nerd firend is with me
volatile uint8_t buzz_ocr[3];  // pitch (ocr0a) for each of the 3 notes
volatile uint16_t buzz_cnt[3]; // toggle count for each note (controls duration)
volatile uint8_t buzz_idx;     // which note we're currently on (0, 1, or 2)
volatile uint16_t buzz_count;  // toggles remaining in current note

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
void usart_init(void);
void serial_putchar(char c);
void send_speed(int16_t mm_sec);
void show_remote_speed(int16_t mm_sec);
void timer0_init(void);
void play_ascending(void);
void play_descending(void);

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

// TIMER2 - fast pwm for servo dial indidcator (OC2A = PB3)
// prescalar 1024 -> one tick = 64 microsec, period = 256 ticks = 16.384ms
// 0.75ms pulse -> right -> OCR2A = 12 (10 secs elapsed)
// 2.25ms pulse -> left -> OCR2A = 35 (0 secs elapsed)
void timer2_init(void)
{
    TCCR2A |= (0b11 << WGM20);  // fast PWM, modulus = 256
    TCCR2A |= (0b10 << COM2A0); // clear OC2A at compare, set at BOTTOM
    OCR2A = 35;                 // start at full left (no time elapsed yet)
    TCCR2B |= (0b111 << CS20);  // prescaler = 1024 (longest poss period)
}

// configure timer0 in ctc mode for buzzer tone generation, but DO NOT star
// the timer only runs while a tone is sounding (started by play_ascending / play_descending
// stopped by the isr when the sequence finishes
void timer0_init(void)
{
    TCCR0A = (1 << WGM01);   // ctc mode: timer counts 0..ocr0a then resets
    TIMSK0 |= (1 << OCIE0A); // enable interrupt (fires the isr)
    // tccr0b stays 0 -> prescaler bits clear -> timer is stopped until we set them
}

// kick off the ascending 3-note sequence: c5 -> e5 -> g5
// played when |remote speed| > threshold (remote board is faster than our threshold)
void play_ascending(void)
{
    TCCR0B = 0;           // stop any tone already in progress
    PORTB &= ~(1 << PB5); // make sure buzzer pin starts low

    // load the 3 notes from low to high pitch
    buzz_ocr[0] = 238;
    buzz_cnt[0] = 366; // c5 (low)
    buzz_ocr[1] = 188;
    buzz_cnt[1] = 461; // e5 (mid)
    buzz_ocr[2] = 158;
    buzz_cnt[2] = 548; // g5 (high)

    // start the first note - isr will handle the rest in the background
    buzz_idx = 0;
    OCR0A = buzz_ocr[0];                // sets pitch
    buzz_count = buzz_cnt[0];           // sets duration
    TCNT0 = 0;                          // start counting from zero
    TCCR0B = (1 << CS01) | (1 << CS00); // prescaler 64 jus starts timer0
}

// kick off the descending 3 note sequence: g5 -> e5 -> c5
// played when |remote speed| <= threshold (remote is at or below our threshold)
void play_descending(void)
{
    TCCR0B = 0;
    PORTB &= ~(1 << PB5);

    // load the 3 notes from high to low pitch
    buzz_ocr[0] = 158;
    buzz_cnt[0] = 548; // g5 (high)
    buzz_ocr[1] = 188;
    buzz_cnt[1] = 461; // e5 (mid)
    buzz_ocr[2] = 238;
    buzz_cnt[2] = 366; // c5 (low)

    buzz_idx = 0;
    OCR0A = buzz_ocr[0];
    buzz_count = buzz_cnt[0];
    TCNT0 = 0;
    TCCR0B = (1 << CS01) | (1 << CS00);
}

// SEE SLIDES on serial interfaces for info on how to configure USART0 module with given settings
// configure usart0 for 9600 baud rate, 8 data bits, no parity, 1 stop bit, async
// also enables rx-complete interrupt so received bytes go through ISR(USART_RX_vect)
void usart_init(void)
{
    // baud rate (103)
    UBRR0 = MYUBRR;

    // note here: we need baud rate bc we dont have a clock signal. so the receiver has to know when to sample the line, jus tby
    // seeing start but it samples the line every 1/baud seconds to recover each bit

    // ucsr0b: enable rx, tx, and rx-complete interrupt
    // can check every time through main loop for character, but can get stuck waiting if half was sent. better solution is to
    // use interrupts for the received data - receiver interrupts are enabled by setting the RXCIE0 bit to a one
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

    // ucsr0c: 8 data bits = ucsz01:00 = 11
    // other bits default to zero -> async, no parity, 1 stop bit (8n1)
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// transmit one character via usart - polling style from slides
// blocks until the transmit data register is empty, then loads the byte
// udre0 = 1 means "data register empty - safe to write next byte to udr0"
void serial_putchar(char ch)
{
    while ((UCSR0A & (1 << UDRE0)) == 0)
    {
    } // wait for tx buffer to be free
    UDR0 = ch; // writing udr0 kicks off transmission
}

// send the local speed as "@<value>$"
// example: 17.4 cm/sec = "@174$"
// snprintf %d handles the optional negative sign automatically
void send_speed(int16_t mm_sec)
{
    char buf[8]; // worst case "@-9999$" + null = 8 bytes
    snprintf(buf, 8, "@%d$", mm_sec);

    // walk the string and send each byte. loop ends at null terminator
    // which is NOT transmitted - it just marks end of our string.
    for (uint8_t i = 0; buf[i] != '\0'; i++)
        serial_putchar(buf[i]);
}

// display remote speed at lower-right of lcd in cm/sec (integer)
// only 4 columns available (cols 12-15) so we show signed integer cm/sec
// like " 12" or "-99". incoming value is in mm/sec, so /10 gives cm/sec.
void show_remote_speed(int16_t mm_sec)
{
    char buf[8];                 // sized for worst case int16 plus null
    int16_t cm = mm_sec / 10;    // integer cm/sec, sign preserved
    snprintf(buf, 5, "%4d", cm); // right-aligned in 4 chars
    lcd_moveto(1, COL_REMOTE);
    lcd_stringout(buf);
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
        if (echo_state == 1) // guard in case of PCINT firing for no reason (electrical noise ig)
        {
            TCCR1B = 0; // stop timer
            uint16_t count = TCNT1;

            // convert count to distance in mm
            // prescaler 8 -> 16MHz/8 = 2MHz -> 0.5us per tick
            // sound travels at 340 m/s = 340000 mm/s
            // distance_mm = count * 0.5us * 340000mm/s / 2 = count * 17 / 200
            uint16_t dist_mm = (uint32_t)count * 17 / 200;
            // gotta cast, count can reach 47000... this * 17 = 799000 which is over 16 bit uint16_1 (max 65535)
            // without cast multiplicatino overlflow so we evaluate in 32 bit artihmetic then put it back into uint16_t after the /200 since it fits

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
// STATE_TIMING = stopwatch tick (increment elapsed_tenths, check 10s timeout)
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

// timer0 isr - fires every half-period of the current tone
// while count > 0: toggle pb5 to make the square thing that creates the sound
// when count reaches 0: load the next note's values, or stop if all 3 notes are done
ISR(TIMER0_COMPA_vect)
{
    if (buzz_count > 0)
    {
        PORTB ^= (1 << PB5); // toggle buzzer pin (this is what makes the sound)
        buzz_count--;
    }
    else
    {
        // current note is done - move to next note in the sequence
        buzz_idx++;
        if (buzz_idx < 3)
        {
            // load the next note's pitch and duration from the arrays
            OCR0A = buzz_ocr[buzz_idx];
            buzz_count = buzz_cnt[buzz_idx];
            TCNT0 = 0;
            // timer is still running - no need to restart it
        }
        else
        {
            // all 3 notes finished - shut everything down
            TCCR0B = 0;           // stop timer0 (no more interrupts)
            PORTB &= ~(1 << PB5); // leave buzzer pin low (silent)
        }
    }
}

// usart receive-complete isr - fires every time one byte arrives on rx
// implements the @<optional minus><1-4 digits>$ protocol from the spec.
// on ANY error (missing @, garbage chars, overflow, missing $) we just reset
// rx_started and wait for the next '@'
ISR(USART_RX_vect)
{
    char c = UDR0; // reading udr0 also clears the rxc0 (rx complete) flag

    if (c == '@')
    {
        // start of a new packet.. even if the previus one was incomplete,
        // we abandon it here and start fresh. this is our recovery mechanism
        // for partially-transmitted or garbled packets
        rx_started = 1;
        rx_count = 0;
        rx_valid = 0;
    }

    else if (rx_started)
    {
        if (c == '$')
        {
            // end-of-packet marker. only valid if at least one digit arrived
            // (spec: "if there is no threshold data thwn the flag should not be set")
            if (rx_count > 0)
            {
                rx_buf[rx_count] = '\0'; // null-terminate so sscanf can parse
                rx_valid = 1;            // signal main loop: data ready
            }
            rx_started = 0; // either way, packet is done
        }
        else if (c == '-' || (c >= '0' && c <= '9'))
        {
            // valid character so store it if there's room
            // buffer is 6 bytes, max payload is 5 chars (-9999) + null,
            // so rx_count = 0..4 is ok. count == 5 mesns overflow!!!
            if (rx_count < 5)
            {
                rx_buf[rx_count] = c;
                rx_count++;
            }
            else
            {
                // overflow: toss thi bad boy
                rx_started = 0;
            }
        }
        else
        {
            // garbage character mid-packet so discard everything we have
            rx_started = 0;
        }
    }
    // if rx_started is 0 and c isn't '@', we just ignore the byte
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
    PORTB &= ~(1 << PB4);             // drive pb4 low to enable the rx tri-state buffer (during programming pb4 floats high via pull-up,
                                      // which disables the buffer)
    PORTB &= ~(1 << PB5);             // buzzer off
    PORTD &= ~(1 << PD3);             // trigger idle-low
    PORTC |= (1 << PC4) | (1 << PC5); // encoder pull-ups on
    led_off_all();                    // LEDs off during init

    // module initialisation
    lcd_init();
    adc_init();
    timer2_init();
    timer1_setup_pulse(); // TIMER1 ready for first echo measurement
    usart_init();
    timer0_init(); // configure timer0 for buzzer (stays stopped until a tone plays)

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

    // TEMPORARY DEBUG play startup tone to verify buzzer works
    play_ascending();
    _delay_ms(1500); // wait long enough for the 3-note sequence to finish

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

        // remote speed packet received? parse it and update lcd
        if (rx_valid)
        {
            rx_valid = 0; // clear flag FIRST so a packet arriving during sscanf doesn't get its rx_valid=1 fucked by us setting it to 0

            int16_t parsed;
            // %hd = 16-bit signed int (matches int16_t). cast away volatile because
            // we know the isr won't touch rx_buf again until the next '@' arrives.
            if (sscanf((char *)rx_buf, "%hd", &parsed) == 1)
            {
                remote_mm_sec = parsed;
                remote_valid = 1;
                show_remote_speed(remote_mm_sec);

                // pick which sequence to play based on |remote speed| vs threshold.
                // remote_mm_sec is in mm/sec, threshold is in cm/sec, so multiply threshold by 10.
                // only the magnitude matters - direction (positive vs negative) is ignored.
                uint16_t remote_mag = (remote_mm_sec < 0) ? (uint16_t)(-remote_mm_sec)
                                                          : (uint16_t)(remote_mm_sec);
                if (remote_mag > (uint16_t)threshold * 10)
                    play_ascending(); // remote is faster than threshold
                else
                    play_descending(); // remote is at or below threshold
            }
        }

        // stopwatch timeout: 10s elapsed with no second measurement
        if (stopwatch_timeout)
        {
            stopwatch_timeout = 0;
            state = STATE_IDLE;
            speed_valid = 0;
            update_led(); // back to blue
            OCR2A = 35;   // servo back to start
            lcd_moveto(0, COL_TIME);
            lcd_stringout("    "); // clear elapsed time
        }

        // update elapsed time display and servo dial while stopwatch is running
        if (state == STATE_TIMING && time_changed)
        {
            time_changed = 0;
            show_elapsed();
            // OCR2A = 35 - (elapsed_tenths * 23) / 100
            // fuck it overflows if we multiply by 23, fix (cast)
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
                // send speed to remote board
                send_speed(speed_mm_sec); // transmit our speed in mm/sec to the remote board
            }
            else
            {
                // second measurement failed
                lcd_moveto(0, COL_R2);
                lcd_stringout(" ---");
                speed_valid = 0;
                update_led();
            }

            OCR2A = 35; // servo back to start position
            state = STATE_IDLE;
            _delay_ms(200); // debounce
        }

        _delay_ms(20);
    }

    return 0; // never reached
}