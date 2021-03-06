/*
 * @file    main.c
 * @brief   Photocell 2. Read the light sensor signal through ADC,
 *          flash LEDs accordingly to digitized analog input signal.
 *          This version differs from version 1 in that this version
 *          deosn't use fixed thresholds for classification of ADC output,
 *          but implements finite state machine to drive calibration process.
 *          Calibration allows for chosing of optimal thresholds in current
 *          environment.
 * @details Compile with 'make flash', i.e. JTAG enabled.
 *          Each ADC conversion is started manually (by writing 1 to ADSC). 
 *          One needs to calibrate accordingly to sensor's spectral response.
 *          This is written for SEN 09088 mini photocell (GL5528):
 *          - light resistance at 10Lux(at 25*C): 8-20 KOhm
 *          - dark resistance at 0 Lux: 1.0 MOhm (min)
 *          - gamma value at 100-10 Lux: 0.7
 *          - power dissipation (at 25*C): 100 mW
 *          - max voltage (at 25*C): 150 V
 *          - spectral response peak (at 25*C): 540 nm
 *          - ambient temperature range: -30 ~ +70 *C
 *
 *          Signaling:
 *          PORTD:
 *          PD0 - flashing with 1Hz freq: malloc failure
 *          PD1 - flashing with 1Hz freq: calibration...
 *          PD3 - toggled: new sample appended to sma buffer
 *
 * @date    01 Sep 2016 07:39 PM
 */


#define __DELAY_BACKWARD_COMPATIBLE__

#ifndef F_CPU
#define F_CPU 1000000UL    /* 1 MHz clock speed */
#endif

#include <avr/io.h>
#include <util/delay.h>

#include <stdlib.h>


#include "fsm.h"
#define BUFF_TYPE uint8_t
#include "sma_buf.h"


/* Voltage divider:
 *
 ********************************
 * Vin --
 *     R1 (6.67 kOhm used)
 *     -----Vout (to ADC port 0)
 *     R2 (photocell)
 *     gnd
 *******************************
 *
 * Vout = Vin * R2 / (R1 + R2)
 */


#define SMA_SAMPLES_PER_SEC 5u
#define SMA_SAMPLES_PER_SECONDS(seconds) ((seconds) * SMA_SAMPLES_PER_SEC)
#define SMA_SAMPLES_PER_MILISECONDS(ms) ((ms) * SMA_SAMPLES_PER_SEC / 1000)
#define CLK_FREQ 15625u
#define CLK_TICKS_PER_SMA_SAMPLE ((CLK_FREQ) / (SMA_SAMPLES_PER_SEC))
#define CLK_TICKS_PER_SMA_SAMPLES(x) ((x) * (CLK_TICKS_PER_SMA_SAMPLE))
#define MIN(x,y) ((x) < (y) ? (x) : (y))

uint8_t V_REF = 5;  /* 5V reference */

/* Derive ADC FSM from fsm */
struct adc {
    struct fsm      super_;
    struct sma_buf  sma;
    uint8_t         sma_prev;   /* we use uint8_t to store ADC readings */
    uint8_t         resolution;
};

/* Derive ADC events from fsm_event */
struct adc_event {
    struct fsm_event    super_;
    uint16_t            sample;
};

/* Signals used by ADC FSM */
enum {
    ADC_NEW_SAMPLE_SIG,
    ADC_CALIBRATION_SIG
};

/* Override fsm methods. */
static void adc_ctor(struct adc* me);
static void adc_initial(struct adc* me, struct adc_event* e);
static void adc_default(struct adc* me, struct adc_event* e);
static void adc_calibration(struct adc* me, struct adc_event* e);
static void adc_flash(struct adc* me, struct adc_event* e);

/* @brief   Initialize ADC */
static void adc_init(void) {
    ADMUX = 1 << REFS0; /* set up voltage reference to AVCC */
    ADCSRA = (1 << ADPS2);   /* set prescaler mode to 16: 1MHz/16 = 62.5kHz so it fits into 50kHz - 200kHz suggested in datasheet */
    ADCSRA |= (1 << ADEN); /* enable ADC */
    TCCR1B |= ((1 << CS11) | (1 << CS10));   /* use 16 bit timer, set timer prescaler to 64 mode, 1 tick = 1 / (F_CPU/64) s => 1 tick = (1/15625)s = 0.000064s, 15625 ticks/s */
    TCNT1 = 0; /* reset/start */
}

/* @brief   Return ADC sample. */
static uint16_t adc_read(uint8_t channel) {
    channel &= 0x07;    /* assert channel is between 0-7 inclusive */
    ADMUX = (ADMUX & 0xF8) | channel;   /* set multiplexer, clear bottom 3 bits before ORing. In Single Conversion mode, always select the channel before starting the conversion. */
    ADCSRA |= (1 << ADSC);  /* START single conversion, This bit stays high as long as the conversion is in progress and will be cleared by hardware when the conversion is completed. */
    while (!(ADCSRA & (1 << ADIF)));    /* allow conversion to complete, could use ADSC flag as well (but ADIF is officially supported version): while (ADCSRA & (1 << ADSC)); (because When a conversion is complete, the result is written to the ADC Data Registers, and ADIF is set. In single conversion mode, ADSC is cleared simultaneously. The software may then set ADSC again, and a new conversion will be initiated on the first rising ADC clock edge.) */
    return (ADC);
}

/* @brief   Initialize port pins to be output direction. */
static void ports_init(void) {
    DDRC = 0xFF;    /* set all PORTC to OUTPUT */
    PORTC = 0x00;   /* set LOW all output port pins */
    DDRD = 0xFF;    /* set as output - this will be our control interface */
    PORTD = 0x00;
}

/* @brief   ADC reading from voltage.
 * @details Voltage in mV for proper arithmetic without truncation.
static uint16_t adc_from_v(uint16_t v) {
    uint32_t tmp = v << 10;  V * 1024 
    return ((tmp / (V_REF * 1000)) & 0xFFFF);
} */

/* @brief   Voltage reading from ADC sample.
 * @details Result in mV. 
static uint16_t v_from_adc(uint16_t adc) {
    uint32_t tmp = adc * V_REF *1000;
    return ((tmp >> 10) & 0xFFFF);
} */

int
main(void) {
    struct adc          adc;
    struct adc_event    adc_e;

    ports_init();
    adc_init();
    adc_ctor(&adc);

    while(1) {
        adc_e.sample = adc_read(0);
        adc_e.super_.signal = ADC_NEW_SAMPLE_SIG;
        FSM_DISPATCH_((struct fsm*)&adc, (struct fsm_event*)&adc_e);
        _delay_loop_2(10);               /* using 16 bit counter, 10 loops, each of them takes 4 CPU cycles, so if F_CPU is 1MHz this does busy waiting for 40 microseconds */
    }
}

/* @brief   Flash PORTD pin 0 to signal failure. */
static void adc_signal_malloc_failure(void) {
    uint8_t n;
    while (1) {
        PORTD ^= (1 << PD0);
        n = 0;
        while (n++ < 4) { /* 1.048576s delay */
            _delay_loop_2(0);   /* 65536 loops * 4 cycles * 1microsecond = 0.262144s */
        }
    }
}

static void adc_ctor(struct adc* me) {
    if (me == NULL) {
        return;
    }
    memset(me, 0, sizeof(struct adc));
    INIT_SMA_BUF(&me->sma, SMA_SAMPLES_PER_SECONDS(10));
    if (me->sma.data == NULL) {
        adc_signal_malloc_failure();
        return;
    }
    FSM_CTOR_(&me->super_, &adc_initial);
}

static void adc_initial(struct adc* me, struct adc_event* e) {
    (void) e;
    FSM_TRANSITION_(&me->super_, &adc_default);
}

static void adc_default(struct adc* me, struct adc_event* e) {
    uint8_t sma;
    uint8_t resolution;

    if (me == NULL || e == NULL) {
        return;
    }
    switch (e->super_.signal) {
        case ADC_NEW_SAMPLE_SIG:
            if (TCNT1 >= CLK_TICKS_PER_SMA_SAMPLES(SMA_SAMPLES_PER_SECONDS(0.5))) {   /* 0.5s passed */
                APPEND_SMA_VAL(&me->sma, MIN(255, e->sample / 4)); /* buffer stores uint8_t */
                TCNT1 = 0;              /* reset timer counter */
                PORTD ^= (1 << PD3);    /* toggle PD3 */
            }
            adc_flash(me, e);
            sma = me->sma.sma;
            resolution = me->resolution;
            if (me->sma.lpos >= SMA_SAMPLES_PER_SECONDS(10) && ((sma > me->sma_prev + MIN(resolution, 255 - me->sma_prev)) || (sma < me->sma_prev - MIN(resolution, me->sma_prev)))) {
                FSM_TRANSITION_(&me->super_, &adc_calibration);         /* start calibration, do the state transition */
                e->super_.signal = ADC_CALIBRATION_SIG;
                FSM_DISPATCH_(&me->super_, &e->super_);
            }
            break;
        default:
            break;
    }
}

static void adc_calibration(struct adc* me, struct adc_event* e) {
    if (me == NULL || e == NULL) {
        return;
    }
    switch (e->super_.signal) {
        case ADC_CALIBRATION_SIG:
            PORTD |= (1 << PD0);                                    /* start calibration */
            PORTC = 0x00;
            RESET_SMA_BUF(&me->sma);                                /* reset SMA buffer and append new value */
        case ADC_NEW_SAMPLE_SIG:
            if (TCNT1 >= CLK_TICKS_PER_SMA_SAMPLES(SMA_SAMPLES_PER_SECONDS(0.5))) {     /* 0.5s passed */
                APPEND_SMA_VAL(&me->sma, MIN(255, e->sample / 4));                      /* buffer stores uint8_t */
                PORTD ^= (1 << PD1);                                                    /* flash PD1 */
                TCNT1 = 0;                                                              /* reset timer counter */
                if (me->sma.lpos >= SMA_SAMPLES_PER_SECONDS(10)) {
                    me->sma_prev = me->sma.sma;
                    me->resolution = MIN(me->sma.sma, 255 - me->sma.sma) / 4;   /* classification segment length */
                    PORTD &= ~(1 << PD0);                                       /* turn PD0 off */
                    FSM_TRANSITION_(&me->super_, &adc_default);                 /* stop calibration */
                }
                break;
            }
        default:
            break;
    }
}

static void adc_flash(struct adc* me, struct adc_event* e) {
    uint8_t adc_sample;
    uint8_t resolution;
    uint8_t sma;
    if (me == NULL || e == NULL) {
        return;
    }
    adc_sample = e->sample;
    resolution = me->resolution;
    sma = me->sma.sma;
    if (adc_sample > sma + 3 * resolution) {
        PORTC = 0b11111111;         /* set all HIGH */
    } else if (adc_sample > sma + 2 * resolution) {
        PORTC = 0b01111111;
    } else if (adc_sample > 1 * resolution) {
        PORTC = 0b00111111;
    } else if (adc_sample >= sma) {
        PORTC = 0b00011111;
    } else if (adc_sample > sma - 1 * resolution) {
        PORTC = 0b00001111;
    } else if (adc_sample > sma - 2 * resolution) {
        PORTC = 0b00000111;
    } else if (adc_sample > sma - 3 * resolution) {
        PORTC = 0b00000011;
    } else if (adc_sample > sma - 4 * resolution) {
        PORTC = 0b00000001;
    } else {
        PORTC = 0x00;               /* set all LOW */
    }
}
