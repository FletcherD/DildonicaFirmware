#include <soc.h>
#include <nrfx.h>
#include <zephyr/drivers/gpio.h>
#include <nrfx_timer.h>
#include <nrfx_comp.h>
#include <nrfx_ppi.h>

#include <zephyr/kernel.h>

#include <math.h>
#include "circularbuffer.hpp"

#include <nrfx_uarte.h>
#include <nrfx_gpiote.h>

#include "dildonica.hpp"



static nrfx_timer_t TIMER1 = NRFX_TIMER_INSTANCE(1);
static nrfx_timer_t TIMER2 = NRFX_TIMER_INSTANCE(2);

struct DildonicaSampleRaw {
    uint8_t zone;
    uint32_t cyclePeriod;
};


const float ExponentialMeanMaxError = 10000;
const float ExponentialMeanRate = 0.001;
struct DildonicaZoneState {
    float cyclePeriodExponentialMean;
    float valueNormalized;
};

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

static const struct gpio_dt_spec PIN_LED0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec PIN_LED1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec PIN_LED2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct gpio_dt_spec PIN_LED3 = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);
// const int PIN_RGBLED_CLOCK = 6;
// const int PIN_RGBLED_DATA = 8;
//static const struct gpio_dt_spec PIN_DILDONICA_OSC = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, dildonica_osc_in_gpios);
static const struct gpio_dt_spec PIN_DILDONICA_ZONE_SELECT[] = {
    GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, dildonica_zone_select_gpios, 0),
    GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, dildonica_zone_select_gpios, 1),
    GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, dildonica_zone_select_gpios, 2),
    GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, dildonica_zone_select_gpios, 3),
    GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, dildonica_zone_select_gpios, 4),
    GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, dildonica_zone_select_gpios, 5),
    GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, dildonica_zone_select_gpios, 6),
    GPIO_DT_SPEC_GET_BY_IDX(ZEPHYR_USER_NODE, dildonica_zone_select_gpios, 7),
};


// Timer begins counting after this many oscillation cycles
// The delay is to ignore irregular cycles right after the coil is powered up
const uint32_t DILDONICA_MEASUREMENT_CYCLES_BEGIN = 32;
// Measurement stops after this many cycles
const uint32_t DILDONICA_MEASUREMENT_CYCLES_END = 10000 + DILDONICA_MEASUREMENT_CYCLES_BEGIN;
// If we don't have enough cycles after this much time, timeout - hardware error (coil broken, etc.)
const uint32_t DILDONICA_MEASUREMENT_TIMEOUT_US = 20000;


// Low and high voltage threshold values used for counting cycles
const uint32_t DILDONICA_OSC_COMP_THRESH_LO = 10;
const uint32_t DILDONICA_OSC_COMP_THRESH_HI = 20;

const uint8_t DILDONICA_N_ZONES = 8;
uint8_t dildonicaCurZone = 0;

CircularQueue<DildonicaSampleRaw, 256> dildonicaSampleQueue;

const float DILDONICA_SAMPLE_MAX_ERROR = 10000;
const float DILDONICA_SAMPLE_AVERAGE_COEFF = 0.001;

DildonicaZoneState dildonicaZoneStates[DILDONICA_N_ZONES];

const uint8_t DILDONICA_MIDI_CONTROL_START = 41;
// This is: Units of MIDI Control Change for a 100% change in cycle period
const float DILDONICA_MIDI_CONTROL_SLOPE = 10000.0;

uint32_t led_test = 0;


template <typename T, size_t N>
char ( &_ArraySizeHelper( T (&array)[N] ))[N];
#define mycountof( array ) (sizeof( _ArraySizeHelper( array ) ))


/* UARTE *///////////////////////////////

/** @brief Symbol specifying UARTE instance to be used. */
#define UARTE_INST_IDX 1

/** @brief Symbol specifying TX pin number of UARTE. */
#define UARTE_TX_PIN NRF_GPIO_PIN_MAP(0,3)

/** @brief Symbol specifying RX pin number of UARTE. */
#define UARTE_RX_PIN NRF_GPIO_PIN_MAP(0,28)

nrfx_uarte_t uarte_inst = NRFX_UARTE_INSTANCE(UARTE_INST_IDX);

static void uarte_handler(nrfx_uarte_event_t const * p_event, void * p_context)
{
}

void setup_uarte() {

#if defined(__ZEPHYR__)
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_UARTE_INST_GET(UARTE_INST_IDX)), IRQ_PRIO_LOWEST,
                NRFX_UARTE_INST_HANDLER_GET(UARTE_INST_IDX), 0, 0);
#endif

    nrfx_uarte_config_t uarte_config = NRFX_UARTE_DEFAULT_CONFIG(UARTE_TX_PIN, UARTE_RX_PIN);
    uarte_config.p_context = &uarte_inst;
    uarte_config.baudrate = NRF_UARTE_BAUDRATE_1000000;
    nrfx_uarte_init(&uarte_inst, &uarte_config, uarte_handler);
}

void printf_uart( const char* format, ... ) {
    static uint8_t tx_buffer[64];
    va_list arglist;
    va_start( arglist, format );
    size_t len = vsprintf( (char*)tx_buffer, format, arglist );
    va_end( arglist );

    nrfx_uarte_tx(&uarte_inst, tx_buffer, len, 0);
}


/* UARTE *///////////////////////////////


void setDildonicaZoneActiveMask(uint8_t zoneMask) {
    for (uint8_t i = 0; i != DILDONICA_N_ZONES; i++) {
        bool pinState = ((1 << i) &
        zoneMask);
        gpio_pin_set_dt(&PIN_DILDONICA_ZONE_SELECT[i], pinState);
    }
}


static void timer1_handler(nrf_timer_event_t event_type, void* p_context) {
    if (event_type == NRF_TIMER_EVENT_COMPARE2) {
        // Measurement timeout - probably coil problem.

        DildonicaSampleRaw thisSample = {
            dildonicaCurZone,
            DILDONICA_MEASUREMENT_TIMEOUT_US * 16
        };
        dildonicaSampleQueue.enqueue(thisSample);

        dildonicaCurZone = (dildonicaCurZone + 1) % DILDONICA_N_ZONES;
        setDildonicaZoneActiveMask(1 << dildonicaCurZone);

        nrfx_timer_clear(&TIMER1);
        nrfx_timer_clear(&TIMER2);
    }
}

static void timer2_handler(nrf_timer_event_t event_type, void* p_context) {
    if (event_type == NRF_TIMER_EVENT_COMPARE1) {
        // Normal coil measurement

        DildonicaSampleRaw thisSample = {
            dildonicaCurZone,
            nrfx_timer_capture_get(&TIMER1, NRF_TIMER_CC_CHANNEL1) - nrfx_timer_capture_get(&TIMER1, NRF_TIMER_CC_CHANNEL0)
        };
        dildonicaSampleQueue.enqueue(thisSample);

        dildonicaCurZone = (dildonicaCurZone + 1) % DILDONICA_N_ZONES;
        setDildonicaZoneActiveMask(1 << dildonicaCurZone);

        // size_t len = sprintf((char*)uart_tx_buf, "Hello dildonica");
        // uart_tx(uart, uart_tx_buf, len, SYS_FOREVER_US);

        nrfx_timer_clear(&TIMER1);
        nrfx_timer_clear(&TIMER2);
    }
}

static void comp_handler(nrf_comp_event_t event_type) {
}

void setup_comparator() {

#if defined(__ZEPHYR__)
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_COMP), 0,
                nrfx_comp_irq_handler, 0, 0);
#endif

    nrfx_comp_config_t comp_config = NRFX_COMP_DEFAULT_CONFIG(NRF_COMP_INPUT_2);
    comp_config.main_mode = NRF_COMP_MAIN_MODE_SE;
    comp_config.speed_mode = NRF_COMP_SP_MODE_HIGH;
    comp_config.reference = NRF_COMP_REF_VDD;
    comp_config.threshold.th_down = DILDONICA_OSC_COMP_THRESH_LO;
    comp_config.threshold.th_up = DILDONICA_OSC_COMP_THRESH_HI;
    nrfx_err_t err = nrfx_comp_init(&comp_config, comp_handler);

    nrfx_comp_start(0,0);
}

void setup_ppi() {
    nrf_ppi_channel_t ppi_channel;

    // Comparator UP -> Timer2 Count
    nrfx_ppi_channel_alloc(&ppi_channel);
    nrfx_ppi_channel_assign(ppi_channel, nrfx_comp_event_address_get(NRF_COMP_EVENT_UP), nrfx_timer_task_address_get(&TIMER2, NRF_TIMER_TASK_COUNT));
    nrfx_ppi_channel_enable(ppi_channel);

    // Timer2 Compare -> Timer1 Capture
    nrfx_ppi_channel_alloc(&ppi_channel);
    nrfx_ppi_channel_assign(ppi_channel, nrfx_timer_event_address_get(&TIMER2, NRF_TIMER_EVENT_COMPARE0), nrfx_timer_task_address_get(&TIMER1, NRF_TIMER_TASK_CAPTURE0));
    nrfx_ppi_channel_enable(ppi_channel);

    nrfx_ppi_channel_alloc(&ppi_channel);
    nrfx_ppi_channel_assign(ppi_channel, nrfx_timer_event_address_get(&TIMER2, NRF_TIMER_EVENT_COMPARE1), nrfx_timer_task_address_get(&TIMER1, NRF_TIMER_TASK_CAPTURE1));
    nrfx_ppi_channel_enable(ppi_channel);
}

void setup_gpio() {
    gpio_pin_configure_dt(&PIN_LED0, GPIO_OUTPUT);
    gpio_pin_set_dt(&PIN_LED0, 0);
    gpio_pin_configure_dt(&PIN_LED1, GPIO_OUTPUT);
    gpio_pin_set_dt(&PIN_LED1, 0);
    gpio_pin_configure_dt(&PIN_LED2, GPIO_OUTPUT);
    gpio_pin_set_dt(&PIN_LED2, 0);
    //gpio_pin_configure_dt(&PIN_DILDONICA_OSC, GPIO_INPUT);

    for (int i = 0; i < mycountof(PIN_DILDONICA_ZONE_SELECT); i++) {
        gpio_pin_configure_dt(&PIN_DILDONICA_ZONE_SELECT[i], GPIO_OUTPUT);
    }

    setDildonicaZoneActiveMask(1);
}

void setup_timers() {

#if defined(__ZEPHYR__)
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER_INST_GET(1)), 0,
                NRFX_TIMER_INST_HANDLER_GET(1), 0, 0);
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER_INST_GET(2)), 0,
                NRFX_TIMER_INST_HANDLER_GET(2), 0, 0);
#endif

    uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(TIMER1.p_reg);
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    timer_cfg.p_context = (void*)"Some context";

    nrfx_timer_init(&TIMER1, &timer_cfg, timer1_handler);
    // Timeout interrupt
    nrfx_timer_extended_compare(&TIMER1,
                                NRF_TIMER_CC_CHANNEL2,
                                DILDONICA_MEASUREMENT_TIMEOUT_US * 16,
                                (nrf_timer_short_mask_t)0,
                                true);

    timer_cfg.mode = NRF_TIMER_MODE_COUNTER;
    nrfx_timer_init(&TIMER2, &timer_cfg, timer2_handler);
    nrfx_timer_extended_compare(&TIMER2,
                                NRF_TIMER_CC_CHANNEL0,
                                DILDONICA_MEASUREMENT_CYCLES_BEGIN,
                                (nrf_timer_short_mask_t)0,
                                false);
    nrfx_timer_extended_compare(&TIMER2,
                                NRF_TIMER_CC_CHANNEL1,
                                DILDONICA_MEASUREMENT_CYCLES_END,
                                (nrf_timer_short_mask_t)0,
                                true);

    nrfx_timer_enable(&TIMER1);
    nrfx_timer_enable(&TIMER2);
}


int main(void) {
    setup_uarte();

    setup_gpio();
    setup_timers();

    setup_comparator();
    setup_ppi();

    printf_uart("Hello Dildonica\r\n");

    // //setupMidi();  // Assuming this function is refactored to use nrfx as well

    while (1) {
        if (!dildonicaSampleQueue.is_empty()) {
            DildonicaSampleRaw dSample = dildonicaSampleQueue.dequeue();

            gpio_pin_set_dt(&PIN_LED0, dSample.cyclePeriod & 1);

            uint8_t dSampleZone = dSample.zone;

            printf_uart("%d %d\r\n", dSampleZone, dSample.cyclePeriod);
            // DildonicaZoneState& zoneState = dildonicaZoneStates[dSampleZone];


            // updateDildonicaZone(zoneState, dSample);

            // static char message[64];
            // size_t message_len = sprintf(message, "%d, %d, %0.9f\r\n", dSampleZone, dSample.cyclePeriod, zoneState.valueNormalized);
            // serialWrite((uint8_t *) message, message_len);

            // int32_t midiControlValue = lround(zoneState.valueNormalized * DILDONICA_MIDI_CONTROL_SLOPE) + 63;
            // midiControlValue = max(0, midiControlValue);
            // midiControlValue = min(127, midiControlValue);
            // midiSendControlChange(DILDONICA_MIDI_CONTROL_START + dSampleZone, midiControlValue);

            // if(dSampleZone == DILDONICA_N_ZONES-1) {
            //     uint32_t colorR = abs(10000. * (dildonicaZoneStates[0].valueNormalized + dildonicaZoneStates[1].valueNormalized + dildonicaZoneStates[2].valueNormalized));
            //     colorR = min(255, colorR);
            //     uint32_t colorG = abs(10000. * (dildonicaZoneStates[3].valueNormalized + dildonicaZoneStates[4].valueNormalized));
            //     colorG = min(255, colorG);
            //     uint32_t colorB = abs(10000. * (dildonicaZoneStates[5].valueNormalized + dildonicaZoneStates[6].valueNormalized + dildonicaZoneStates[7].valueNormalized));
            //     colorB = min(255, colorB);
            //     rgbLed.setPixelColor(0, colorR, colorG, colorB);
            //     rgbLed.show();
            // }
        }


    }
}

/** @} */
