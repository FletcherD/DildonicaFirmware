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

#define D_TIMER_INST_IDX 3
#define D_COUNTER_INST_IDX 4
static nrfx_timer_t TIMER_D_TIMER = NRFX_TIMER_INSTANCE(D_TIMER_INST_IDX);
static nrfx_timer_t TIMER_D_COUNTER = NRFX_TIMER_INSTANCE(D_COUNTER_INST_IDX);

#define TICKS_PER_MILLISECOND (NRF_TIMER_BASE_FREQUENCY_GET(TIMER_D_TIMER.p_reg) / 1000) 
#define TIMESTAMP_MS_MAX (1<<13)
#define TIMESTAMP_TICKS_MAX (TIMESTAMP_MS_MAX * TICKS_PER_MILLISECOND)
static uint32_t curTime = 0;

struct DildonicaSampleRaw {
    uint8_t zone;
    uint32_t timestamp;
    uint32_t cyclePeriod;
};

const float ExponentialMeanMaxError = 10000;
const float ExponentialMeanRate = 0.001;
struct DildonicaZoneState {
    float cyclePeriodExponentialMean;
    float valueNormalized;
    uint8_t midiControlValue;
};

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#define LED_ON 1
#define LED_OFF 0
static const struct gpio_dt_spec PIN_LED0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
//static const struct gpio_dt_spec PIN_LED1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
//static const struct gpio_dt_spec PIN_LED2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
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
const uint32_t DILDONICA_MEASUREMENT_CYCLES_BEGIN = 100;
// Measurement stops after this many cycles
const uint32_t DILDONICA_MEASUREMENT_CYCLES_END = 10000 + DILDONICA_MEASUREMENT_CYCLES_BEGIN;
// If we don't have enough cycles after this much time, timeout - hardware error (coil broken, etc.)
const uint32_t DILDONICA_MEASUREMENT_TIMEOUT_US = 50000;


// Low and high voltage threshold values used for counting cycles
const uint32_t DILDONICA_OSC_COMP_THRESH_LO = 10;
const uint32_t DILDONICA_OSC_COMP_THRESH_HI = 20;

const uint8_t DILDONICA_N_ZONES = 8;
uint8_t dildonicaCurZone = 0;

static CircularQueue<DildonicaSampleRaw, 256> dildonicaSampleQueue;

const float DILDONICA_SAMPLE_MAX_ERROR = 10000;
const float DILDONICA_SAMPLE_AVERAGE_COEFF = 0.001;

DildonicaZoneState dildonicaZoneStates[DILDONICA_N_ZONES];

const uint8_t DILDONICA_MIDI_CONTROL_START = 41;
// This is: Units of MIDI Control Change for a 100% change in cycle period
const float DILDONICA_MIDI_CONTROL_SLOPE = 10000.0;

extern "C" void send_midi_control_change(uint16_t timestamp, uint8_t channel, uint8_t controller, uint8_t value);
extern "C" void setup_bluetooth_peripheral();

uint32_t led_test = 0;


template <typename T, size_t N>
char ( &_ArraySizeHelper( T (&array)[N] ))[N];
#define mycountof( array ) (sizeof( _ArraySizeHelper( array ) ))


void setDildonicaZoneActiveMask(uint8_t zoneMask) {
    for (uint8_t i = 0; i != 8; i++) {
        bool pinState = ((1 << i) & zoneMask);
        gpio_pin_set_dt(&PIN_DILDONICA_ZONE_SELECT[i], pinState);
    }
}

void updateDildonicaZone(DildonicaZoneState& zoneState, DildonicaSampleRaw& sample) {
    float cyclePeriod = (float)sample.cyclePeriod;
    zoneState.valueNormalized = cyclePeriod - zoneState.cyclePeriodExponentialMean;
    if( fabs(zoneState.valueNormalized) > ExponentialMeanMaxError ) {
        zoneState.cyclePeriodExponentialMean = cyclePeriod;
        zoneState.valueNormalized = 0;
    } else {
        zoneState.cyclePeriodExponentialMean += zoneState.valueNormalized * ExponentialMeanRate;
    }
    zoneState.valueNormalized /= zoneState.cyclePeriodExponentialMean;
}

static void d_timer_handler(nrf_timer_event_t event_type, void* p_context) {
    if (event_type == NRF_TIMER_EVENT_COMPARE2) {
        // Measurement timeout - probably coil problem.

        uint32_t sampleTime = DILDONICA_MEASUREMENT_TIMEOUT_US * 16;
        curTime += sampleTime;
        curTime = (curTime % TIMESTAMP_TICKS_MAX);

        DildonicaSampleRaw thisSample = {
            dildonicaCurZone,
            curTime,
            sampleTime
        };
        dildonicaSampleQueue.enqueue(thisSample);

        dildonicaCurZone = (dildonicaCurZone + 1) % DILDONICA_N_ZONES;
        setDildonicaZoneActiveMask(1 << dildonicaCurZone);

        nrfx_timer_clear(&TIMER_D_TIMER);
        nrfx_timer_clear(&TIMER_D_COUNTER);
    }
}

static void d_counter_handler(nrf_timer_event_t event_type, void* p_context) {
    if (event_type == NRF_TIMER_EVENT_COMPARE1) {
        // Normal coil measurement
        
        curTime += nrfx_timer_capture_get(&TIMER_D_TIMER, NRF_TIMER_CC_CHANNEL1);
        curTime = (curTime % TIMESTAMP_TICKS_MAX);

        DildonicaSampleRaw thisSample = {
            dildonicaCurZone,
            curTime,
            nrfx_timer_capture_get(&TIMER_D_TIMER, NRF_TIMER_CC_CHANNEL1) - nrfx_timer_capture_get(&TIMER_D_TIMER, NRF_TIMER_CC_CHANNEL0)
        };
        dildonicaSampleQueue.enqueue(thisSample);

        dildonicaCurZone = (dildonicaCurZone + 1) % DILDONICA_N_ZONES;
        setDildonicaZoneActiveMask(1 << dildonicaCurZone);

        nrfx_timer_clear(&TIMER_D_TIMER);
        nrfx_timer_clear(&TIMER_D_COUNTER);
    }
}

static void comp_handler(nrf_comp_event_t event_type) {
}

void setup_comparator() {

#if defined(__ZEPHYR__)
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_COMP), IRQ_PRIO_LOWEST,
                nrfx_comp_irq_handler, 0, 0);
#endif

    nrfx_comp_config_t comp_config = NRFX_COMP_DEFAULT_CONFIG(NRF_COMP_INPUT_4);
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

    // Comparator UP -> TIMER_D_COUNTER Count
    nrfx_ppi_channel_alloc(&ppi_channel);
    nrfx_ppi_channel_assign(ppi_channel, nrfx_comp_event_address_get(NRF_COMP_EVENT_UP), nrfx_timer_task_address_get(&TIMER_D_COUNTER, NRF_TIMER_TASK_COUNT));
    nrfx_ppi_channel_enable(ppi_channel);

    // TIMER_D_COUNTER Compare -> Timer1 Capture
    nrfx_ppi_channel_alloc(&ppi_channel);
    nrfx_ppi_channel_assign(ppi_channel, nrfx_timer_event_address_get(&TIMER_D_COUNTER, NRF_TIMER_EVENT_COMPARE0), nrfx_timer_task_address_get(&TIMER_D_TIMER, NRF_TIMER_TASK_CAPTURE0));
    nrfx_ppi_channel_enable(ppi_channel);

    nrfx_ppi_channel_alloc(&ppi_channel);
    nrfx_ppi_channel_assign(ppi_channel, nrfx_timer_event_address_get(&TIMER_D_COUNTER, NRF_TIMER_EVENT_COMPARE1), nrfx_timer_task_address_get(&TIMER_D_TIMER, NRF_TIMER_TASK_CAPTURE1));
    nrfx_ppi_channel_enable(ppi_channel);
}

void setup_gpio() {
    gpio_pin_configure_dt(&PIN_LED0, GPIO_OUTPUT);
    //gpio_pin_configure_dt(&PIN_LED1, GPIO_OUTPUT);
    //gpio_pin_configure_dt(&PIN_LED2, GPIO_OUTPUT);
    gpio_pin_set_dt(&PIN_LED0, LED_OFF);
    //gpio_pin_set_dt(&PIN_LED1, LED_OFF);
    //gpio_pin_set_dt(&PIN_LED2, LED_OFF);
    //gpio_pin_configure_dt(&PIN_DILDONICA_OSC, GPIO_INPUT);

    for (int i = 0; i < 8; i++) {
        gpio_pin_configure_dt(&PIN_DILDONICA_ZONE_SELECT[i], GPIO_OUTPUT);
        gpio_pin_set_dt(&PIN_DILDONICA_ZONE_SELECT[i], 0);
    }

    setDildonicaZoneActiveMask(1);
}

void setup_timers() {

#if defined(__ZEPHYR__)
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER_INST_GET(D_TIMER_INST_IDX)), IRQ_PRIO_LOWEST,
                NRFX_TIMER_INST_HANDLER_GET(D_TIMER_INST_IDX), 0, 0);
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER_INST_GET(D_COUNTER_INST_IDX)), IRQ_PRIO_LOWEST,
                NRFX_TIMER_INST_HANDLER_GET(D_COUNTER_INST_IDX), 0, 0);
#endif

    uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(TIMER_D_TIMER.p_reg);
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    timer_cfg.p_context = (void*)"Some context";

    nrfx_timer_init(&TIMER_D_TIMER, &timer_cfg, d_timer_handler);
    // Timeout interrupt
    nrfx_timer_extended_compare(&TIMER_D_TIMER,
                                NRF_TIMER_CC_CHANNEL2,
                                DILDONICA_MEASUREMENT_TIMEOUT_US * 16,
                                (nrf_timer_short_mask_t)0,
                                true);

    base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(TIMER_D_COUNTER.p_reg);
    nrfx_timer_config_t counter_cfg = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
    counter_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    counter_cfg.p_context = (void*)"Some context";
    counter_cfg.mode = NRF_TIMER_MODE_COUNTER;
    nrfx_timer_init(&TIMER_D_COUNTER, &counter_cfg, d_counter_handler);
    nrfx_timer_extended_compare(&TIMER_D_COUNTER,
                                NRF_TIMER_CC_CHANNEL0,
                                DILDONICA_MEASUREMENT_CYCLES_BEGIN,
                                (nrf_timer_short_mask_t)0,
                                false);
    nrfx_timer_extended_compare(&TIMER_D_COUNTER,
                                NRF_TIMER_CC_CHANNEL1,
                                DILDONICA_MEASUREMENT_CYCLES_END,
                                (nrf_timer_short_mask_t)0,
                                true);

    nrfx_timer_enable(&TIMER_D_TIMER);
    nrfx_timer_enable(&TIMER_D_COUNTER);
}

extern "C" uint32_t peripheral_gatt_write(uint32_t);

int main(void) {
    // setup_uarte();

    // setup_gpio();
    // setup_timers();

    // setup_comparator();
    // setup_ppi();

    peripheral_gatt_write(1);
}

/* size of stack area used by each thread */
#define STACKSIZE 1024
/* scheduling priority used by each thread */
#define PRIORITY 7

extern "C" void send_midi_thread(void);

K_THREAD_DEFINE(send_midi_thread_id, STACKSIZE, send_midi_thread, NULL, NULL, NULL,
		PRIORITY, 0, 0);