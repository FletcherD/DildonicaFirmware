#ifndef DILDONICA_HPP
#define DILDONICA_HPP

#include <cstdint>

struct DildonicaSampleRaw {
    uint8_t zone;
    uint32_t timestamp;
    uint32_t cyclePeriod;
};

struct DildonicaZoneState {
    float cyclePeriodExponentialMean;
    float valueNormalized;
    uint8_t midiControlValue;
};

void setDildonicaZoneActiveMask(uint8_t zoneMask);
void updateDildonicaZone(DildonicaZoneState& zoneState, DildonicaSampleRaw& sample);
void setup_comparator();
void setup_ppi();
void setup_gpio();
void setup_timers();
void dildonica_thread(void);

// Constants and configuration values
const uint32_t DILDONICA_MEASUREMENT_CYCLES_BEGIN = 100;
const uint32_t DILDONICA_MEASUREMENT_CYCLES_END = 10100;
const uint32_t DILDONICA_MEASUREMENT_TIMEOUT_US = 50000;
const uint32_t DILDONICA_OSC_COMP_THRESH_LO = 10;
const uint32_t DILDONICA_OSC_COMP_THRESH_HI = 20;
const uint8_t DILDONICA_N_ZONES = 8;
const uint8_t DILDONICA_MIDI_CONTROL_START = 41;
const float DILDONICA_MIDI_CONTROL_SLOPE = 10000.0f;

#endif