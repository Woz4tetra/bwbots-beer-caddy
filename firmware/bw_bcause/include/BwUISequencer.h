#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <BwDriveTrain.h>
#include <StoredSequences.h>

typedef enum BwSequenceType {
    BW_SEQ_START_TONE = 0,
    BW_SEQ_STOP_TONE = 1,
    BW_SEQ_DELAY = 2,
    BW_SEQ_SET_RING_LED = 3,
    BW_SEQ_SHOW_LED = 4
} BwSequenceType_t;

class BwUISequencer
{
private:
    uint64_t** _sequences;
    BwDriveTrain* drive;
    Adafruit_NeoPixel* led_ring;

    uint8_t _selected_serial;
    uint16_t _selected_index;
    int _num_pixels;
    uint32_t _sequence_start_time;
    bool _loop_sequence;
    uint32_t _cumulative_delay;
    bool _is_delay_active;
    bool _from_flash;
    int _prev_status;

    void play_tone_from_param(uint64_t parameters);
    void stop_tone_from_param(uint64_t parameters);
    void set_led_from_param(uint64_t parameters);
    uint32_t get_delay(uint64_t parameters);

    void set_module_tone(unsigned int channel, int frequency, int volume);

public:
    const static uint8_t MAX_NUM_SEQUENCES = 64;  // MAX_NUM_SEQUENCES + 1 reserved for no sequence
    BwUISequencer(BwDriveTrain* drive, Adafruit_NeoPixel* led_ring, int num_pixels);
    void allocate_sequence(uint8_t serial, uint16_t length);
    bool set_element(uint8_t serial, uint16_t index, uint64_t parameter);
    bool play_sequence(uint8_t serial, bool loop_sequence, bool from_flash);
    void stop_sequence();
    uint8_t get_status() { return (uint8_t)_prev_status; }
    uint16_t get_index() { return _selected_index; }
    uint8_t get_current_sequence() { return _selected_serial; }
    bool is_current_from_flash() { return _from_flash; }
    int update();
};
