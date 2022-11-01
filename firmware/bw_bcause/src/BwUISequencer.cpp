#include <BwUISequencer.h>

BwUISequencer::BwUISequencer(BwDriveTrain* drive, Adafruit_NeoPixel* led_ring, int num_pixels)
{
    this->drive = drive;
    this->led_ring = led_ring;
    _num_pixels = num_pixels;
    _sequences = new uint64_t*[MAX_NUM_SEQUENCES];
    for (uint8_t serial = 0; serial < MAX_NUM_SEQUENCES; serial++) {
        _sequences[serial] = NULL;
    }
    _selected_serial = MAX_NUM_SEQUENCES + 1;
    _is_delay_active = false;
    _from_flash = false;
    _prev_status = 0;
}

void BwUISequencer::allocate_sequence(uint8_t serial, uint16_t length)
{
    uint32_t arr_length = (uint32_t)(length) + 1;
    if (_sequences[serial] != NULL) {
        free(_sequences[serial]);
    }
    _sequences[serial] = new uint64_t[arr_length];
    _sequences[serial][0] = arr_length - 1;
}

bool BwUISequencer::set_element(uint8_t serial, uint16_t index, uint64_t parameter) {
    if (_sequences[serial] == NULL) {
        return false;
    }
    _sequences[serial][index + 1] = parameter;
    return true;
}

bool BwUISequencer::play_sequence(uint8_t serial, bool loop_sequence, bool from_flash)
{
    if (from_flash) {
        if (serial >= NUM_STORED_SEQUENCES) {
            return false;
        }
    }
    else {
        if (_sequences[serial] == NULL) {
            return false;
        }
    }
    _from_flash = from_flash;
    _selected_serial = serial;
    _selected_index = 0;
    _loop_sequence = loop_sequence;
    _sequence_start_time = millis();
    _cumulative_delay = 0;
    _is_delay_active = false;
    _prev_status = 0;
    return true;
}

void BwUISequencer::stop_sequence()
{
    _selected_serial = MAX_NUM_SEQUENCES + 1;
}

int BwUISequencer::update()
{
    if (_selected_serial == MAX_NUM_SEQUENCES + 1) {
        return 0;
    }
    uint64_t sequence_length;
    if (_from_flash) {
        sequence_length = get_stored_element(_selected_serial, 0);
    }
    else {
        sequence_length = _sequences[_selected_serial][0];
    }
    if (_selected_index >= sequence_length) {
        if (_loop_sequence) {
            _selected_index = 0;
        }
        else {
            _selected_serial = MAX_NUM_SEQUENCES + 1;
            Serial.println("Sequence completed");
            return 0;
        }
    }
    if (_selected_index == 0) {
        _sequence_start_time = millis();
        _cumulative_delay = 0;
    }
    uint32_t dt = millis() - _sequence_start_time;
    if (dt < _cumulative_delay) {
        return _prev_status;  // wait for delay to finish
    }
    else if (_is_delay_active) {
        _selected_index++;
        _is_delay_active = false;
    }

    uint64_t parameter;
    if (_from_flash) {
        parameter = get_stored_element(_selected_serial, _selected_index + 1);
    }
    else {
        parameter = _sequences[_selected_serial][_selected_index + 1];
    }
    BwSequenceType_t type = static_cast<BwSequenceType_t>(parameter & 0b1111);
    Serial.print("Sequence index ");
    Serial.print(_selected_index);
    Serial.print(" of ");
    Serial.print(sequence_length);
    Serial.print(", type=");
    Serial.println(type);

    switch (type)
    {
    case BW_SEQ_START_TONE:
        play_tone_from_param(parameter);
        _selected_index++;
        _prev_status = 2;  // indicates that motors are required for this element
        break;

    case BW_SEQ_STOP_TONE:
        stop_tone_from_param(parameter);
        _selected_index++;
        _prev_status = 2;  // indicates that motors are required for this element
        break;

    case BW_SEQ_DELAY:
        _cumulative_delay += get_delay(parameter);
        _is_delay_active = true;
        break;

    case BW_SEQ_SET_RING_LED:
        set_led_from_param(parameter);
        _selected_index++;
        _prev_status = 1;
        break;

    case BW_SEQ_SHOW_LED:
        Serial.println("Show leds");
        led_ring->show();
        _selected_index++;
        _prev_status = 1;
        break;

    default:
        _selected_index++;
        break;
    }
    return _prev_status;  // indicates that no motors are required for this element
}

void BwUISequencer::play_tone_from_param(uint64_t parameters)
{
    uint16_t frequency = (parameters >> 4) & 0xffff;
    uint8_t volume = (parameters >> 20) & 0xff;
    uint8_t channel = (parameters >> 28) & 0b1111;
    Serial.print("Playing tone. channel=");
    Serial.print(channel);
    Serial.print(", freq=");
    Serial.print(frequency);
    Serial.print(", volume=");
    Serial.println(volume);
    set_module_tone(channel, frequency, volume);
}

void BwUISequencer::stop_tone_from_param(uint64_t parameters)
{
    uint8_t channel = (parameters >> 28) & 0b1111;
    Serial.print("Stopping tone. channel=");
    Serial.println(channel);
    set_module_tone(channel, 0, 0);
}

void BwUISequencer::set_led_from_param(uint64_t parameters)
{
    uint32_t color = (parameters >> 4) & 0xffffffff;
    uint16_t index = (uint16_t)((parameters >> 36) & 0xffff);
    Serial.print("Setting led. index=");
    Serial.print(index);
    Serial.print(", color=");
    Serial.println(color);
    led_ring->setPixelColor(index, color);
}

uint32_t BwUISequencer::get_delay(uint64_t parameters) {
    uint32_t delay = (uint32_t)((parameters >> 4) & 0xffff);
    Serial.print("Set delay: ");
    Serial.println(delay);
    return delay;
}

void BwUISequencer::set_module_tone(unsigned int channel, int frequency, int volume)
{
    if (channel < drive->get_num_motors()) {
        if (frequency > 0) {
            drive->get_module(channel)->get_motor()->set_frequency(frequency);
        }
        drive->get_module(channel)->command_wheel_pwm(volume);
    }
}
