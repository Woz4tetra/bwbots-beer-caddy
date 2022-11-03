#include <StoredSequences.h>

uint64_t get_stored_element(uint8_t serial, uint64_t index)
{
    if (serial >= NUM_STORED_SEQUENCES) {
        Serial.println("Serial exceeds number of stored sequences");
        return 0;
    }
    uint64_t array_index = SEQUENCE_LOCATIONS[serial];
    uint64_t sequence_length = STORED_SEQUENCES[array_index];
    if (index >= sequence_length) {
        Serial.println("Sequence exceeds expected length");
        return 0;
    }
    else {
        return STORED_SEQUENCES[array_index + index];
    }
}