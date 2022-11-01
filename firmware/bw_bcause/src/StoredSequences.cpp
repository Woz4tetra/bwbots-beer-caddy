#include <StoredSequences.h>

uint64_t get_stored_element(uint8_t serial, uint64_t index)
{
    if (serial >= NUM_STORED_SEQUENCES) {
        return 0;
    }
    uint64_t array_index = SEQUENCE_LOCATIONS[serial];
    uint64_t sequence_length = STORED_SEQUENCES[array_index];
    if (array_index + index >= sequence_length) {
        return 0;
    }
    else {
        return STORED_SEQUENCES[array_index + index];
    }
}
