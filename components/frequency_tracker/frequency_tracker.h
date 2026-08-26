#ifndef FREQUENCY_TRACKER_H
#define FREQUENCY_TRACKER_H

#include <stdbool.h>
#include "esp_err.h"


typedef enum
{
    FREQUENCY_STATE_SEARCHING = 0,
    FREQUENCY_STATE_CANDIDATE,
    FREQUENCY_STATE_STABLE

} frequency_tracker_state_t;


typedef struct
{
    frequency_tracker_state_t state;

    float candidate_frequency_hz;
    float stable_frequency_hz;

    int confirmation_count;
    int missed_window_count;

} frequency_tracker_t;


esp_err_t frequency_tracker_init(
    frequency_tracker_t *tracker
);


esp_err_t frequency_tracker_update(
    frequency_tracker_t *tracker,
    bool consensus_valid,
    float consensus_frequency_hz
);


const char *frequency_tracker_state_string(
    frequency_tracker_state_t state
);


#endif