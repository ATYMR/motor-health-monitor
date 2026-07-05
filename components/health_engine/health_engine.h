#ifndef HEALTH_ENGINE_H
#define HEALTH_ENGINE_H

#include <stddef.h>
#include "esp_err.h"

typedef struct
{
    float rms;
    float peak;
    float peak_to_peak;
    float crest_factor;
} vibration_metrics_t;

esp_err_t health_engine_analyze(
    const float *samples,
    size_t sample_count,
    vibration_metrics_t *metrics
);

#endif