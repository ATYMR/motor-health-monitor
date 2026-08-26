#include "frequency_tracker.h"

#include <math.h>
#include <stddef.h>


#define TRACKER_FREQUENCY_TOLERANCE_HZ    1.5f

#define TRACKER_CONFIRMATIONS_REQUIRED    3

#define TRACKER_MAX_MISSED_WINDOWS        2


esp_err_t frequency_tracker_init(
    frequency_tracker_t *tracker
)
{
    if (tracker == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }


    tracker->state =
        FREQUENCY_STATE_SEARCHING;


    tracker->candidate_frequency_hz =
        0.0f;


    tracker->stable_frequency_hz =
        0.0f;


    tracker->confirmation_count =
        0;


    tracker->missed_window_count =
        0;


    return ESP_OK;
}


esp_err_t frequency_tracker_update(
    frequency_tracker_t *tracker,
    bool consensus_valid,
    float consensus_frequency_hz
)
{
    if (tracker == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }


    /*======================================================
     * NO VALID CONSENSUS THIS WINDOW
     *=====================================================*/

    if (!consensus_valid)
    {
        /*
         * SEARCHING state:
         *
         * Nothing is being tracked yet.
         */

        if (tracker->state ==
            FREQUENCY_STATE_SEARCHING)
        {
            return ESP_OK;
        }


        /*
         * Candidate or stable frequency exists.
         *
         * Allow a small number of missed windows.
         */

        tracker->missed_window_count++;


        if (tracker->missed_window_count >
            TRACKER_MAX_MISSED_WINDOWS)
        {
            /*
             * Lost the signal.
             *
             * Reset tracker.
             */

            tracker->state =
                FREQUENCY_STATE_SEARCHING;


            tracker->candidate_frequency_hz =
                0.0f;


            tracker->stable_frequency_hz =
                0.0f;


            tracker->confirmation_count =
                0;


            tracker->missed_window_count =
                0;
        }


        return ESP_OK;
    }


    /*======================================================
     * VALID CONSENSUS RECEIVED
     *=====================================================*/


    /*
     * A valid frequency resets the missed-window counter.
     */

    tracker->missed_window_count = 0;


    /*======================================================
     * SEARCHING → CANDIDATE
     *=====================================================*/

    if (tracker->state ==
        FREQUENCY_STATE_SEARCHING)
    {
        tracker->candidate_frequency_hz =
            consensus_frequency_hz;


        tracker->confirmation_count =
            1;


        tracker->state =
            FREQUENCY_STATE_CANDIDATE;


        return ESP_OK;
    }


    /*======================================================
     * CANDIDATE STATE
     *=====================================================*/

    if (tracker->state ==
        FREQUENCY_STATE_CANDIDATE)
    {
        float difference =
            fabsf(
                consensus_frequency_hz -
                tracker->candidate_frequency_hz
            );


        /*
         * Frequency agrees with candidate.
         */

        if (difference <=
            TRACKER_FREQUENCY_TOLERANCE_HZ)
        {
            /*
             * Running average of confirmed observations.
             */

            tracker->candidate_frequency_hz =
                (
                    tracker->candidate_frequency_hz *
                    tracker->confirmation_count
                    +
                    consensus_frequency_hz
                )
                /
                (
                    tracker->confirmation_count +
                    1
                );


            tracker->confirmation_count++;


            /*
             * Promote candidate to STABLE.
             */

            if (tracker->confirmation_count >=
                TRACKER_CONFIRMATIONS_REQUIRED)
            {
                tracker->stable_frequency_hz =
                    tracker->candidate_frequency_hz;


                tracker->state =
                    FREQUENCY_STATE_STABLE;
            }
        }
        else
        {
            /*
             * Different frequency appeared.
             *
             * Replace old candidate and begin again.
             */

            tracker->candidate_frequency_hz =
                consensus_frequency_hz;


            tracker->confirmation_count =
                1;
        }


        return ESP_OK;
    }


    /*======================================================
     * STABLE STATE
     *=====================================================*/

    if (tracker->state ==
        FREQUENCY_STATE_STABLE)
    {
        float difference =
            fabsf(
                consensus_frequency_hz -
                tracker->stable_frequency_hz
            );


        /*
         * Frequency remains close to stable frequency.
         */

        if (difference <=
            TRACKER_FREQUENCY_TOLERANCE_HZ)
        {
            /*
             * Slowly adapt stable frequency.
             *
             * 80% previous stable value
             * 20% new observation
             */

            tracker->stable_frequency_hz =
                (
                    0.8f *
                    tracker->stable_frequency_hz
                )
                +
                (
                    0.2f *
                    consensus_frequency_hz
                );


            return ESP_OK;
        }


        /*
         * A significantly different frequency appeared.
         *
         * Start tracking it as a new candidate.
         */

        tracker->candidate_frequency_hz =
            consensus_frequency_hz;


        tracker->confirmation_count =
            1;


        tracker->state =
            FREQUENCY_STATE_CANDIDATE;


        return ESP_OK;
    }


    return ESP_OK;
}


const char *frequency_tracker_state_string(
    frequency_tracker_state_t state
)
{
    switch (state)
    {
        case FREQUENCY_STATE_SEARCHING:
            return "SEARCHING";


        case FREQUENCY_STATE_CANDIDATE:
            return "CANDIDATE";


        case FREQUENCY_STATE_STABLE:
            return "STABLE";


        default:
            return "UNKNOWN";
    }
}