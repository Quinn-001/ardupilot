#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>

// The per-cycle EKF3 runtime record is useful for short profiling sessions but
// is too verbose for ordinary embedded flight logs. Board targets opt in with
// a bounded rate; zero omits ERCT entirely.
#ifndef ESTIMATOR_CYCLE_RUNTIME_LOG_RATE_HZ
#define ESTIMATOR_CYCLE_RUNTIME_LOG_RATE_HZ 0
#endif

#if HAL_LOGGING_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX)
#include <time.h>
#endif

#if HAL_LOGGING_ENABLED
static constexpr uint8_t ESTIMATOR_RUNTIME_EKF3_ID = 0;
static constexpr uint8_t ESTIMATOR_RUNTIME_CINS_ID = 1;
static constexpr uint8_t ESTIMATOR_RUNTIME_PHASE_INPUT = 0;
static constexpr uint8_t ESTIMATOR_RUNTIME_PHASE_PREDICT = 1;
static constexpr uint8_t ESTIMATOR_RUNTIME_PHASE_COVARIANCE = 2;
static constexpr uint8_t ESTIMATOR_RUNTIME_PHASE_DELAY = 3;
static constexpr uint8_t ESTIMATOR_RUNTIME_PHASE_MAG_YAW = 4;
static constexpr uint8_t ESTIMATOR_RUNTIME_PHASE_POSVEL = 5;
static constexpr uint8_t ESTIMATOR_RUNTIME_PHASE_OTHER_FUSION = 6;
static constexpr uint8_t ESTIMATOR_RUNTIME_PHASE_OUTPUT = 7;
static constexpr uint8_t ESTIMATOR_RUNTIME_PHASE_INIT = 8;
static constexpr uint8_t ESTIMATOR_EVENT_GPS_SAMPLE = 0;
static constexpr uint8_t ESTIMATOR_EVENT_GPS_VEL_CORRECTION = 1;
static constexpr uint8_t ESTIMATOR_EVENT_GPS_POS_CORRECTION = 2;
static constexpr uint8_t ESTIMATOR_CYCLE_GPS_SAMPLE = 1U << 0;
static constexpr uint8_t ESTIMATOR_CYCLE_GPS_VEL_CORRECTION = 1U << 1;
static constexpr uint8_t ESTIMATOR_CYCLE_GPS_POS_CORRECTION = 1U << 2;
static constexpr uint32_t ESTIMATOR_RUNTIME_LOG_INTERVAL_US = 1000000U;
static constexpr uint32_t ESTIMATOR_RUNTIME_LOG_SAMPLES = 400U;

static uint64_t estimator_runtime_micros64()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec) * 1000000U + uint64_t(ts.tv_nsec) / 1000U;
#else
    return AP_HAL::micros64();
#endif
}

class EstimatorTopLevelRuntimeAccumulator {
public:
    void log_sample(uint8_t estimator_id, uint32_t elapsed_us)
    {
        sample_count++;
        total_us += elapsed_us;
        max_us = MAX(max_us, elapsed_us);

        const uint64_t log_time_us = AP_HAL::micros64();
        if (last_log_time_us == 0) {
            last_log_time_us = log_time_us;
        }
        if (sample_count < ESTIMATOR_RUNTIME_LOG_SAMPLES &&
            log_time_us - last_log_time_us < ESTIMATOR_RUNTIME_LOG_INTERVAL_US) {
            return;
        }

        AP::logger().WriteEstimatorRuntime(estimator_id,
                                           sample_count,
                                           float(total_us) / float(sample_count),
                                           float(max_us),
                                           float(elapsed_us));

        sample_count = 0;
        total_us = 0;
        max_us = 0;
        last_log_time_us = log_time_us;
    }

private:
    uint32_t sample_count = 0;
    uint64_t total_us = 0;
    uint32_t max_us = 0;
    uint64_t last_log_time_us = 0;
};

class EstimatorPhaseRuntimeAccumulator {
public:
    void log_sample(uint8_t estimator_id, uint8_t phase_id, uint32_t elapsed_us)
    {
        sample_count++;
        total_us += elapsed_us;
        max_us = MAX(max_us, elapsed_us);

        const uint64_t log_time_us = AP_HAL::micros64();
        if (last_log_time_us == 0) {
            last_log_time_us = log_time_us;
        }
        if (sample_count < ESTIMATOR_RUNTIME_LOG_SAMPLES &&
            log_time_us - last_log_time_us < ESTIMATOR_RUNTIME_LOG_INTERVAL_US) {
            return;
        }

        AP::logger().WriteEstimatorPhaseRuntime(estimator_id,
                                                phase_id,
                                                sample_count,
                                                float(total_us) / float(sample_count),
                                                float(max_us),
                                                float(elapsed_us));

        sample_count = 0;
        total_us = 0;
        max_us = 0;
        last_log_time_us = log_time_us;
    }

private:
    uint32_t sample_count = 0;
    uint64_t total_us = 0;
    uint32_t max_us = 0;
    uint64_t last_log_time_us = 0;
};

class EstimatorEventRateAccumulator {
public:
    void log_event(uint8_t estimator_id, uint8_t event_id, uint32_t count=1)
    {
        event_count += count;

        const uint64_t log_time_us = AP_HAL::micros64();
        if (last_log_time_us == 0) {
            last_log_time_us = log_time_us;
        }

        const uint64_t elapsed_us_64 = log_time_us - last_log_time_us;
        if (elapsed_us_64 < ESTIMATOR_RUNTIME_LOG_INTERVAL_US) {
            return;
        }

        const uint32_t elapsed_us = uint32_t(MIN(elapsed_us_64, uint64_t(UINT32_MAX)));
        const float rate_hz = elapsed_us > 0 ? float(event_count) * 1.0e6f / float(elapsed_us) : 0.0f;
        AP::logger().WriteEstimatorEventRate(estimator_id,
                                             event_id,
                                             event_count,
                                             elapsed_us,
                                             rate_hz);

        event_count = 0;
        last_log_time_us = log_time_us;
    }

private:
    uint32_t event_count = 0;
    uint64_t last_log_time_us = 0;
};
#endif
