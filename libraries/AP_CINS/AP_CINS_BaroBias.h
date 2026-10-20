#pragma once

#include <cmath>
#include <stdint.h>

// Paired-sensor measurement conditioning, separate from the CINS IMU bias model.
class CINSBaroBias {
public:
    struct Config {
        float time_constant;
        float max_rate;
        float max_bias;
        float jump_rate;
        float dwell;
    };
    enum class Result : uint8_t { Held, Baseline, Gap, Jump, Qualifying, Updated, Configuration };

    float bias() const { return estimate; }
    void reset() { *this = CINSBaroBias{}; }

    Result update(const Config &cfg, uint32_t timestamp_ms, float datum_down,
                  float gps_down, float gps_vz, float trust, bool usable)
    {
        if (!std::isfinite(cfg.time_constant) || cfg.time_constant < 1.0f || cfg.time_constant > 300.0f ||
            !std::isfinite(cfg.max_rate) || cfg.max_rate < 0.001f || cfg.max_rate > 1.0f ||
            !std::isfinite(cfg.max_bias) || cfg.max_bias < 0.1f || cfg.max_bias > 20.0f ||
            !std::isfinite(cfg.jump_rate) || cfg.jump_rate < 0.1f || cfg.jump_rate > 10.0f ||
            !std::isfinite(cfg.dwell) || cfg.dwell < 0.0f || cfg.dwell > 30.0f) {
            have_history = false;
            stable_time = 0.0f;
            return Result::Configuration;
        }
        if (!usable || !std::isfinite(datum_down) || !std::isfinite(gps_down) ||
            !std::isfinite(gps_vz) || !std::isfinite(trust) || trust <= 0.0f || trust > 1.0f) {
            have_history = false;
            stable_time = 0.0f;
            return Result::Held;
        }
        // A held packet must never accumulate adaptation or qualifying time.
        if (have_history && timestamp_ms == last_ms) {
            return Result::Held;
        }
        const float difference = datum_down - gps_down;
        const float dt = (timestamp_ms - last_ms) * 0.001f;
        const bool baseline = !have_history;
        const bool gap = !baseline && dt > 1.0f;
        const bool jump = !baseline && !gap &&
            (fabsf(difference - last_difference) > cfg.jump_rate * dt ||
             fabsf(gps_down - last_gps_down - 0.5f * (gps_vz + last_vz) * dt) > cfg.jump_rate * dt);
        last_ms = timestamp_ms;
        last_difference = difference;
        last_gps_down = gps_down;
        last_vz = gps_vz;
        have_history = true;
        if (baseline || gap || jump) {
            stable_time = 0.0f;
            return baseline ? Result::Baseline : (gap ? Result::Gap : Result::Jump);
        }
        stable_time = fminf(stable_time + dt, cfg.dwell);
        if (stable_time < cfg.dwell) {
            return Result::Qualifying;
        }
        const float alpha = trust * -expm1f(-dt / cfg.time_constant);
        const float step = clamp(alpha * (difference - estimate), -cfg.max_rate * dt, cfg.max_rate * dt);
        // Projection allows an estimate on either bound to return inward.
        estimate = clamp(estimate + step, -cfg.max_bias, cfg.max_bias);
        return Result::Updated;
    }

private:
    static float clamp(float value, float lower, float upper)
    {
        return fmaxf(lower, fminf(value, upper));
    }
    float estimate = 0.0f;
    float stable_time = 0.0f;
    float last_difference = 0.0f;
    float last_gps_down = 0.0f;
    float last_vz = 0.0f;
    uint32_t last_ms = 0;
    bool have_history = false;
};
