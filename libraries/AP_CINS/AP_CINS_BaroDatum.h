#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

// Startup alignment is learned without changing disarmed GPS aiding.
// Preparation freezes it before home capture; no learning is allowed in flight.
class CINSBaroDatum
{
public:
    enum class Phase : uint8_t { Tracking, Prepared, Frozen };
    enum class Reason : uint8_t { Acquiring, Pair, Prediction, Ready, Changed, Stale, Unavailable };

    struct Config {
        float gps_lag, baro_lag, max_age, consistency_gate;
        float innovation_gate, ground_effect_gate, failure_time, transition_rate;
        int8_t mode, gps_mode, ct_gate, bias_mode, source_policy;
        uint8_t primary;
        uint16_t calibration;

        bool operator==(const Config &c) const
        {
            // Parameters identify a datum configuration, rather than a noisy measurement.
            return same_float(gps_lag, c.gps_lag) && same_float(baro_lag, c.baro_lag) &&
                   same_float(max_age, c.max_age) && same_float(consistency_gate, c.consistency_gate) &&
                   same_float(innovation_gate, c.innovation_gate) && same_float(ground_effect_gate, c.ground_effect_gate) &&
                   same_float(failure_time, c.failure_time) && same_float(transition_rate, c.transition_rate) &&
                   mode == c.mode && gps_mode == c.gps_mode &&
                   ct_gate == c.ct_gate && bias_mode == c.bias_mode &&
                   source_policy == c.source_policy && primary == c.primary &&
                   calibration == c.calibration;
        }

    private:
        static bool same_float(float a, float b)
        {
            return memcmp(&a, &b, sizeof(a)) == 0;
        }
    };

    void reset()
    {
        _phase = Phase::Tracking;
        _has_offset = _invalid = _qualified = _prediction_valid = _dwell_valid = false;
        _count = 0;
        _offset = _pair_residual = _gps_innovation = 0.0f;
        _reason = Reason::Acquiring;
        ++_generation;
    }

    void check_config(const Config &config)
    {
        if (_has_offset && !(_config == config)) {
            if (_phase == Phase::Tracking) {
                reset();
            } else {
                invalidate();
            }
        }
    }

    void invalidate()
    {
        if (!_invalid) {
            ++_generation;
        }
        _invalid = true;
        _dwell_valid = false;
        _reason = Reason::Changed;
    }

    void reject_pair()
    {
        if (_phase == Phase::Tracking) {
            reset();
        }
        _qualified = false;
        _dwell_valid = false;
        _reason = Reason::Unavailable;
    }

    void pair(uint32_t gps_ms, float raw_down, float gps_down,
              const Config &config, bool qualified)
    {
        check_config(config);
        if (_phase != Phase::Tracking) {
            // Record readiness of a prepared snapshot, without changing its datum.
            _qualified = qualified && !_invalid;
            _sample_ms = gps_ms;
            _pair_residual = raw_down - gps_down - _offset;
            if (_phase == Phase::Prepared) {
                _prediction_valid = false;
            }
            return;
        }
        if (!qualified || !std::isfinite(raw_down) || !std::isfinite(gps_down)) {
            reject_pair();
            return;
        }
        if (_has_offset && gps_ms == _sample_ms) {
            return;
        }
        const uint32_t dt_ms = gps_ms - _sample_ms;
        if (_has_offset && dt_ms > MAX_AGE_MS) {
            reset();
        }
        const float observation = raw_down - gps_down;
        if (!_has_offset) {
            _offset = observation;
            _config = config;
            _has_offset = true;
        } else {
            const float alpha = 1.0f - expf(-0.001f * dt_ms);
            _offset += alpha * (observation - _offset);
        }
        if (_count < MIN_PAIRS) {
            ++_count;
        }
        _sample_ms = gps_ms;
        _qualified = true;
        _pair_residual = observation - _offset;
        _prediction_valid = false;
    }

    void prediction(uint32_t gps_ms, float innovation)
    {
        if (_phase == Phase::Frozen) {
            return;
        }
        _gps_innovation = innovation;
        _prediction_valid = _qualified && _has_offset && gps_ms == _sample_ms &&
                            std::isfinite(innovation);
        const float datum_epsilon = datum_tolerance();
        if (_invalid) {
            _dwell_valid = false;
            _reason = Reason::Changed;
        } else if (!_qualified) {
            _dwell_valid = false;
            _reason = Reason::Unavailable;
        } else if (!_prediction_valid || _count < MIN_PAIRS) {
            _dwell_valid = false;
            _reason = Reason::Acquiring;
        } else if (fabsf(_pair_residual) > datum_epsilon) {
            _dwell_valid = false;
            _reason = Reason::Pair;
        } else if (fabsf(innovation) > OBSERVER_GPS_TOLERANCE_M) {
            _dwell_valid = false;
            _reason = Reason::Prediction;
        } else {
            if (!_dwell_valid) {
                _dwell_ms = gps_ms;
                _dwell_valid = true;
            }
            _reason = gps_ms - _dwell_ms >= DWELL_MS ? Reason::Ready : Reason::Acquiring;
        }
    }

    bool ready(uint32_t now_ms) const
    {
        return _has_offset && !_invalid && _qualified && _prediction_valid && _dwell_valid &&
               _reason == Reason::Ready && now_ms - _sample_ms <= MAX_AGE_MS;
    }

    bool prepare(uint32_t now_ms)
    {
        if (!ready(now_ms)) {
            return false;
        }
        _phase = Phase::Prepared;
        ++_generation;
        return true;
    }

    void commit(uint32_t now_ms)
    {
        if (_phase == Phase::Tracking) {
            prepare(now_ms); // Shadow CINS may not participate in the active arm checks.
        }
        _invalid = _phase != Phase::Prepared || !ready(now_ms);
        _phase = Phase::Frozen;
        ++_generation;
    }

    bool frozen_valid() const
    {
        return _phase == Phase::Frozen && _has_offset && !_invalid;
    }
    bool has_offset() const
    {
        return _has_offset;
    }
    bool matches_config(const Config &config) const
    {
        return _has_offset && _config == config;
    }
    Phase phase() const
    {
        return _phase;
    }
    float offset() const
    {
        return _offset;
    }
    float pair_residual() const
    {
        return _pair_residual;
    }
    float gps_innovation() const
    {
        return _gps_innovation;
    }
    uint8_t count() const
    {
        return _count;
    }
    uint32_t generation() const
    {
        return _generation;
    }
    uint32_t sample_ms() const
    {
        return _sample_ms;
    }
    Reason reason(uint32_t now_ms) const
    {
        return _has_offset && now_ms - _sample_ms > MAX_AGE_MS ? Reason::Stale : _reason;
    }

private:
    float datum_tolerance() const
    {
        return fminf(0.25f, fminf(_config.consistency_gate * 0.25f,
                                  fminf(_config.innovation_gate * 0.25f, _config.ground_effect_gate * 0.5f)));
    }

    // Receiver height noise can exceed the datum-pair tolerance while the
    // observer is settled enough to capture home. Keep these checks separate.
    static constexpr float OBSERVER_GPS_TOLERANCE_M = 1.0f;
    static constexpr uint8_t MIN_PAIRS = 5;
    static constexpr uint32_t MAX_AGE_MS = 500;
    static constexpr uint32_t DWELL_MS = 2000;
    Config _config {};
    Phase _phase {Phase::Tracking};
    Reason _reason {Reason::Acquiring};
    float _offset {}, _pair_residual {}, _gps_innovation {};
    uint32_t _sample_ms {}, _dwell_ms {}, _generation {};
    uint8_t _count {};
    bool _has_offset {}, _invalid {}, _qualified {}, _prediction_valid {}, _dwell_valid {};
};
