#pragma once

#include <cmath>

namespace SITL {

// Phenomenological pressure disturbance, expressed as equivalent height bias.
// Input is the mean normalized active motor command, not physical thrust.
class BaroPropwash {
public:
    float update(float input, float amplitude, float threshold, float reference, float tau, float dt)
    {
        if (!std::isfinite(input) || !std::isfinite(amplitude) ||
            !std::isfinite(threshold) || !std::isfinite(reference) ||
            !std::isfinite(tau) || !std::isfinite(dt) ||
            std::fpclassify(amplitude) == FP_ZERO || reference <= threshold || threshold < 0.0f || reference > 1.0f) {
            _bias = 0.0f;
            return _bias;
        }
        const float fraction = fminf(1.0f, fmaxf(0.0f, (input - threshold) / (reference - threshold)));
        const float target = amplitude * fraction;
        if (dt > 0.0f) {
            const float alpha = tau > 0.0f ? -expm1f(-dt / tau) : 1.0f;
            _bias += alpha * (target - _bias);
        }
        return _bias;
    }

private:
    float _bias = 0.0f;
};

}
