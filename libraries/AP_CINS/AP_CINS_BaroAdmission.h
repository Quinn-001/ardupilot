#pragma once

#include <cmath>

// Per-packet admission is independent of persistent height-source selection.
struct CINSBaroAdmission {
    bool accepted;
    bool innovation_bad;
    bool ground_effect_bad;

    template <typename T>
    static CINSBaroAdmission evaluate(bool finite_candidate, T innovation_down,
                                     bool ground_effect_expected, float innovation_gate,
                                     float ground_effect_gate)
    {
        if (!finite_candidate || !std::isfinite(innovation_down)) {
            return {false, false, false};
        }
        const bool innovation_bad = innovation_down > innovation_gate || innovation_down < -innovation_gate;
        const bool ground_effect_bad = ground_effect_expected &&
            (innovation_down > ground_effect_gate || innovation_down < -ground_effect_gate);
        return {!innovation_bad && !ground_effect_bad, innovation_bad, ground_effect_bad};
    }
};
