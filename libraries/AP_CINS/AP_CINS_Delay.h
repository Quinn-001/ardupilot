#pragma once

#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Math/LieGroups.h>

namespace AP_CINS_Delay {

// Rebuild only the retained suffix, without consuming or copying the history.
template<typename HistoryEntry>
bool rebuild(const ObjectBuffer<HistoryEntry> &history, const Gal3F &pending, Gal3F &output)
{
    Gal3F rebuilt = Gal3F::identity();
    const uint32_t count = history.available();
    for (uint32_t i = 0; i < count; i++) {
        HistoryEntry entry;
        if (!history.peek(entry, i)) {
            return false;
        }
        Matrix3F rotation;
        entry.rot.rotation_matrix(rotation);
        rebuilt = rebuilt * Gal3F(rotation, entry.pos, entry.vel, entry.tau);
    }
    output = rebuilt * pending;
    return true;
}

} // namespace AP_CINS_Delay
