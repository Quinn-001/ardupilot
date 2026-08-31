/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_CINS_ENABLED

#include "AP_ExternalAHRS_CINS.h"
#include <GCS_MAVLink/GCS.h>

AP_ExternalAHRS_CINS::AP_ExternalAHRS_CINS(AP_ExternalAHRS *_frontend,
                                           AP_ExternalAHRS::state_t &_state,
                                           AP_CINS *&cins_ptr) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    // Make the dynamically allocated CINS parameter subgroup accessible
    cins_ptr = &cins;

    // AP_Param::load_all() ran before cins_ptr was valid, so explicitly
    // restore the saved CINS parameters now that the object exists.
    AP_Param::load_object_from_eeprom(&cins, AP_CINS::var_info);

    cins.init();

    // CINS does not provide new sensors, only a state estimate
    set_default_sensors(0);
}

void AP_ExternalAHRS_CINS::update(void)
{
    cins.update();

    if (!reported_running && AP_HAL::millis() >= 5000) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CINS backend running");
        reported_running = true;
    }
    if (!reported_initialised && cins.initialised()) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CINS initialised");
        reported_initialised = true;
    }
    if (!reported_delay_buffer_failure && cins.delay_buffer_failed()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "CINS delay buffer full");
        reported_delay_buffer_failure = true;
    }

    WITH_SEMAPHORE(state.sem);

    state.accel = cins.get_accel();
    state.gyro = cins.get_gyro();
    state.quat = cins.get_quat();
    state.location = cins.get_location();
    state.velocity = cins.get_velocity();
    state.have_origin = cins.get_origin(state.origin);
    state.have_quaternion = true;
    state.have_location = state.have_origin;
    state.have_velocity = true;
}

/*
  get filter status, assume all OK if we have GPS lock
 */
void AP_ExternalAHRS_CINS::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    if (cins.healthy()) {
        status.flags.initalized = 1;
        status.flags.attitude = 1;
        status.flags.vert_vel = 1;
        status.flags.vert_pos = 1;
        status.flags.horiz_vel = 1;
        status.flags.horiz_pos_rel = 1;
        status.flags.horiz_pos_abs = 1;
        status.flags.pred_horiz_pos_rel = 1;
        status.flags.pred_horiz_pos_abs = 1;
        status.flags.using_gps = 1;
    }
}

/*
  get filter variances
 */
bool AP_ExternalAHRS_CINS::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    return cins.get_variances(velVar, posVar, hgtVar, magVar, tasVar);
}

#endif // AP_EXTERNAL_AHRS_CINS_ENABLED
