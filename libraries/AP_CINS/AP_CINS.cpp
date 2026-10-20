/*
  CINS state estimator for AP_AHRS, developed by Mr Patrick Wiltshire and Dr Pieter Van Goor
 */

#include "AP_CINS.h"
#include "AP_CINS_Delay.h"
#include "AP_CINS_BaroAdmission.h"

#if AP_EXTERNAL_AHRS_CINS_ENABLED

#include <AP_DAL/AP_DAL.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_EstimatorRuntimeLogging.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// gains tested for 5Hz GPS
#define CINS_GAIN_GPSPOS_ATT (1.0E-5)
#define CINS_GAIN_GPSVEL_ATT (1.0E-4)

// Gains for new CINS method
#define CINS_GAIN_GPSPOS_POS (10.0)
#define CINS_GAIN_GPSVEL_VEL (10.0)
#define CINS_GAIN_MAG (0.0)//(0.5)
#define CINS_GAIN_Q11 (0.001)
#define CINS_GAIN_Q22 (0.001)
// Gains for the Gyro and Accel Biases
#define CINS_GAIN_POS_GYR_BIAS (1E-7)
#define CINS_GAIN_VEL_GYR_BIAS (1E-6)
#define CINS_GAIN_MAG_GYR_BIAS (0.0)//(0.005)
#define CINS_SAT_GYR_BIAS (0.02) // 0.0873 rad/s is 5 deg/s
#define CINS_GAIN_POS_ACC_BIAS (1E-3)
#define CINS_GAIN_VEL_ACC_BIAS (1E-3)
#define CINS_GAIN_MAG_ACC_BIAS (0.0)//(0.005)
#define CINS_SAT_ACC_BIAS (0.5)
// Naive GPS Time Delay settings
#define CINS_GPS_DELAY (0.2) // seconds of delay
// Variance gain for health checking
#define CINS_VAR_LOWPASS (0.01) // low pass of measurement errors
#define CINS_VAR_SCALE (2.) // accept measurements with normalised variance smaller than this

/*
  CINS is very fast, but stack hungry, using Os keeps stack usage down
 */
#pragma GCC optimize("Os")

// table of user settable parameters
const AP_Param::GroupInfo AP_CINS::var_info[] = {

    // @Param: GPATT
    // @DisplayName: CINS GPS position attitude gain
    // @Description: CINS GPS position attitude gain
    // @Range: 0.0001 0.001
    // @User: Advanced
    AP_GROUPINFO("GPATT", 1, AP_CINS, gains.gpspos_att, CINS_GAIN_GPSPOS_ATT),

    // @Param: GVATT
    // @DisplayName: CINS GPS velocity attitude gain
    // @Description: CINS GPS velocity attitude gain
    // @Range: 0.0001 0.001
    // @User: Advanced
    AP_GROUPINFO("GVATT", 2, AP_CINS, gains.gpsvel_att, CINS_GAIN_GPSVEL_ATT),

    // @Param: GPLAG
    // @DisplayName: CINS GPS lag
    // @Description: CINS GPS lag
    // @Range: 0.0 0.2
    // @User: Advanced
    // @Units: s
    AP_GROUPINFO("GPLAG", 3, AP_CINS, gains.gps_lag, CINS_GPS_DELAY),

    // @Param: GPPOS
    // @DisplayName: CINS GPS position gain
    // @Description: CINS GPS position gain
    // @Range: 0.1 10.0
    // @User: Advanced
    AP_GROUPINFO("GPPOS", 4, AP_CINS, gains.gps_pos, CINS_GAIN_GPSPOS_POS),

    // @Param: GPVEL
    // @DisplayName: CINS GPS velocity gain
    // @Description: CINS GPS velocity gain
    // @Range: 0.1 10.0
    // @User: Advanced
    AP_GROUPINFO("GPVEL", 5, AP_CINS, gains.gps_vel, CINS_GAIN_GPSVEL_VEL),

    // @Param: MGATT
    // @DisplayName: CINS magnetometer attitude gain
    // @Description: CINS magnetometer (compass) attitude gain
    // @Range: 0.1 10.0
    // @User: Advanced
    AP_GROUPINFO("MGATT", 6, AP_CINS, gains.mag_att, CINS_GAIN_MAG),

    // @Param: INTQV
    // @DisplayName: CINS internal velocity Q gain
    // @Description: CINS internal velocity Q gain
    // @Range: 0.0001 0.1
    // @User: Advanced
    AP_GROUPINFO("INTQV", 7, AP_CINS, gains.Q11, CINS_GAIN_Q11),

    // @Param: INTQP
    // @DisplayName: CINS internal position Q gain
    // @Description: CINS internal position Q gain
    // @Range: 0.0001 0.1
    // @User: Advanced
    AP_GROUPINFO("INTQP", 8, AP_CINS, gains.Q22, CINS_GAIN_Q22),

    // @Param: GPPGB
    // @DisplayName: CINS GPS position - gyro bias gain
    // @Description: CINS GPS position - gyro bias gain
    // @Range: 0.0000001 0.000001
    // @User: Advanced
    AP_GROUPINFO("GPPGB", 9, AP_CINS, gains.gps_pos_gyr_bias, CINS_GAIN_POS_GYR_BIAS),

    // @Param: GPVGB
    // @DisplayName: CINS GPS velocity - gyro bias gain
    // @Description: CINS GPS velocity - gyro bias gain
    // @Range: 0.0000001 0.000001
    // @User: Advanced
    AP_GROUPINFO("GPVGB", 10, AP_CINS, gains.gps_vel_gyr_bias, CINS_GAIN_VEL_GYR_BIAS),

    // @Param: MAGGB
    // @DisplayName: CINS magnetometer - gyro bias gain
    // @Description: CINS magnetometer (compass) gyro bias gain
    // @Range: 0.001 0.0001
    // @User: Advanced
    AP_GROUPINFO("MAGGB", 11, AP_CINS, gains.mag_gyr_bias, CINS_GAIN_MAG_GYR_BIAS),

    // @Param: SATGB
    // @DisplayName: CINS gyro bias saturation
    // @Description: CINS gyro bias saturation
    // @Range: 0.05 0.1
    // @User: Advanced
    AP_GROUPINFO("SATGB", 12, AP_CINS, gains.sat_gyr_bias, CINS_SAT_GYR_BIAS),


    // @Param: GPPAB
    // @DisplayName: CINS GPS position - accel. bias gain
    // @Description: CINS GPS position - accel. bias gain
    // @Range: 0.0000001 0.000001
    // @User: Advanced
    AP_GROUPINFO("GPPAB", 13, AP_CINS, gains.gps_pos_acc_bias, CINS_GAIN_POS_ACC_BIAS),

    // @Param: GPVAB
    // @DisplayName: CINS GPS velocity - accel. bias gain
    // @Description: CINS GPS velocity - accel. bias gain
    // @Range: 0.0000001 0.000001
    // @User: Advanced
    AP_GROUPINFO("GPVAB", 14, AP_CINS, gains.gps_vel_acc_bias, CINS_GAIN_VEL_ACC_BIAS),

    // @Param: MAGAB
    // @DisplayName: CINS magnetometer - accel. bias gain
    // @Description: CINS magnetometer (compass) accel. bias gain
    // @Range: 0.001 0.0001
    // @User: Advanced
    AP_GROUPINFO("MAGAB", 15, AP_CINS, gains.mag_acc_bias, CINS_GAIN_MAG_ACC_BIAS),

    // @Param: SATAB
    // @DisplayName: CINS accel. bias saturation
    // @Description: CINS accel. bias saturation
    // @Range: 0.05 0.1
    // @User: Advanced
    AP_GROUPINFO("SATAB", 16, AP_CINS, gains.sat_acc_bias, CINS_SAT_ACC_BIAS),

    // @Param: VARLP
    // @DisplayName: CINS variance low-pass gain
    // @Description: CINS variance low-pass gain used for health checks
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("VARLP", 17, AP_CINS, gains.variance_lowpass, CINS_VAR_LOWPASS),

    // @Param: VARSC
    // @DisplayName: CINS variance scale
    // @Description: CINS variance scale used for health checks
    // @Range: 1.0 5.0
    // @User: Advanced
    AP_GROUPINFO("VARSC", 18, AP_CINS, gains.variance_scale, CINS_VAR_SCALE),

    // @Param: GPMOD
    // @DisplayName: CINS GPS correction scheduling mode
    // @Description: Selects CINS correction mode. Mode 2 uses finite event-based GPS and magnetometer corrections.
    // @Values: 0:Held measurement at IMU rate,1:Fresh GPS sample Euler,2:Fresh GPS finite map
    // @User: Advanced
    AP_GROUPINFO("GPMOD", 19, AP_CINS, gains.gps_correction_mode, 0),

    // @Param: LOGR
    // @DisplayName: CINS detail logging rate
    // @Description: Rate for CINS and CIN2 detail records. A nonzero value also enables CIBD composite diagnostics at the GPS sample rate. Runtime and event summaries retain their independent one-Hz reporting. Set to zero to disable CINS detail records.
    // @Range: 0 400
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("LOGR", 20, AP_CINS, detail_log_rate, 10),

    // @Param: MEMR
    // @DisplayName: CINS memory diagnostic logging rate
    // @Description: Rate for CIMD heap diagnostics. The record contains total free heap, largest contiguous free block, and CINS state flags. Set to zero to disable it.
    // @Range: 0 20
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("MEMR", 21, AP_CINS, memory_log_rate, 0),

    // @Param: BAMOD
    // @DisplayName: CINS barometer composite mode
    // @Description: Selects the experimental GPS north/east plus barometric down position measurement. Height uses GPS while disarmed; barometer alignment and height-source settings are frozen at arming. GPMOD=0 retains continuous barometer aiding unless BACT enables whole-position gating. GPMOD=2 may reject individual bad position events before the same smooth fallback. GPMOD=1 uses GPS position only. Changes to height-source settings are deferred until disarm.
    // @Values: 0:Disabled,1:Composite position vector
    // @User: Advanced
    AP_GROUPINFO("BAMOD", 22, AP_CINS, baro_config.mode, 0),

    // @Param: BALAG
    // @DisplayName: CINS barometer lag
    // @Description: Estimated delay between the physical pressure measurement and its barometer timestamp. This is used with GPLAG to select a barometer sample representing the same time as the delayed GPS position.
    // @Range: 0.0 0.2
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("BALAG", 23, AP_CINS, baro_config.lag, 0.0),

    // @Param: BAAGE
    // @DisplayName: CINS barometer maximum sample age
    // @Description: Maximum allowed time mismatch between the selected barometer sample and the delayed GPS measurement time. Unavailable candidates are held in ungated CT or rejected by gated CT and finite modes until persistent GPS fallback.
    // @Range: 0.005 0.1
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("BAAGE", 24, AP_CINS, baro_config.max_age, 0.05),

    // @Param: BAGAT
    // @DisplayName: CINS barometer/GPS consistency gate
    // @Description: Difference between datum-aligned barometric Down and GPS Down used as a source-consistency fault signal. It does not cause an immediate GPS source switch.
    // @Range: 0.1 50.0
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("BAGAT", 25, AP_CINS, baro_config.gate, 10.0),

    // @Param: BAIGT
    // @DisplayName: CINS barometer innovation gate
    // @Description: Maximum absolute delayed barometric Down innovation accepted by finite mode and by continuous mode when BACT is enabled. Rejection disables the complete position measurement correction.
    // @Range: 0.1 10.0
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("BAIGT", 26, AP_CINS, baro_config.innovation_gate, 1.0),

    // @Param: BAGEF
    // @DisplayName: CINS barometer ground-effect gate
    // @Description: Maximum absolute delayed barometric Down innovation accepted by finite mode and by continuous mode with BACT enabled while takeoff or touchdown ground effect is expected. Rejections caused by this ground-effect gate do not advance BAFLT.
    // @Range: 0.1 5.0
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("BAGEF", 27, AP_CINS, baro_config.ground_effect_gate, 0.5),

    // @Param: BAFLT
    // @DisplayName: CINS barometer persistent-fault time
    // @Description: Time for which barometer aiding must remain persistently inconsistent or unavailable before BAMOD latches a smooth GPS-height fallback for the rest of the armed flight. Expected ground-effect innovation rejections do not contribute to this timer.
    // @Range: 0.5 10.0
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("BAFLT", 28, AP_CINS, baro_config.failure_time, 2.0),

    // @Param: BASLW
    // @DisplayName: CINS GPS fallback transition rate
    // @Description: Maximum rate at which the temporary GPS-height alignment offset is removed after a persistent barometer fault.
    // @Range: 0.1 5.0
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("BASLW", 29, AP_CINS, baro_config.transition_rate, 0.5),

    // @Param: BACT
    // @DisplayName: CINS continuous barometer position gate
    // @Description: Enables whole-position measurement rejection in GPMOD=0 using BAIGT and BAGEF. Acceptance is held between GPS events. IMU, velocity and common auxiliary corrections continue; BAFLT triggers the existing GPS fallback. Requires BAMOD=1. Finite mode is unchanged.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("BACT", 30, AP_CINS, baro_config.ct_gate, 0),


    // @Param: BBMOD
    // @DisplayName: CINS barometer bias adaptation
    // @Description: Experimental bounded GPS-referenced barometer bias adaptation after accepted position events. Requires BAMOD=1 and GPMOD=0 or 2. GPS must supply healthy accuracy and vertical velocity reports. The current measurement uses the previous bias; adaptation affects the next event. GPS height error can be interpreted as barometer bias.
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("BBMOD", 31, AP_CINS, baro_config.bias_mode, 0),

    // @Param: BBTC
    // @DisplayName: CINS barometer bias time constant
    // @Description: Time constant of qualified paired-sensor bias adaptation.
    // @Range: 1 300
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("BBTC", 32, AP_CINS, baro_config.bias_tc, 30.0),

    // @Param: BBRAT
    // @DisplayName: CINS barometer bias rate limit
    // @Description: Maximum change rate of the estimated barometer bias.
    // @Range: 0.001 1
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("BBRAT", 33, AP_CINS, baro_config.bias_rate, 0.05),

    // @Param: BBLIM
    // @DisplayName: CINS barometer bias magnitude limit
    // @Description: Maximum magnitude of adaptive barometer bias relative to the frozen startup datum.
    // @Range: 0.1 20
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("BBLIM", 34, AP_CINS, baro_config.bias_limit, 5.0),

    // @Param: BBVAC
    // @DisplayName: CINS barometer bias GPS vertical accuracy
    // @Description: GPS vertical accuracy threshold for bias adaptation. Missing reports or values at or above this threshold hold adaptation.
    // @Range: 0.1 10
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("BBVAC", 35, AP_CINS, baro_config.bias_vacc, 2.0),

    // @Param: BBSAC
    // @DisplayName: CINS barometer bias GPS speed accuracy
    // @Description: GPS speed accuracy threshold for bias adaptation. Missing reports or values at or above this threshold hold adaptation.
    // @Range: 0.1 2
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("BBSAC", 36, AP_CINS, baro_config.bias_sacc, 0.5),

    // @Param: BBJMP
    // @DisplayName: CINS barometer bias jump detection
    // @Description: Maximum rate of paired height disagreement change or GPS height change unexplained by GPS vertical velocity. Exceeding either restarts qualification.
    // @Range: 0.1 10
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("BBJMP", 37, AP_CINS, baro_config.bias_jump, 1.0),

    // @Param: BBDWL
    // @DisplayName: CINS barometer bias qualification time
    // @Description: Required consecutive qualified time before adapting bias. Invalid data, rejected position events, jumps or GPS gaps over one second restart qualification.
    // @Range: 0 30
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("BBDWL", 38, AP_CINS, baro_config.bias_dwell, 3.0),

    // @Param: BASRC
    // @DisplayName: CINS barometer height-source policy
    // @Description: Selects persistent-fault GPS fallback or a fixed barometer composite in GPMOD=2 with BAMOD=1. Fixed mode retains sample health, timing, datum, innovation and ground-effect gates; rejected packets omit the whole position correction without substituting GPS. Velocity aiding continues. Frozen at arming; CT and GPS-only modes are unchanged.
    // @Values: 0:Managed GPS fallback,1:Fixed barometer composite
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("BASRC", 39, AP_CINS, baro_config.source_policy, 0),

    AP_GROUPEND
};

Vector3F computeRotationCorrection(const Vector3F& v1, const Vector3F& v2, const ftype& gain, const ftype& dt);

// constructor
AP_CINS::AP_CINS(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}


/*
  initialise the filter
 */
void AP_CINS::init(void)
{
    //Initialise XHat and ZHat as stationary at the origin
    state.XHat = Gal3F::identity();
    state.ZHat = SIM23::identity();

    state.gyr_bias.zero();
    state.gyr_bias_gain_mat.rot.zero();
    state.gyr_bias_gain_mat.vel.zero();
    state.gyr_bias_gain_mat.pos.zero();

    state.acc_bias.zero();
    state.acc_bias_gain_mat.rot.zero();
    state.acc_bias_gain_mat.vel.zero();
    state.acc_bias_gain_mat.pos.zero();

    delayer.YR = Gal3F::identity();
    delayer.history_increment = Gal3F::identity();
    delayer.history_increment_count = 0;
    delayer.gps_correction_dt = 0.0;
    delayer.gps_correction_pending = false;
    delayer.baro_down = nanf("");
    delayer.baro_result = BaroCandidateResult::Disabled;
    delayer.baro_armed_event = false;
    delayer.datum_generation = 0;
    delayer.ground_effect_expected = false;
    delayer.position_correction_valid = true;
    delayer.last_gps_vel_correction_applied = false;
    delayer.last_gps_pos_correction_applied = false;
    delayer.buffer_failed = false;
    delayer.history_released_for_compass_calibration = false;

    reset_baro_composite();

    heapVars.I3.identity();
    heapVars.zero_vector.zero();
    heapVars.gravity_vector = Vector3F(0,0,GRAVITY_MSS);

    state.variances.velVar = 10.0;
    state.variances.posVar = 10.0;
    state.variances.magVar = 1.0;
    state.variances.steps_to_initialise = 100;
}

void AP_CINS::prepare_for_compass_calibration(void)
{
    if (delayer.history_released_for_compass_calibration) {
        return;
    }
    update_baro_history();

    // ObjectBuffer keeps one object of backing storage even at size zero, but
    // this releases the CINS_MAX_HISTORY allocation before compass calibration
    // requests its 300-sample buffer and dedicated thread stack. ArduPilot
    // requires a reboot after compass calibration, so do not reallocate history
    // beside the calibrator's permanently allocated thread stack.
    if (!delayer.stamped_inputs.set_size(0)) {
        delayer.buffer_failed = true;
        return;
    }
    delayer.history_released_for_compass_calibration = true;
}
/*
  update function, called at loop rate
 */
void AP_CINS::update(void)
{
    WITH_SEMAPHORE(datum_sem);
    auto &dal = AP::dal();
    update_baro_arming();
    write_memory_diagnostic(dal.micros64());

    if (delayer.history_released_for_compass_calibration) {
        return;
    }
    update_baro_history();
#if HAL_LOGGING_ENABLED
    const uint64_t update_start_us = estimator_runtime_micros64();
    static EstimatorPhaseRuntimeAccumulator init_runtime;
    static EstimatorPhaseRuntimeAccumulator input_runtime;
    static EstimatorPhaseRuntimeAccumulator mag_yaw_runtime;
    static EstimatorEventRateAccumulator gps_sample_rate;
    static EstimatorEventRateAccumulator gps_vel_correction_rate;
    static EstimatorEventRateAccumulator gps_pos_correction_rate;
    bool gps_sample_received = false;
    uint32_t input_elapsed_us = 0;
    uint64_t phase_start_us;
#endif

    if (!done_yaw_init) {
#if HAL_LOGGING_ENABLED
        phase_start_us = estimator_runtime_micros64();
#endif
        done_yaw_init = init_yaw();
#if HAL_LOGGING_ENABLED
        init_runtime.log_sample(ESTIMATOR_RUNTIME_CINS_ID,
                                ESTIMATOR_RUNTIME_PHASE_INIT,
                                uint32_t(estimator_runtime_micros64() - phase_start_us));
#endif
    }

#if HAL_LOGGING_ENABLED
    phase_start_us = estimator_runtime_micros64();
#endif
    const auto &ins = dal.ins();

    // Get delta angle and convert to gyro rad/s
    const uint8_t gyro_index = ins.get_first_usable_gyro();
    Vector3f delta_angle;
    float dangle_dt;
    if (!ins.get_delta_angle(gyro_index, delta_angle, dangle_dt) || dangle_dt <= 0) {
        // can't update, no delta angle
#if HAL_LOGGING_ENABLED
        input_elapsed_us += uint32_t(estimator_runtime_micros64() - phase_start_us);
        input_runtime.log_sample(ESTIMATOR_RUNTIME_CINS_ID,
                                 ESTIMATOR_RUNTIME_PHASE_INPUT,
                                 input_elapsed_us);
#endif
        return;
    }
    // turn delta angle into a gyro in radians/sec
    const Vector3F gyro = (delta_angle / dangle_dt).toftype();

    // get delta velocity and convert to accel m/s/s
    Vector3f delta_velocity;
    float dvel_dt;
    if (!ins.get_delta_velocity(gyro_index, delta_velocity, dvel_dt) || dvel_dt <= 0) {
        // can't update, no delta velocity
#if HAL_LOGGING_ENABLED
        input_elapsed_us += uint32_t(estimator_runtime_micros64() - phase_start_us);
        input_runtime.log_sample(ESTIMATOR_RUNTIME_CINS_ID,
                                 ESTIMATOR_RUNTIME_PHASE_INPUT,
                                 input_elapsed_us);
#endif
        return;
    }
    // turn delta velocity into a accel vector
    const Vector3F accel = (delta_velocity / dvel_dt).toftype();
#if HAL_LOGGING_ENABLED
    input_elapsed_us += uint32_t(estimator_runtime_micros64() - phase_start_us);
#endif

    delayer.last_gps_vel_correction_applied = false;
    delayer.last_gps_pos_correction_applied = false;
    if (done_yaw_init && state.have_origin) {
        update_imu(gyro, accel, dangle_dt);
    }

#if HAL_LOGGING_ENABLED
    const bool position_correction_valid_used = delayer.position_correction_valid;
    const uint8_t position_source_used = uint8_t(baro_state.source);
    phase_start_us = estimator_runtime_micros64();
#endif
    // see if we have new GPS data
    const auto &gps = dal.gps();
    if (gps.status() >= AP_DAL_GPS::GPS_OK_FIX_3D) {
        const uint32_t last_gps_fix_ms = gps.last_message_time_ms(0);
        if (last_gps_update_ms != last_gps_fix_ms) {
            // don't allow for large gain if we lose and regain GPS
            const float gps_dt = last_gps_update_ms == 0 ? 0.0 :
                                 MIN((last_gps_fix_ms - last_gps_update_ms)*0.001, 1);
            last_gps_update_ms = last_gps_fix_ms;
            const auto &loc = gps.location();
            if (!state.have_origin) {
                state.have_origin = true;
                state.origin = loc;

#if HAL_LOGGING_ENABLED
                AP::logger().WriteEstimatorReset(dal.micros64(), dal.millis(), 1, DAL_CORE(0),
                                                AP_Logger::EstimatorReset::ORIGIN, 255);
#endif
            }
            const auto & vel = gps.velocity();
            const Vector3d pos = state.origin.get_distance_NED_double(loc);
            const Vector3F gps_pos = pos.toftype();
            ftype baro_down = nanf("");
            const BaroCandidateResult baro_result =
                get_baro_candidate(gps_pos, last_gps_fix_ms, baro_down);
            const bool ground_effect_expected =
                dal.get_takeoff_expected() || dal.get_touchdown_expected();

            update_gps(gps_pos, vel.toftype(), gps_dt,
                       baro_result, baro_down, ground_effect_expected);
#if HAL_LOGGING_ENABLED
            gps_sample_received = true;
#endif
        }
    }
#if HAL_LOGGING_ENABLED
    input_elapsed_us += uint32_t(estimator_runtime_micros64() - phase_start_us);
    input_runtime.log_sample(ESTIMATOR_RUNTIME_CINS_ID,
                             ESTIMATOR_RUNTIME_PHASE_INPUT,
                             input_elapsed_us);
#endif

#if HAL_LOGGING_ENABLED
    phase_start_us = estimator_runtime_micros64();
#endif
    update_attitude_from_compass();
#if HAL_LOGGING_ENABLED
    mag_yaw_runtime.log_sample(ESTIMATOR_RUNTIME_CINS_ID,
                               ESTIMATOR_RUNTIME_PHASE_MAG_YAW,
                               uint32_t(estimator_runtime_micros64() - phase_start_us));
#endif

#if HAL_LOGGING_ENABLED
    const uint32_t update_elapsed_us = uint32_t(estimator_runtime_micros64() - update_start_us);
    static EstimatorTopLevelRuntimeAccumulator top_level_runtime;
    top_level_runtime.log_sample(ESTIMATOR_RUNTIME_CINS_ID,
                                 update_elapsed_us);
#if ESTIMATOR_CYCLE_RUNTIME_LOG_RATE_HZ > 0
    // Raw cycle records are deliberately an opt-in profiling feature: they can
    // add substantial logger traffic on embedded targets.
    static uint32_t last_cycle_runtime_log_ms;
    const uint32_t cycle_runtime_log_interval_ms =
        1000U / ESTIMATOR_CYCLE_RUNTIME_LOG_RATE_HZ;
    const uint32_t cycle_runtime_now_ms = AP_HAL::millis();
    if (cycle_runtime_now_ms - last_cycle_runtime_log_ms >= cycle_runtime_log_interval_ms) {
        last_cycle_runtime_log_ms = cycle_runtime_now_ms;
        uint8_t event_mask = 0;
        if (gps_sample_received) {
            event_mask |= ESTIMATOR_CYCLE_GPS_SAMPLE;
        }
        if (delayer.last_gps_vel_correction_applied) {
            event_mask |= ESTIMATOR_CYCLE_GPS_VEL_CORRECTION;
        }
        if (delayer.last_gps_pos_correction_applied) {
            event_mask |= ESTIMATOR_CYCLE_GPS_POS_CORRECTION;
        }
        AP::logger().WriteEstimatorCycleRuntime(ESTIMATOR_RUNTIME_CINS_ID,
                                                event_mask,
                                                float(update_elapsed_us));
    }
#endif
    if (gps_sample_received) {
        gps_sample_rate.log_event(ESTIMATOR_RUNTIME_CINS_ID,
                                  ESTIMATOR_EVENT_GPS_SAMPLE);
    }
    if (delayer.last_gps_vel_correction_applied) {
        gps_vel_correction_rate.log_event(ESTIMATOR_RUNTIME_CINS_ID,
                                          ESTIMATOR_EVENT_GPS_VEL_CORRECTION);
    }
    if (delayer.last_gps_pos_correction_applied) {
        gps_pos_correction_rate.log_event(ESTIMATOR_RUNTIME_CINS_ID,
                                          ESTIMATOR_EVENT_GPS_POS_CORRECTION);
    }
#endif

    // Write CINS detail messages at their own bounded rate. This deliberately
    // does not affect estimator execution or the independent runtime/event
    // summaries above.
    const int16_t configured_detail_log_rate = detail_log_rate.get();
    const uint32_t detail_log_interval_ms = configured_detail_log_rate > 0 ?
        1000U / uint16_t(configured_detail_log_rate) : 0;
    const uint32_t now_ms = AP_HAL::millis();
    if (configured_detail_log_rate <= 0 ||
        now_ms - last_detail_log_ms < detail_log_interval_ms) {
        return;
    }
    last_detail_log_ms = now_ms;

    // Record the held acceptance decision and actual module application at the
    // existing detail rate, including CT cycles between GPS events.
    if (baro_mode() == 1) {
        AP::logger().WriteStreaming(
            "CICG", "TimeUS,PosOK,PosUpd,VelUpd,Src", "s----", "F----", "QBBBB",
            dal.micros64(), uint8_t(position_correction_valid_used),
            uint8_t(delayer.last_gps_pos_correction_applied),
            uint8_t(delayer.last_gps_vel_correction_applied), position_source_used);
    }

    // @LoggerMessage: CINS
    // @Description: CINS state
    // @Field: TimeUS: Time since system startup
    // @Field: I: instance
    // @Field: Roll: euler roll
    // @Field: Pitch: euler pitch
    // @Field: Yaw: euler yaw
    // @Field: VN: velocity north
    // @Field: VE: velocity east
    // @Field: VD: velocity down
    // @Field: PN: position north
    // @Field: PE: position east
    // @Field: PD: position down
    // @Field: Lat: latitude
    // @Field: Lon: longitude
    // @Field: Alt: altitude AMSL
    ftype roll_rad, pitch_rad, yaw_rad;
    state.XHat.rot().to_euler(&roll_rad, &pitch_rad, &yaw_rad);
    const Location loc = get_location();

    AP::logger().WriteCINSState(dal.micros64(),
                                DAL_CORE(0),
                                degrees(roll_rad),
                                degrees(pitch_rad),
                                wrap_360(degrees(yaw_rad)),
                                state.XHat.vel().x,
                                state.XHat.vel().y,
                                state.XHat.vel().z,
                                state.XHat.pos().x,
                                state.XHat.pos().y,
                                state.XHat.pos().z,
                                loc.lat,
                                loc.lng,
                                loc.alt*0.01);

    // @LoggerMessage: CIN2
    // @Description: Extra CINS states
    // @Field: TimeUS: Time since system startup
    // @Field: I: instance
    // @Field: GX: Gyro Bias X
    // @Field: GY: Gyro Bias Y
    // @Field: GZ: Gyro Bias Z
    // @Field: AX: Accel Bias X
    // @Field: AY: Accel Bias Y
    // @Field: AZ: Accel Bias Z
    // @Field: AVN: auxiliary velocity north
    // @Field: AVE: auxiliary velocity east
    // @Field: AVD: auxiliary velocity down
    // @Field: APN: auxiliary position north
    // @Field: APE: auxiliary position east
    // @Field: APD: auxiliary position down
    AP::logger().WriteCINSExtra(dal.micros64(),
                                DAL_CORE(0),
                                degrees(state.gyr_bias.x),
                                degrees(state.gyr_bias.y),
                                degrees(state.gyr_bias.z),
                                state.acc_bias.x,
                                state.acc_bias.y,
                                state.acc_bias.z,
                                state.ZHat.W1().x,
                                state.ZHat.W1().y,
                                state.ZHat.W1().z,
                                state.ZHat.W2().x,
                                state.ZHat.W2().y,
                                state.ZHat.W2().z);
}

void AP_CINS::write_memory_diagnostic(uint64_t time_us)
{
#if HAL_LOGGING_ENABLED
    const int8_t configured_rate_hz = memory_log_rate.get();
    if (configured_rate_hz <= 0) {
        return;
    }
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t interval_ms = 1000U / uint8_t(configured_rate_hz);
    if (now_ms - last_memory_log_ms < interval_ms) {
        return;
    }
    last_memory_log_ms = now_ms;

    uint32_t free_bytes;
    uint32_t largest_block_bytes;
    hal.util->get_heap_info(free_bytes, largest_block_bytes);
    uint8_t flags = hal.util->get_soft_armed() ? 1U : 0U;
    flags |= state.have_origin ? 2U : 0U;
    flags |= done_yaw_init ? 4U : 0U;
    flags |= delayer.buffer_failed ? 8U : 0U;
    flags |= delayer.history_released_for_compass_calibration ? 16U : 0U;
    flags |= baro_mode() == 1 ? 32U : 0U;
    flags |= baro_datum.frozen_valid() ? 64U : 0U;
    flags |= baro_state.last_sample_accepted ? 128U : 0U;
    AP::logger().WriteCINSMemory(time_us, free_bytes, largest_block_bytes, flags);
#endif
}

void AP_CINS::reset_baro_composite()
{
    reset_baro_history();
    if (!hal.util->get_soft_armed()) {
        reset_baro_source();
    }
}

void AP_CINS::reset_baro_history()
{
    baro_bias.reset();
    if (baro_datum.phase() == CINSBaroDatum::Phase::Tracking) {
        baro_datum.reset();
    } else {
        baro_datum.invalidate();
    }
    baro_state.last_update_ms = 0;
    baro_state.next = 0;
    baro_state.count = 0;
    baro_state.primary = uint8_t(-1);
    baro_state.healthy = false;
    baro_state.last_sample_accepted = false;
}

void AP_CINS::reset_baro_source()
{
    baro_state.source = HeightSource::GpsStartup;
    baro_state.bad_time = 0.0;
    baro_state.transition_offset = 0.0;
    baro_state.selected_down = 0.0;
    baro_state.selected_down_valid = false;
}


CINSBaroDatum::Config AP_CINS::raw_baro_datum_config() const
{
    const auto &barometer = AP::dal().baro();
    return {gains.gps_lag.get(), baro_config.lag.get(), baro_config.max_age.get(), baro_config.gate.get(),
            baro_config.innovation_gate.get(), baro_config.ground_effect_gate.get(),
            baro_config.failure_time.get(), baro_config.transition_rate.get(),
            baro_config.mode.get(), gains.gps_correction_mode.get(), baro_config.ct_gate.get(),
            baro_config.bias_mode.get(), baro_config.source_policy.get(),
            barometer.get_primary(), barometer.get_calibration_generation()};
}

CINSBaroDatum::Config AP_CINS::baro_datum_config() const
{
    if (!last_soft_armed) {
        return raw_baro_datum_config();
    }
    auto config = flight_baro_config;
    // Numeric settings are held for this flight. Sensor identity remains live,
    // so recalibration or a primary switch still invalidates the frozen datum.
    const auto &barometer = AP::dal().baro();
    config.primary = barometer.get_primary();
    config.calibration = barometer.get_calibration_generation();
    return config;
}


bool AP_CINS::baro_datum_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (baro_mode() != 1 || gps_correction_mode() == 1) {
        return true;
    }
    const uint32_t now = AP_HAL::millis();
    const auto &barometer = AP::dal().baro();
    const auto &gps = AP::dal().gps();
    const uint8_t primary = barometer.get_primary();
    float vacc = 0.0f;
    float sacc = 0.0f;
    const bool gps_qualified = gps.primary_sensor() == 0 && gps.status(0) >= AP_DAL_GPS::GPS_OK_FIX_3D &&
        gps.vertical_accuracy(0, vacc) && gps.speed_accuracy(0, sacc) &&
        isfinite(vacc) && vacc > 0.0f && vacc <= 5.0f &&
        isfinite(sacc) && sacc > 0.0f && sacc <= 1.0f;
    const char *reason = "height alignment settling";
    if (!healthy() || !baro_state.healthy || primary >= barometer.num_instances() ||
        !barometer.healthy(primary)) {
        reason = "height alignment unavailable";
    } else if (!gps_qualified || gps.last_message_time_ms(0) != baro_datum.sample_ms()) {
        reason = "qualified GPS/baro pair unavailable";
    } else if (baro_datum.has_offset() && !baro_datum.matches_config(baro_datum_config())) {
        reason = "height calibration/config changed";
    } else if (baro_datum.ready(now)) {
        return true;
    } else {
        switch (baro_datum.reason(now)) {
        case CINSBaroDatum::Reason::Pair: reason = "GPS/barometer alignment settling"; break;
        case CINSBaroDatum::Reason::Prediction: reason = "GPS height state settling"; break;
        case CINSBaroDatum::Reason::Changed: reason = "height calibration/config changed"; break;
        case CINSBaroDatum::Reason::Stale: reason = "height alignment input stale"; break;
        case CINSBaroDatum::Reason::Unavailable: reason = "qualified GPS/baro pair unavailable"; break;
        default: break;
        }
    }
    hal.util->snprintf(failure_msg, failure_msg_len, "CINS: %s", reason);
    return false;
}

bool AP_CINS::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    WITH_SEMAPHORE(datum_sem);
    return baro_datum_check(failure_msg, failure_msg_len);
}

bool AP_CINS::prepare_for_arming(bool required, char *failure_msg, uint8_t failure_msg_len)
{
    WITH_SEMAPHORE(datum_sem);
    flight_config_prepared = false;
    if (baro_mode() != 1 || gps_correction_mode() == 1) {
        flight_baro_config = raw_baro_datum_config();
        flight_config_prepared = true;
        return true;
    }
    if (!baro_datum_check(failure_msg, failure_msg_len)) {
        if (!required) {
            flight_baro_config = raw_baro_datum_config();
            flight_config_prepared = true;
        }
        return !required;
    }
    const bool prepared = baro_datum.prepare(AP_HAL::millis());
    if (prepared || !required) {
        flight_baro_config = raw_baro_datum_config();
        flight_config_prepared = true;
    } else {
        hal.util->snprintf(failure_msg, failure_msg_len, "CINS: height alignment expired");
    }
    write_baro_datum_diagnostic();
    return prepared || !required;
}

void AP_CINS::cancel_arming_preparation()
{
    WITH_SEMAPHORE(datum_sem);
    if (!hal.util->get_soft_armed()) {
        flight_config_prepared = false;
    }
    if (!hal.util->get_soft_armed() && baro_datum.phase() == CINSBaroDatum::Phase::Prepared) {
        baro_datum.reset();
        baro_bias.reset();
        write_baro_datum_diagnostic();
    }
}

bool AP_CINS::arming_reference_prepared() const
{
    WITH_SEMAPHORE(datum_sem);
    return baro_datum.phase() == CINSBaroDatum::Phase::Prepared;
}

void AP_CINS::update_baro_arming()
{
    const bool armed = hal.util->get_soft_armed();
    if (armed == last_soft_armed) {
        return;
    }
    last_soft_armed = armed;
    settings_change_reported = false;
    if (armed) {
        if (!flight_config_prepared) {
            flight_baro_config = raw_baro_datum_config();
        }
        baro_datum.commit(AP::dal().millis());
    } else {
        flight_config_prepared = false;
        baro_datum.reset();
        baro_bias.reset();
        // A queued armed event must not reintroduce barometer aiding after disarm.
        delayer.baro_armed_event = false;
        delayer.aiding_pos = delayer.gps_pos;
        delayer.position_correction_valid = true;
    }
    reset_baro_source();
    write_baro_datum_diagnostic();
}

void AP_CINS::write_baro_datum_diagnostic() const
{
#if HAL_LOGGING_ENABLED
    if (detail_log_rate.get() <= 0 || baro_mode() != 1) {
        return;
    }
    const uint32_t now = AP::dal().millis();
    AP::logger().WriteStreaming("CIBF", "TimeUS,GPSMS,Phase,Offset,PairD,GPSInn,Ready,Reason,Age,Gen",
                               "ss-mmm--s-", "FC-000--C-", "QIBfffBBII",
                               AP::dal().micros64(), baro_datum.sample_ms(), uint8_t(baro_datum.phase()),
                               baro_datum.has_offset() ? baro_datum.offset() : nanf(""),
                               baro_datum.pair_residual(), baro_datum.gps_innovation(),
                               uint8_t(baro_datum.phase() == CINSBaroDatum::Phase::Frozen ?
                                       baro_datum.frozen_valid() : baro_datum.ready(now)),
                               uint8_t(baro_datum.reason(now)), now - baro_datum.sample_ms(),
                               baro_datum.generation());
#endif
}

void AP_CINS::update_baro_history()
{
    if (last_soft_armed && !settings_change_reported) {
        const auto requested = raw_baro_datum_config();
        auto frozen = flight_baro_config;
        frozen.primary = requested.primary;
        frozen.calibration = requested.calibration;
        if (!(frozen == requested)) {
            settings_change_reported = true;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CINS: height settings deferred until disarm");
        }
    }
    if (baro_mode() != 1) {
        reset_baro_composite();
        return;
    }

    const auto &barometer = AP::dal().baro();
    const uint8_t primary = barometer.get_primary();
    if (barometer.num_instances() == 0 || primary >= barometer.num_instances()) {
        baro_state.healthy = false;
        return;
    }

    baro_datum.check_config(baro_datum_config());
    const uint16_t calibration = barometer.get_calibration_generation();
    if (primary != baro_state.primary || calibration != baro_state.calibration_generation) {
        // Never release an armed fallback latch or relearn a frozen datum.
        reset_baro_history();
        baro_state.primary = primary;
        baro_state.calibration_generation = calibration;
    }

    if (!barometer.healthy(primary)) {
        baro_state.healthy = false;
        return;
    }

    const uint32_t timestamp_ms = barometer.get_last_update(primary);
    const float altitude = barometer.get_altitude(primary);
    if (timestamp_ms == 0 || !isfinite(altitude)) {
        baro_state.healthy = false;
        return;
    }
    baro_state.healthy = true;

    if (timestamp_ms == baro_state.last_update_ms) {
        return;
    }
    baro_state.last_update_ms = timestamp_ms;

    baro_state.history[baro_state.next] = { timestamp_ms, -altitude };
    baro_state.next = (baro_state.next + 1U) % CINS_BARO_HISTORY_SIZE;
    if (baro_state.count < CINS_BARO_HISTORY_SIZE) {
        baro_state.count++;
    }
}

AP_CINS::BaroCandidateResult AP_CINS::get_baro_candidate(
    const Vector3F &gps_pos,
    uint32_t gps_timestamp_ms,
    ftype &baro_down)
{
    // One diagnostic per new GPS sample, including failed alignment attempts.
    // NaN means unavailable; CIBF records tracking/prepared/frozen status.
    const float gps_down = gps_pos.z;
    float raw_down = nanf("");
    float aligned_down = nanf("");
    uint32_t timing_error_ms = UINT32_MAX;
    baro_down = nanf("");

    const auto finish = [&](BaroCandidateResult result) {
#if HAL_LOGGING_ENABLED
        if (detail_log_rate.get() > 0) {
            const float offset = baro_datum.has_offset() ? baro_datum.offset() : nanf("");
            AP::logger().WriteStreaming("CIBD", "TimeUS,GPSMS,GPSD,RawD,BarD,Offset,TErr,Count,Result",
                                       "ssmmmm---", "FC0000---", "QIffffIBB",
                                       AP::dal().micros64(), gps_timestamp_ms,
                                       gps_down, raw_down, aligned_down, offset,
                                       timing_error_ms, baro_datum.count(), uint8_t(result));
        }
#else
        (void)gps_down;
        (void)raw_down;
        (void)aligned_down;
        (void)timing_error_ms;
#endif
        return result;
    };

    if (baro_mode() != 1) {
        return finish(BaroCandidateResult::Disabled);
    }
    if (!baro_state.healthy) {
        return finish(BaroCandidateResult::Unhealthy);
    }
    if (baro_state.count == 0) {
        return finish(BaroCandidateResult::NoHistory);
    }

    const float gps_lag = get_gps_lag();
    const float baro_lag = baro_datum_config().baro_lag;
    const float max_age = baro_datum_config().max_age;
    const float gate = baro_datum_config().consistency_gate;
    const float innovation_gate = baro_datum_config().innovation_gate;
    const float ground_effect_gate = baro_datum_config().ground_effect_gate;
    const float failure_time = baro_datum_config().failure_time;
    const float transition_rate = baro_datum_config().transition_rate;
    if (!isfinite(gps_lag) || !isfinite(baro_lag) || !isfinite(max_age) ||
        !isfinite(gate) || !isfinite(innovation_gate) ||
        !isfinite(ground_effect_gate) || !isfinite(failure_time) ||
        !isfinite(transition_rate) || gps_lag < 0.0f || gps_lag > 0.2f ||
        baro_lag < 0.0f || baro_lag > 0.2f ||
        max_age < 0.005f || max_age > 0.1f ||
        gate < 0.1f || gate > 50.0f ||
        innovation_gate < 0.1f || innovation_gate > 10.0f ||
        ground_effect_gate < 0.1f || ground_effect_gate > 5.0f ||
        failure_time < 0.5f || failure_time > 10.0f ||
        transition_rate < 0.1f || transition_rate > 5.0f) {
        return finish(BaroCandidateResult::Configuration);
    }

    // GPS and barometer receipt timestamps share the AP_HAL millisecond clock.
    // Select the barometer receipt time corresponding to the GPS measurement
    // time: t_baro = t_gps - gps_lag + baro_lag.
    const int32_t lag_difference_ms = int32_t((baro_lag - gps_lag) * 1000.0f);
    const uint32_t target_timestamp_ms = gps_timestamp_ms + lag_difference_ms;

    uint8_t best_index = 0;
    uint32_t best_error_ms = UINT32_MAX;
    for (uint8_t i = 0; i < baro_state.count; i++) {
        const uint32_t sample_time_ms = baro_state.history[i].timestamp_ms;
        const uint32_t forward_error = sample_time_ms - target_timestamp_ms;
        const uint32_t backward_error = target_timestamp_ms - sample_time_ms;
        const uint32_t error_ms = MIN(forward_error, backward_error);
        if (error_ms < best_error_ms) {
            best_error_ms = error_ms;
            best_index = i;
        }
    }

    timing_error_ms = best_error_ms;
    raw_down = baro_state.history[best_index].down;
    if (baro_datum.has_offset()) {
        aligned_down = raw_down - baro_datum.offset() -
            (baro_datum_config().bias_mode == 1 ? baro_bias.bias() : 0.0f);
    }
    const uint32_t max_age_ms = uint32_t(max_age * 1000.0f + 0.5f);
    if (best_error_ms > max_age_ms) {
        return finish(BaroCandidateResult::Timing);
    }

    const ftype raw_baro_down = baro_state.history[best_index].down;
    if (!isfinite(raw_baro_down) || gps_pos.is_nan() || gps_pos.is_inf()) {
        return finish(BaroCandidateResult::Nonfinite);
    }

    const auto &gps = AP::dal().gps();
    float vacc = 0.0f;
    float sacc = 0.0f;
    const bool qualified = gps.primary_sensor() == 0 && gps.status(0) >= AP_DAL_GPS::GPS_OK_FIX_3D &&
        gps.vertical_accuracy(0, vacc) && gps.speed_accuracy(0, sacc) &&
        isfinite(vacc) && vacc > 0.0f && vacc <= 5.0f &&
        isfinite(sacc) && sacc > 0.0f && sacc <= 1.0f;
    baro_datum.pair(gps_timestamp_ms, float(raw_baro_down), gps_down,
                    baro_datum_config(), qualified);
    if (!baro_datum.has_offset()) {
        return finish(BaroCandidateResult::Aligning);
    }
    if (hal.util->get_soft_armed() && !baro_datum.frozen_valid()) {
        return finish(BaroCandidateResult::DatumUnavailable);
    }

    const ftype composite_down = raw_baro_down - baro_datum.offset() -
        (baro_datum_config().bias_mode == 1 ? baro_bias.bias() : 0.0f);
    aligned_down = composite_down;
    if (!isfinite(composite_down)) {
        return finish(BaroCandidateResult::Nonfinite);
    }

    // A GPS/barometer disagreement is now a source-consistency signal rather
    // than an instruction to switch immediately to GPS height. Keeping the
    // finite barometer candidate allows GPMOD=0 to preserve its continuous
    // measurement structure and GPMOD=2 to make its own delayed-innovation test.
    baro_down = composite_down;
    if (fabsF(composite_down - gps_pos.z) > gate) {
        return finish(BaroCandidateResult::Consistency);
    }

    return finish(BaroCandidateResult::Accepted);
}

void AP_CINS::update_baro_bias()
{
    if (baro_datum_config().bias_mode != 1) {
        return;
    }
    const CINSBaroBias::Config cfg {
        baro_config.bias_tc.get(), baro_config.bias_rate.get(), baro_config.bias_limit.get(),
        baro_config.bias_jump.get(), baro_config.bias_dwell.get()
    };
    const float before = baro_bias.bias();
    // Reconstruct the datum-aligned observation using the bias applied when
    // the candidate was formed. The initial alignment remains frozen.
    const float datum_down = delayer.baro_down + before;
    const bool usable = baro_mode() == 1 && gps_correction_mode() != 1 &&
        baro_datum.frozen_valid() && baro_state.source == HeightSource::Baro &&
        delayer.baro_result == BaroCandidateResult::Accepted &&
        baro_state.last_sample_accepted && delayer.position_correction_valid &&
        delayer.last_gps_pos_correction_applied;
    const auto result = baro_bias.update(cfg, delayer.bias_timestamp_ms, datum_down,
                                       delayer.gps_pos.z, delayer.gps_vel.z, delayer.bias_trust, usable);
#if HAL_LOGGING_ENABLED
    if (detail_log_rate.get() > 0) {
        AP::logger().WriteStreaming("CIBB", "TimeUS,GPSMS,Datum,Bias0,Bias1,Resid,Trust,Result",
                                   "ssmmmm--", "FC00000-", "QIfffffB",
                                   AP::dal().micros64(), delayer.bias_timestamp_ms,
                                   datum_down, before, baro_bias.bias(),
                                   float(delayer.baro_down - delayer.gps_pos.z),
                                   delayer.bias_trust, uint8_t(result));
    }
#else
    (void)result;
#endif
}

void AP_CINS::write_baro_source_diagnostic(
    const ftype gps_down,
    const ftype baro_down,
    const ftype innovation_down,
    const ftype used_down,
    const ftype bad_time,
    const bool ground_effect_expected,
    const bool position_valid) const
{
#if HAL_LOGGING_ENABLED
    if (detail_log_rate.get() <= 0) {
        return;
    }

    const ftype bg_diff =
        (isfinite(baro_down) && isfinite(gps_down)) ? baro_down - gps_down : nanf("");
    AP::logger().WriteStreaming(
        "CIBS",
        "TimeUS,GPSD,BarD,InnovD,UsedD,BGDiff,BadT,GE,Src,PosOK",
        "smmmmms---", "F000000---", "QffffffBBB",
        AP::dal().micros64(),
        float(gps_down), float(baro_down), float(innovation_down),
        float(used_down), float(bg_diff), float(bad_time),
        uint8_t(ground_effect_expected), uint8_t(baro_state.source),
        uint8_t(position_valid));
#else
    (void)gps_down;
    (void)baro_down;
    (void)innovation_down;
    (void)used_down;
    (void)bad_time;
    (void)ground_effect_expected;
    (void)position_valid;
#endif
}

void AP_CINS::write_baro_innovation_diagnostic(
    const ftype predicted_down,
    const ftype baro_innovation_down,
    const ftype gps_innovation_down,
    const ftype current_lag) const
{
#if HAL_LOGGING_ENABLED
    if (detail_log_rate.get() <= 0) {
        return;
    }

    const GL2 &AZ = state.ZHat.A();
    AP::logger().WriteStreaming(
        "CIBX",
        "TimeUS,PredD,BarInn,GPSInn,Lag,A11,A12,A21,A22",
        "smmms----", "F0000----", "Qffffffff",
        AP::dal().micros64(),
        float(predicted_down), float(baro_innovation_down),
        float(gps_innovation_down), float(current_lag),
        float(AZ.a11()), float(AZ.a12()), float(AZ.a21()), float(AZ.a22()));
#else
    (void)predicted_down;
    (void)baro_innovation_down;
    (void)gps_innovation_down;
    (void)current_lag;
#endif
}

// Slew only the datum offset, so GPS motion is followed during the transition.
void AP_CINS::update_gps_height_transition(const ftype gps_dt)
{
    const ftype max_step =
        is_positive(gps_dt) ? baro_datum_config().transition_rate * gps_dt : 0.0;
    if (fabsF(baro_state.transition_offset) <= max_step) {
        baro_state.transition_offset = 0.0;
        baro_state.source = HeightSource::Gps;
    } else if (is_positive(max_step)) {
        baro_state.transition_offset +=
            (baro_state.transition_offset > 0.0 ? -max_step : max_step);
    }
    baro_state.selected_down = delayer.gps_pos.z + baro_state.transition_offset;
}

void AP_CINS::update_baro_source_ct(const Vector3F &gps_pos, const ftype gps_dt)
{
    delayer.aiding_pos = gps_pos;
    delayer.position_correction_valid = true;
    baro_state.last_sample_accepted = false;

    const bool finite_baro_candidate =
        delayer.baro_result == BaroCandidateResult::Accepted ||
        delayer.baro_result == BaroCandidateResult::Consistency;

    // Use GPS until the first aligned barometer candidate is available.
    // Normal datum alignment is not an availability fault.
    if (baro_state.source == HeightSource::GpsStartup) {
        if (finite_baro_candidate) {
            baro_state.source = HeightSource::Baro;
            baro_state.selected_down_valid = true;
        } else {
            baro_state.selected_down = gps_pos.z;
            const bool aligning = delayer.baro_result == BaroCandidateResult::Aligning ||
                                  delayer.baro_result == BaroCandidateResult::NoHistory;
            if (aligning) {
                baro_state.bad_time = 0.0;
            } else if (is_positive(gps_dt)) {
                baro_state.bad_time += gps_dt;
            }
            if (baro_state.bad_time >= baro_datum_config().failure_time) {
                baro_state.source = HeightSource::Gps;
            }
            write_baro_source_diagnostic(gps_pos.z, delayer.baro_down, nanf(""),
                                         gps_pos.z, baro_state.bad_time,
                                         delayer.ground_effect_expected, true);
            return;
        }
    }

    if (baro_state.source == HeightSource::Baro) {
        if (finite_baro_candidate) {
            // CT deliberately continues to use a finite but inconsistent baro
            // sample. This preserves the original continuous measurement
            // structure; the inconsistency only drives the persistent-fault timer.
            delayer.aiding_pos.z = delayer.baro_down;
            baro_state.selected_down = delayer.baro_down;
            baro_state.last_sample_accepted = true;
        } else {
            // If no finite aligned barometer value exists at this GPS event,
            // hold the previous vertical measurement rather than jump to GPS.
            delayer.aiding_pos.z = baro_state.selected_down;
        }

        // A finite barometer sample disturbed during expected ground effect is
        // intentionally still used by ungated CT, but it must not be interpreted
        // as evidence of a persistent sensor failure.
        const bool expected_ground_effect_candidate =
            delayer.ground_effect_expected && finite_baro_candidate;
        if (delayer.baro_result == BaroCandidateResult::Accepted ||
            expected_ground_effect_candidate) {
            baro_state.bad_time = 0.0;
        } else if (is_positive(gps_dt)) {
            baro_state.bad_time += gps_dt;
        }

        if (baro_state.bad_time >= baro_datum_config().failure_time) {
            baro_state.source = HeightSource::GpsTransition;
            baro_state.transition_offset = baro_state.selected_down - gps_pos.z;
            // The first fallback sample is exactly continuous with the outgoing
            // barometric/held height. Start slewing on the next GPS event.
            delayer.aiding_pos.z = gps_pos.z + baro_state.transition_offset;
            baro_state.selected_down = delayer.aiding_pos.z;
            baro_state.last_sample_accepted = false;
            write_baro_source_diagnostic(gps_pos.z, delayer.baro_down, nanf(""),
                                         delayer.aiding_pos.z, baro_state.bad_time,
                                         delayer.ground_effect_expected, true);
            return;
        }
    }

    if (baro_state.source == HeightSource::GpsTransition) {
        update_gps_height_transition(gps_dt);
        delayer.aiding_pos.z = baro_state.selected_down;
        baro_state.last_sample_accepted = false;
    } else if (baro_state.source == HeightSource::Gps) {
        delayer.aiding_pos.z = gps_pos.z;
        baro_state.selected_down = delayer.aiding_pos.z;
        baro_state.last_sample_accepted = false;
    }

    write_baro_source_diagnostic(gps_pos.z, delayer.baro_down, nanf(""),
                                 delayer.aiding_pos.z, baro_state.bad_time,
                                 delayer.ground_effect_expected, true);
}

/*
  update on new GPS sample
 */
void AP_CINS::update_gps(
    const Vector3F &gps_pos,
    const Vector3F &vel,
    const ftype gps_dt,
    BaroCandidateResult baro_result,
    const ftype baro_down,
    const bool ground_effect_expected)
{
    // Update the delayer variables. Sensor-source management is performed at
    // the GPS event, while the selected vector is consumed by the continuous
    // or finite correction path in update_imu().
    if (!AP_CINS_Delay::rebuild(delayer.stamped_inputs, delayer.history_increment, delayer.YR)) {
        delayer.buffer_failed = true;
        delayer.gps_correction_pending = false;
        return;
    }
    // Snapshot quality with this packet; do not read a newer GPS at consumption.
    delayer.bias_trust = 0.0f;
    delayer.bias_timestamp_ms = AP::dal().gps().last_message_time_ms(0);
    if (baro_datum_config().bias_mode == 1) {
        const auto &gps = AP::dal().gps();
        float vacc = 0.0f;
        float sacc = 0.0f;
        const float vmax = baro_config.bias_vacc.get();
        const float smax = baro_config.bias_sacc.get();
        if (gps.primary_sensor() == 0 &&
            gps.status(0) >= AP_DAL_GPS::GPS_OK_FIX_3D && gps.num_sats(0) >= 6 &&
            gps.have_vertical_velocity(0) && gps.vertical_accuracy(0, vacc) && gps.speed_accuracy(0, sacc) &&
            isfinite(vacc) && vacc > 0.0f && isfinite(sacc) && sacc > 0.0f &&
            isfinite(vmax) && vmax >= 0.1f && vmax <= 10.0f &&
            isfinite(smax) && smax >= 0.1f && smax <= 2.0f && vacc < vmax && sacc < smax) {
            delayer.bias_trust = (1.0f - vacc / vmax) * (1.0f - sacc / smax);
        }
    }
    delayer.gps_vel = vel;
    delayer.gps_pos = gps_pos;
    delayer.aiding_pos = gps_pos;
    delayer.baro_down = baro_down;
    delayer.baro_result = baro_result;
    delayer.baro_armed_event = hal.util->get_soft_armed();
    delayer.datum_generation = baro_datum.generation();
    delayer.ground_effect_expected = ground_effect_expected;
    delayer.position_correction_valid = true;

    // A GPS fallback is latched only for an armed flight. While disarmed, keep
    // the barometer history/datum but reset the source-manager state.
    if (!hal.util->get_soft_armed()) {
        reset_baro_source();
    }

    const int8_t gps_mode = gps_correction_mode();
    if (baro_mode() != 1 || gps_mode == 1 || !delayer.baro_armed_event) {
        // Disarmed and prepared CINS remains GPS-aided; alignment is observational.
        baro_state.last_sample_accepted = false;
        delayer.position_correction_valid = true;
    } else if (gps_mode == 0 && baro_datum_config().ct_gate != 1) {
        update_baro_source_ct(gps_pos, gps_dt);
    } else if (gps_mode == 2 || gps_mode == 0) {
        // Gated CT and finite mode finalise acceptance using the delayed predicted
        // position measurement in update_imu().
        delayer.position_correction_valid = false;
    } else {
        // Unknown correction modes do not enable the experimental BAMOD logic.
        baro_state.last_sample_accepted = false;
        delayer.position_correction_valid = true;
    }

    if (is_positive(gps_dt)) {
        delayer.gps_correction_dt = gps_dt;
        delayer.gps_correction_pending = true;
    }
}


/*
  update from IMU data
 */
void AP_CINS::update_imu(const Vector3F &gyro_rads, const Vector3F &accel_mss, const ftype dt)
{
#if HAL_LOGGING_ENABLED
    static EstimatorPhaseRuntimeAccumulator predict_runtime;
    static EstimatorPhaseRuntimeAccumulator delay_runtime;
    static EstimatorPhaseRuntimeAccumulator posvel_runtime;
    uint64_t phase_start_us = estimator_runtime_micros64();
#endif

	const int8_t gps_mode = gps_correction_mode();
	const bool gps_event_mode = gps_mode >= 1;
	const bool gps_finite_mode = gps_mode == 2;

    //Integrate Dynamics using the Matrix exponential

    // Update the bias gain matrices
    // In mathematical terms, \dot{M} = B = - Ad_{Z^{-1} \hat{X}} [I_3, 0_3; 0_3, I_3; 0_3, 0_3].
    // Here, we split B into its parts and add them to the parts of the bias gain matrix.
    // Propagate the IMU-bias sensitivity matrices in every correction mode.
    // GPMOD=2 uses the same high-rate bias model as the continuous/event-Euler
    // observer; only the aiding measurement jump is discretised differently.
	if (done_yaw_init) {
		const SIM23 XInv_Z = SIM23(state.XHat.inverse()) * state.ZHat;

		state.gyr_bias_gain_mat.rot += -state.XHat.rot() * dt;
		state.gyr_bias_gain_mat.vel +=
			XInv_Z.R().transposed() *
			Matrix3F::skew_symmetric(XInv_Z.W1()) * dt;
		state.gyr_bias_gain_mat.pos +=
			XInv_Z.R().transposed() *
			Matrix3F::skew_symmetric(XInv_Z.W2()) * dt;

		// state.acc_bias_gain_mat.rot is unchanged
		state.acc_bias_gain_mat.vel +=
			-state.XHat.rot() * state.ZHat.A().a11() * dt;
		state.acc_bias_gain_mat.pos +=
			-state.XHat.rot() * state.ZHat.A().a12() * dt;
	}

    const Gal3F leftMat =
    Gal3F::exponential(
        heapVars.zero_vector,
        heapVars.zero_vector,
        heapVars.gravity_vector * dt,
        -dt);

	// All modes propagate XHat using the estimated IMU biases. In GPMOD=2 the
	// bias states are updated by the finite-event aiding maps below.
	const Vector3F gyro_input = gyro_rads - state.gyr_bias;
	const Vector3F accel_input = accel_mss - state.acc_bias;

	const Gal3F rightMat =
		Gal3F::exponential(
			gyro_input * dt,
			heapVars.zero_vector,
			accel_input * dt,
			dt);

	// Update XHat (Observer Dynamics)
	state.XHat = leftMat * state.XHat * rightMat;

    // Update ZHat (Auxiliary Dynamics)
	// const bool gps_finite_mode = gps_correction_mode() == 2;

	if (gps_finite_mode) {
		// Finite discrete construction: propagate Z continuously, but do not
		// apply the continuous Kq correction. The exact finite Kq map (and its
		// matching bias-sensitivity coordinate transform) is applied at the GPS event.
		state.ZHat = SIM23(leftMat) * state.ZHat;
	} else {
		// Existing continuous Kq implementation
		heapVars.GammaInv = SIM23::identity();

		const GL2 S_Gamma =
			0.5 * state.ZHat.A().transposed()
			* GL2(gains.Q11.get(), 0., 0., gains.Q22.get())
			* state.ZHat.A();

		heapVars.GammaInv.A() = GL2::identity() - dt * S_Gamma;

		// Existing bias gain transformation
		compute_bias_update_imu(heapVars.GammaInv.inverse());

		state.ZHat =
			SIM23(leftMat) * state.ZHat * heapVars.GammaInv;
	}

#if HAL_LOGGING_ENABLED
    predict_runtime.log_sample(ESTIMATOR_RUNTIME_CINS_ID,
                               ESTIMATOR_RUNTIME_PHASE_PREDICT,
                               uint32_t(estimator_runtime_micros64() - phase_start_us));
    phase_start_us = estimator_runtime_micros64();
#endif

    // Update the delay matrices and buffer
    delayer.YR = delayer.YR * rightMat;
    delayer.internal_time += dt;
    const uint32_t internal_time_us = uint32_t(uint64_t(delayer.internal_time * 1.0e6));
    const uint32_t gps_lag_us = uint32_t(get_gps_lag() * 1.0e6f);
    delayer.history_increment = delayer.history_increment * rightMat;
    if (++delayer.history_increment_count >= CINS_HISTORY_DECIMATION) {
        stamped_Gal3F stamped_input {};
        stamped_input.timestamp_us = internal_time_us;
        stamped_input.rot.from_rotation_matrix(delayer.history_increment.rot());
        stamped_input.pos = delayer.history_increment.pos();
        stamped_input.vel = delayer.history_increment.vel();
        stamped_input.tau = delayer.history_increment.tau();
        if (!delayer.stamped_inputs.push(stamped_input)) {
            delayer.buffer_failed = true;
        }
        delayer.history_increment = Gal3F::identity();
        delayer.history_increment_count = 0;
    }

    stamped_Gal3F back;
    while (delayer.stamped_inputs.peek(back) &&
           uint32_t(internal_time_us - back.timestamp_us) > gps_lag_us) {
        delayer.stamped_inputs.pop();
    }

#if HAL_LOGGING_ENABLED
    delay_runtime.log_sample(ESTIMATOR_RUNTIME_CINS_ID,
                             ESTIMATOR_RUNTIME_PHASE_DELAY,
                             uint32_t(estimator_runtime_micros64() - phase_start_us));
    phase_start_us = estimator_runtime_micros64();
#endif

    // const int8_t gps_mode = gps_correction_mode();
	// const bool gps_event_mode = gps_mode >= 1;
	// const bool gps_finite_mode = gps_mode == 2;

	if (gps_event_mode && !delayer.gps_correction_pending) {
		return;
	}

	const ftype correction_dt =
		gps_event_mode ? delayer.gps_correction_dt : dt;

    const bool new_gps_event = delayer.gps_correction_pending;
	delayer.gps_correction_pending = false;

    // Update using delayed GPS
	const ftype& current_lag = delayer.YR.tau();

	const Vector3F& left_delay_vel =
		-heapVars.gravity_vector * current_lag;

	const Vector3F& left_delay_pos =
		-heapVars.gravity_vector * 0.5 * current_lag * current_lag;

	const Gal3& YRInv = delayer.YR.inverse();


	// ---------------------------------------------------------
	// Construct delayed GPS VELOCITY measurement tuple
	// ---------------------------------------------------------

	const Vector2F ref_vel(1., 0.);
	const Vector3F mu0_vel = YRInv.vel();

	const Vector3F mu_vel =
		delayer.gps_vel
		- left_delay_vel * ref_vel.x
		- left_delay_pos * ref_vel.y;


	// ---------------------------------------------------------
	// Construct delayed POSITION measurement tuple
	// ---------------------------------------------------------

	const Vector3F mu0_pos = YRInv.pos();
	const Vector2F ref_pos(-current_lag, 1.);
	const Vector3F delay_pos_term =
		left_delay_vel * ref_pos.x +
		left_delay_pos * ref_pos.y;

    // Build delayed vertical residual diagnostics once per BAMOD GPS event, even
    // when ungated CT is in use. This keeps the diagnostics observational only
    // and avoids adding new per-IMU-cycle work to the CT path.
    bool finite_baro_candidate = false;
    Vector3F mu_hat_pos;
    ftype predicted_raw_down = nanf("");
    ftype baro_innovation_down = nanf("");
    if (delayer.baro_armed_event && hal.util->get_soft_armed() &&
        delayer.datum_generation != baro_datum.generation()) {
        // An armed datum change is an unavailable observation, not an
        // instruction to bypass persistence and jump immediately to raw GPS.
        delayer.baro_result = BaroCandidateResult::DatumUnavailable;
        delayer.baro_down = nanf("");
    }
    if (baro_mode() == 1 && new_gps_event) {
        finite_baro_candidate =
            delayer.baro_result == BaroCandidateResult::Accepted ||
            delayer.baro_result == BaroCandidateResult::Consistency;
        mu_hat_pos =
            state.XHat.rot() * mu0_pos +
            state.XHat.vel() * ref_pos.x +
            state.XHat.pos() * ref_pos.y;
        predicted_raw_down = mu_hat_pos.z + delay_pos_term.z;
        baro_innovation_down =
            finite_baro_candidate ? delayer.baro_down - predicted_raw_down : nanf("");
        const ftype gps_innovation_down =
            delayer.gps_pos.z - predicted_raw_down;
        if (!delayer.baro_armed_event) {
            baro_datum.prediction(delayer.bias_timestamp_ms, float(gps_innovation_down));
        }
        write_baro_datum_diagnostic();
        write_baro_innovation_diagnostic(
            predicted_raw_down, baro_innovation_down, gps_innovation_down, current_lag);
    }

    // Select gated aiding once per GPS event. CT holds the acceptance decision
    // between events, so an initially rejected sample cannot be reintroduced
    // by a changing prediction at a later IMU step.
    const bool armed_baro_event = delayer.baro_armed_event && hal.util->get_soft_armed();
    const bool gated_baro = baro_mode() == 1 && armed_baro_event &&
        (gps_finite_mode || (gps_mode == 0 && baro_datum_config().ct_gate == 1));
    if (baro_mode() == 1 && new_gps_event && !armed_baro_event) {
        delayer.aiding_pos = delayer.gps_pos;
        delayer.position_correction_valid = true;
        write_baro_source_diagnostic(delayer.gps_pos.z, delayer.baro_down,
                                     baro_innovation_down, delayer.gps_pos.z, 0.0f,
                                     delayer.ground_effect_expected, true);
    }
    if (gated_baro && new_gps_event) {
        const auto admission = CINSBaroAdmission::evaluate(
            finite_baro_candidate, baro_innovation_down, delayer.ground_effect_expected,
            baro_datum_config().innovation_gate, baro_datum_config().ground_effect_gate);
#if HAL_LOGGING_ENABLED
        if (detail_log_rate.get() > 0) {
            AP::logger().WriteStreaming(
                "CIBA", "TimeUS,GPSMS,Finite,InnovBad,GEBad,Admit", "ss----", "FC----", "QIBBBB",
                AP::dal().micros64(), delayer.bias_timestamp_ms,
                uint8_t(finite_baro_candidate), uint8_t(admission.innovation_bad),
                uint8_t(admission.ground_effect_bad), uint8_t(admission.accepted));
        }
#endif
        if (gps_finite_mode && baro_datum_config().source_policy == 1) {
            // Fixed source retains packet admission without persistent-fault fallback.
            baro_state.source = HeightSource::Baro;
            baro_state.bad_time = 0.0;
            delayer.position_correction_valid = admission.accepted;
            baro_state.last_sample_accepted = admission.accepted;
            if (admission.accepted) {
                delayer.aiding_pos = delayer.gps_pos;
                delayer.aiding_pos.z = delayer.baro_down;
                baro_state.selected_down = delayer.baro_down;
                baro_state.selected_down_valid = true;
            } else {
                baro_state.selected_down_valid = false;
            }
            write_baro_source_diagnostic(
                delayer.gps_pos.z, delayer.baro_down, baro_innovation_down,
                admission.accepted ? delayer.baro_down : nanf(""),
                0.0f, delayer.ground_effect_expected, admission.accepted);
        } else {
            const ftype source_dt = delayer.gps_correction_dt;

            ftype innovation_down = baro_innovation_down;
            bool transition_started = false;
            delayer.position_correction_valid = false;
            baro_state.last_sample_accepted = false;

            const bool startup = baro_state.source == HeightSource::GpsStartup;
            if (startup || baro_state.source == HeightSource::Baro) {
                if (startup) {
                    delayer.aiding_pos = delayer.gps_pos;
                    delayer.position_correction_valid = true;
                }
                bool reject_position = !finite_baro_candidate;
                bool ground_effect_rejection = false;
                if (finite_baro_candidate) {
                    Vector3F baro_pos = delayer.gps_pos;
                    baro_pos.z = delayer.baro_down;

                    ground_effect_rejection = admission.ground_effect_bad;
                    reject_position = !admission.accepted;

                    if (!reject_position) {
                        baro_state.source = HeightSource::Baro;
                        delayer.aiding_pos = baro_pos;
                        delayer.position_correction_valid = true;
                        baro_state.selected_down = delayer.baro_down;
                        baro_state.selected_down_valid = true;
                        baro_state.last_sample_accepted = true;
                    }
                }

                // BAGAT disagreement is retained as a persistent source-consistency
                // signal even when the delayed CINS innovation remains admissible.
                const bool aligning = startup &&
                    (delayer.baro_result == BaroCandidateResult::Aligning ||
                     delayer.baro_result == BaroCandidateResult::NoHistory);
                // Expected ground-effect innovations of either sign are an admissibility reason
                // to reject this position event, not evidence of a persistent sensor fault.
                // Reset BAFLT persistence so repeated landing ground-effect rejections can
                // never accumulate into a GPS fallback. Other unavailable/non-GE failures
                // retain the existing persistent-fault behaviour.
                const bool fault_signal = !aligning && !ground_effect_rejection &&
                    (reject_position ||
                     delayer.baro_result == BaroCandidateResult::Consistency);
                if (ground_effect_rejection) {
                    baro_state.bad_time = 0.0;
                } else if (fault_signal) {
                    baro_state.bad_time += source_dt;
                } else {
                    baro_state.bad_time = 0.0;
                }

                if (baro_state.bad_time >= baro_datum_config().failure_time) {
                    if (baro_state.source == HeightSource::GpsStartup) {
                        // Already aiding with GPS: latch it without an artificial offset.
                        baro_state.source = HeightSource::Gps;
                    } else {
                        baro_state.source = HeightSource::GpsTransition;
                        baro_state.transition_offset =
                            predicted_raw_down - delayer.gps_pos.z;
                        delayer.aiding_pos = delayer.gps_pos;
                        delayer.aiding_pos.z += baro_state.transition_offset;
                        delayer.position_correction_valid = true;
                        baro_state.selected_down = delayer.aiding_pos.z;
                        baro_state.last_sample_accepted = false;
                        transition_started = true;
                    }
                }
            }

            if (baro_state.source == HeightSource::GpsTransition && !transition_started) {
                update_gps_height_transition(source_dt);
                delayer.aiding_pos = delayer.gps_pos;
                delayer.aiding_pos.z = baro_state.selected_down;
                delayer.position_correction_valid = true;
                baro_state.last_sample_accepted = false;
            } else if (baro_state.source == HeightSource::Gps) {
                delayer.aiding_pos = delayer.gps_pos;
                delayer.position_correction_valid = true;
                baro_state.selected_down = delayer.aiding_pos.z;
                baro_state.last_sample_accepted = false;
            }

            write_baro_source_diagnostic(
                delayer.gps_pos.z, delayer.baro_down, innovation_down,
                delayer.position_correction_valid ? delayer.aiding_pos.z : nanf(""),
                baro_state.bad_time, delayer.ground_effect_expected,
                delayer.position_correction_valid);
        }
    }

	const Vector3F mu_pos =
		delayer.aiding_pos - delay_pos_term;


	// These are filled by whichever correction mode is active
	ftype velErrorSq = 0.0;
	ftype posErrorSq = 0.0;


	// ---------------------------------------------------------
	// Apply correction
	// ---------------------------------------------------------

	if (gps_finite_mode) {

		/*
		* Finite discrete correction.
		*
		* IMPORTANT:
		* Position is applied first to match the proof.
		* The velocity update is then called using the already
		* updated XHat and ZHat, so all of its state-dependent
		* quantities a, b, c, omega, etc. are recomputed.
		*/

		if (delayer.position_correction_valid) {
			posErrorSq =
				update_vector_measurement_discrete(
					mu_pos,
					mu0_pos,
					ref_pos,
					gains.gpspos_att.get(),
					gains.gps_pos.get(),
					gains.gps_pos_gyr_bias.get(),
					gains.gps_pos_acc_bias.get(),
					correction_dt,
					0);
			delayer.last_gps_pos_correction_applied = true;
		}


		velErrorSq =
			update_vector_measurement_discrete(
				mu_vel,
				mu0_vel,
				ref_vel,
				gains.gpsvel_att.get(),
				gains.gps_vel.get(),
				gains.gps_vel_gyr_bias.get(),
				gains.gps_vel_acc_bias.get(),
				correction_dt,
				1);
		delayer.last_gps_vel_correction_applied = true;


		// Common finite Kq correction, once after both sensors
		update_Kq_discrete(correction_dt);

		#if HAL_LOGGING_ENABLED
			if (detail_log_rate.get() > 0) {
				const GL2 &AZ = state.ZHat.A();

				AP::logger().WriteStreaming(
					"CIDA",
					"TimeUS,Hq,A11,A12,A21,A22",
					"ss----",
					"F0----",
					"Qfffff",
					AP::dal().micros64(),
					float(correction_dt),
					float(AZ.a11()),
					float(AZ.a12()),
					float(AZ.a21()),
					float(AZ.a22()));
			}
		#endif

	} else {

		/*
		* Existing continuous / event-Euler implementation.
		* Leave this exactly as it was.
		*/

		velErrorSq =
			update_vector_measurement_cts(
				mu_vel,
				mu0_vel,
				ref_vel,
				gains.gpsvel_att.get(),
				gains.gps_vel.get(),
				gains.gps_vel_gyr_bias.get(),
				gains.gps_vel_acc_bias.get(),
				correction_dt);


        // This call contains the complete position state, auxiliary and bias
        // corrections. Skipping it preserves the measurement module boundary.
        if (delayer.position_correction_valid) {
            posErrorSq =
                update_vector_measurement_cts(
                    mu_pos,
                    mu0_pos,
                    ref_pos,
                    gains.gpspos_att.get(),
                    gains.gps_pos.get(),
                    gains.gps_pos_gyr_bias.get(),
                    gains.gps_pos_acc_bias.get(),
                    correction_dt);
            delayer.last_gps_pos_correction_applied = true;
        }
        delayer.last_gps_vel_correction_applied = true;
	}


	// ---------------------------------------------------------
	// Existing health / variance calculations - KEEP
	// ---------------------------------------------------------

	state.variances.velTest =
		velErrorSq /
		(state.variances.velVar * gains.variance_scale.get());

	state.variances.velVar =
		gains.variance_lowpass.get() * velErrorSq
		+ (1. - gains.variance_lowpass.get())
			* state.variances.velVar;


	if (delayer.last_gps_pos_correction_applied) {
		state.variances.posTest =
			posErrorSq /
			(state.variances.posVar * gains.variance_scale.get());

		state.variances.posVar =
			gains.variance_lowpass.get() * posErrorSq
			+ (1. - gains.variance_lowpass.get())
				* state.variances.posVar;
	}


	// The CINS algorithm does not have an internal variance state like an EKF.
	// Instead, we compute the variance by applying a low-pass to the squared
	// measurement errors. If a new squared measurement error arrives that is
	// much larger than this low-passed value, it indicates something is wrong.
	// This only works after initialising for a number of steps first.
	if (state.variances.steps_to_initialise > 0) {
		--state.variances.steps_to_initialise;
	}

    // Adapt only after this event's acceptance and observer correction. Held
    // CT measurements do not repeatedly update the bias.
    if (new_gps_event) {
        update_baro_bias();
    }

#if HAL_LOGGING_ENABLED
    posvel_runtime.log_sample(ESTIMATOR_RUNTIME_CINS_ID,
                              ESTIMATOR_RUNTIME_PHASE_POSVEL,
                              uint32_t(estimator_runtime_micros64() - phase_start_us));
#endif

}

ftype AP_CINS::update_vector_measurement_cts(const Vector3F &measurement, const Vector3F& reference, const Vector2F &ref_base, const ftype& gain_R, const ftype& gain_V, const ftype& gain_gyr_bias, const ftype& gain_acc_bias, const ftype dt)
{
    // Compute and apply an update for an arbitrary vector-type measurement.
    // The measurement is of the form $\mu = R \mu_0 + V C$, where
    // \mu is the measured value
    // \mu_0 is a reference vector (e.g. zero for GPS and 'north' for magnetometer)
    // C is a the reference vector 'base' (e.g. zero for magnetometer and (1,0) or (0,1) for GPS vel. or pos.)
    // The return value is the squared measurement error
    // The update equations are then drawn from Lemma 5.3 in
    // van Goor, Pieter, et al. "Constructive synchronous observer design for inertial navigation with delayed GNSS measurements." European Journal of Control (2024): 101047.

    // Set up some computation variables
    const Vector3F& muHat = state.XHat.rot() * reference + state.XHat.vel()*ref_base.x + state.XHat.pos()*ref_base.y;
    heapVars.ZInv = state.ZHat.inverse();
    const Vector2F& gains_AInvC = heapVars.ZInv.A() * ref_base;
    const GL2& CCT = GL2(ref_base.x*ref_base.x, ref_base.x*ref_base.y, ref_base.x*ref_base.y, ref_base.y*ref_base.y);
    const Vector3F& mu_Z = state.ZHat.W1()*gains_AInvC.x + state.ZHat.W2()*gains_AInvC.y;

    // Compute correction terms
    heapVars.S_Gamma = -0.5 * (gain_V) * heapVars.ZInv.A() * CCT * heapVars.ZInv.A().transposed();
    heapVars.W_Gamma_1 = - (measurement - mu_Z) * gains_AInvC.x * (gain_R+gain_V);
    heapVars.W_Gamma_2 = - (measurement - mu_Z) * gains_AInvC.y * (gain_R+gain_V);
    heapVars.GammaInv = SIM23(heapVars.I3, -heapVars.W_Gamma_1*dt, -heapVars.W_Gamma_2*dt, GL2::identity() - dt*heapVars.S_Gamma);

    heapVars.Omega_Delta = computeRotationCorrection((muHat - mu_Z), -(measurement - mu_Z), 4.*gain_R, dt);
    heapVars.W_Delta1 = (measurement - muHat) * gains_AInvC.x * (gain_R + gain_V);
    heapVars.W_Delta2 = (measurement - muHat) * gains_AInvC.y * (gain_R + gain_V);

    const ftype measurementErrorSq = sq((measurement - muHat).length());

    // Compute the bias correction

    // Construct the components of the 9x9 Adjoint matrix $\Ad_Z$
    // heapVars.AdZ_11 = state.ZHat.R();
    // heapVars.AdZ_12.zero(); // Zero matrix
    // heapVars.AdZ_13.zero(); // Zero matrix
    // heapVars.AdZ_21 = - Matrix3F::skew_symmetric(heapVars.ZInv.W1());
    // heapVars.AdZ_22 = state.ZHat.R() * heapVars.ZInv.A().a11();
    // heapVars.AdZ_23 = state.ZHat.R() * heapVars.ZInv.A().a21();
    // heapVars.AdZ_31 = - Matrix3F::skew_symmetric(heapVars.ZInv.W2());
    // heapVars.AdZ_32 = state.ZHat.R() * heapVars.ZInv.A().a12();
    // heapVars.AdZ_33 = state.ZHat.R() * heapVars.ZInv.A().a22();

    // Linearised measurement matrix $\mu - \hat{\mu} = Dh DR_{\hat{X}} \Ad_Z \varepsilon$.
    // const Matrix3F& pre_C11 = - Matrix3F::skew_symmetric(muHat);
    // const Matrix3F& pre_C12 = heapVars.I3*ref_base.x;
    // const Matrix3F& pre_C13 = heapVars.I3*ref_base.y;

    // const Matrix3F& C11 = pre_C11 * AdZ_11 + pre_C12 * AdZ_21 + pre_C13 * AdZ_31;
    // const Matrix3F& C12 = pre_C11 * AdZ_12 + pre_C12 * AdZ_22 + pre_C13 * AdZ_32;
    // const Matrix3F& C13 = pre_C11 * AdZ_13 + pre_C12 * AdZ_23 + pre_C13 * AdZ_33;
    heapVars.C11 = - Matrix3F::skew_symmetric(muHat) * state.ZHat.R();
    heapVars.C11 += -  Matrix3F::skew_symmetric(heapVars.ZInv.W1()) * ref_base.x;
    heapVars.C11 += - Matrix3F::skew_symmetric(heapVars.ZInv.W2()) * ref_base.y;
    heapVars.C12 = state.ZHat.R() * heapVars.ZInv.A().a11() * ref_base.x;
    heapVars.C12 += state.ZHat.R() * heapVars.ZInv.A().a12() * ref_base.y;
    heapVars.C13 = state.ZHat.R() * heapVars.ZInv.A().a21() * ref_base.x;
    heapVars.C13 += state.ZHat.R() * heapVars.ZInv.A().a22() * ref_base.y;

    // Gain matrix $\Delta = K (\mu - \hat{\mu})$.
    heapVars.K11 = Matrix3F::skew_symmetric(muHat - mu_Z) * 4. * gain_R * dt;
    heapVars.K21 = heapVars.I3 * gains_AInvC.x * (gain_R + gain_V) * dt;
    heapVars.K31 = heapVars.I3 * gains_AInvC.y * (gain_R + gain_V) * dt;

    // Compute the bias correction using the C matrices.
    // The correction to bias is given by the formula delta_b = k_b (MC)^\top (y-\hat{y})
    heapVars.M1Gyr = state.gyr_bias_gain_mat.rot;
    heapVars.M2Gyr = state.gyr_bias_gain_mat.vel;
    heapVars.M3Gyr = state.gyr_bias_gain_mat.pos;

    heapVars.M1Acc = state.acc_bias_gain_mat.rot;
    heapVars.M2Acc = state.acc_bias_gain_mat.vel;
    heapVars.M3Acc = state.acc_bias_gain_mat.pos;

    state.gyr_bias_correction = (heapVars.C11*heapVars.M1Gyr + heapVars.C12*heapVars.M2Gyr + heapVars.C13*heapVars.M3Gyr).mul_transpose(measurement - muHat) * gain_gyr_bias;
    state.acc_bias_correction = (heapVars.C11*heapVars.M1Acc + heapVars.C12*heapVars.M2Acc + heapVars.C13*heapVars.M3Acc).mul_transpose(measurement - muHat) * gain_acc_bias;
    saturate_bias(state.gyr_bias_correction, state.gyr_bias, gains.sat_gyr_bias.get(), dt);
    saturate_bias(state.acc_bias_correction, state.acc_bias, gains.sat_acc_bias.get(), dt);

    // Compute the bias gain matrix updates
    heapVars.A11 = heapVars.I3;
    heapVars.A12.zero(); // Zero matrix
    heapVars.A13.zero(); // Zero matrix
    heapVars.A21 = - Matrix3F::skew_symmetric(heapVars.GammaInv.W1());
    heapVars.A22 = heapVars.I3*heapVars.GammaInv.A().a11();
    heapVars.A23 = heapVars.I3*heapVars.GammaInv.A().a21();
    heapVars.A31 = - Matrix3F::skew_symmetric(heapVars.GammaInv.W2());
    heapVars.A32 = heapVars.I3*heapVars.GammaInv.A().a12();
    heapVars.A33 = heapVars.I3*heapVars.GammaInv.A().a22();

    state.gyr_bias_gain_mat.rot = (heapVars.A11 - heapVars.K11 * heapVars.C11) * heapVars.M1Gyr;
    state.gyr_bias_gain_mat.vel = (heapVars.A21 - heapVars.K21 * heapVars.C11) * heapVars.M1Gyr;
    state.gyr_bias_gain_mat.pos = (heapVars.A31 - heapVars.K31 * heapVars.C11) * heapVars.M1Gyr;
    state.acc_bias_gain_mat.rot = (heapVars.A11 - heapVars.K11 * heapVars.C11) * heapVars.M1Acc;
    state.acc_bias_gain_mat.vel = (heapVars.A21 - heapVars.K21 * heapVars.C11) * heapVars.M1Acc;
    state.acc_bias_gain_mat.pos = (heapVars.A31 - heapVars.K31 * heapVars.C11) * heapVars.M1Acc;
    state.gyr_bias_gain_mat.rot += (heapVars.A12 - heapVars.K11 * heapVars.C12) * heapVars.M2Gyr;
    state.gyr_bias_gain_mat.vel += (heapVars.A22 - heapVars.K21 * heapVars.C12) * heapVars.M2Gyr;
    state.gyr_bias_gain_mat.pos += (heapVars.A32 - heapVars.K31 * heapVars.C12) * heapVars.M2Gyr;
    state.acc_bias_gain_mat.rot += (heapVars.A12 - heapVars.K11 * heapVars.C12) * heapVars.M2Acc;
    state.acc_bias_gain_mat.vel += (heapVars.A22 - heapVars.K21 * heapVars.C12) * heapVars.M2Acc;
    state.acc_bias_gain_mat.pos += (heapVars.A32 - heapVars.K31 * heapVars.C12) * heapVars.M2Acc;
    state.gyr_bias_gain_mat.rot += (heapVars.A13 - heapVars.K11 * heapVars.C13) * heapVars.M3Gyr;
    state.gyr_bias_gain_mat.vel += (heapVars.A23 - heapVars.K21 * heapVars.C13) * heapVars.M3Gyr;
    state.gyr_bias_gain_mat.pos += (heapVars.A33 - heapVars.K31 * heapVars.C13) * heapVars.M3Gyr;
    state.acc_bias_gain_mat.rot += (heapVars.A13 - heapVars.K11 * heapVars.C13) * heapVars.M3Acc;
    state.acc_bias_gain_mat.vel += (heapVars.A23 - heapVars.K21 * heapVars.C13) * heapVars.M3Acc;
    state.acc_bias_gain_mat.pos += (heapVars.A33 - heapVars.K31 * heapVars.C13) * heapVars.M3Acc;

    // Apply the state and bias updates
    state.gyr_bias += state.gyr_bias_correction * dt;
    state.acc_bias += state.acc_bias_correction * dt;
    heapVars.Omega_Delta += state.gyr_bias_gain_mat.rot * state.gyr_bias_correction + state.acc_bias_gain_mat.rot * state.acc_bias_correction;
    // SIM23 stores velocity in W1 and position in W2.
    heapVars.W_Delta1 += state.gyr_bias_gain_mat.vel * state.gyr_bias_correction + state.acc_bias_gain_mat.vel * state.acc_bias_correction;
    heapVars.W_Delta2 += state.gyr_bias_gain_mat.pos * state.gyr_bias_correction + state.acc_bias_gain_mat.pos * state.acc_bias_correction;

    // Construct the correction term Delta
    heapVars.Delta_SIM23 = SIM23(Matrix3F::from_angular_velocity(heapVars.Omega_Delta*dt), heapVars.W_Delta1*dt, heapVars.W_Delta2*dt, GL2::identity());
    heapVars.Delta_SIM23 = state.ZHat * heapVars.Delta_SIM23 * heapVars.ZInv;

    // Update the states using the correction terms
    state.XHat = Gal3F(heapVars.Delta_SIM23.R(), heapVars.Delta_SIM23.W2(), heapVars.Delta_SIM23.W1(), 0.) * state.XHat;
    state.ZHat = state.ZHat * heapVars.GammaInv;

    return measurementErrorSq;
}

ftype AP_CINS::update_vector_measurement_discrete(
    const Vector3F &measurement,
    const Vector3F &reference,
    const Vector2F &ref_base,
    const ftype &gain_R,
    const ftype &gain_V,
    const ftype &gain_gyr_bias,
    const ftype &gain_acc_bias,
    const ftype h_requested,
    const uint8_t update_kind)
{
	// Predicted measurement:
    // muHat = RHat * mu0 + VHat * C
    const Vector3F muHat =
        state.XHat.rot() * reference
        + state.XHat.vel() * ref_base.x
        + state.XHat.pos() * ref_base.y;

    const SIM23 Z_before = state.ZHat;
    const SIM23 ZInv = Z_before.inverse();

    // c = A_Z^{-1} C
    const Vector2F c = ZInv.A() * ref_base;

    // mu_Z = V_Z c
    const Vector3F mu_Z =
        Z_before.W1() * c.x
        + Z_before.W2() * c.y;

    // a = mu - mu_Z
    const Vector3F a = measurement - mu_Z;

    // b = muHat - mu_Z
    const Vector3F b = muHat - mu_Z;

    const ftype measurementErrorSq =
        (measurement - muHat).length_squared();

	const Vector3F omega =
        (b % a) * (4.0 * gain_R);

	const ftype n = c.length_squared();
    const ftype b_norm_sq = b.length_squared();
    const ftype omega_norm = omega.length();

    // Current proof assumes kV > 0.
    // For the initial GPS implementation, do no finite correction if it is not.
    if (!is_positive(gain_V) || !is_positive(h_requested)) {
        return measurementErrorSq;
    }

    constexpr ftype sigma =
        2.0 / 3.14159265358979323846;

    const ftype C_bound =
        b_norm_sq *
            (2.0 * gain_R
             + 36.0 * gain_R * gain_R / gain_V)
        + n *
            ((gain_R + gain_V) *
             (gain_R + gain_V) / gain_V);

	constexpr ftype rho = 0.5;

    ftype h = h_requested;

    if (C_bound > 1.0e-12) {
        h = MIN(
            h,
            rho * sigma / C_bound);
    }

    if (omega_norm > 1.0e-12) {
        constexpr ftype half_pi =
            1.57079632679489661923;

        h = MIN(
            h,
            rho * half_pi / omega_norm);
    }

    if (!is_positive(h)) {
        return measurementErrorSq;
    }

	#if HAL_LOGGING_ENABLED
		if (detail_log_rate.get() > 0) {
			AP::logger().WriteStreaming(
				"CIDU",
				"TimeUS,Kind,HReq,H,CBnd,Omega,CNorm",
				"s-ss---",
				"F-00---",
				"QBfffff",
				AP::dal().micros64(),
				update_kind,
				float(h_requested),
				float(h),
				float(C_bound),
				float(omega_norm),
				float(sqrtF(n)));
		}
	#endif

	const ftype s =
        sqrtF(1.0 + h * gain_V * n);

    // Q = exp(h omega^x)
    const Matrix3F Q =
        Matrix3F::from_angular_velocity(omega * h);

    // A = I + [h kV / (s+1)] c c^T
    const ftype alpha =
        h * gain_V / (s + 1.0);

    const GL2 A_step(
        1.0 + alpha * c.x * c.x,
        alpha * c.x * c.y,
        alpha * c.x * c.y,
        1.0 + alpha * c.y * c.y);

    const ftype lambda =
        h * (gain_R + gain_V) / s;

	const Vector3F U_base = a * lambda;

    const Vector3F U1 = U_base * c.x;
    const Vector3F U2 = U_base * c.y;

	const Vector3F W_base =
        (a - Q * b) * (lambda / s);

    const Vector3F W1 = W_base * c.x;
    const Vector3F W2 = W_base * c.y;

	const SIM23 D(
        Q,
        W1,
        W2,
        GL2::identity());

    const SIM23 B(
        heapVars.I3,
        U1,
        U2,
        A_step);

    // ---------------------------------------------------------------------
    // Bias-augmented finite map
    // ---------------------------------------------------------------------
    // Snapshot M^- before updating the finite sensor sensitivity.
    heapVars.M1Gyr = state.gyr_bias_gain_mat.rot;
    heapVars.M2Gyr = state.gyr_bias_gain_mat.vel;
    heapVars.M3Gyr = state.gyr_bias_gain_mat.pos;
    heapVars.M1Acc = state.acc_bias_gain_mat.rot;
    heapVars.M2Acc = state.acc_bias_gain_mat.vel;
    heapVars.M3Acc = state.acc_bias_gain_mat.pos;

    // Existing CINS measurement Jacobian C = Dh DR_Xhat Ad_Z. Reusing this
    // keeps the finite sensitivity update in the same coordinates as the
    // continuous bias estimator.
    heapVars.C11 = -Matrix3F::skew_symmetric(muHat) * Z_before.R();
    heapVars.C11 += -Matrix3F::skew_symmetric(ZInv.W1()) * ref_base.x;
    heapVars.C11 += -Matrix3F::skew_symmetric(ZInv.W2()) * ref_base.y;
    heapVars.C12 = Z_before.R() * ZInv.A().a11() * ref_base.x;
    heapVars.C12 += Z_before.R() * ZInv.A().a12() * ref_base.y;
    heapVars.C13 = Z_before.R() * ZInv.A().a21() * ref_base.x;
    heapVars.C13 += Z_before.R() * ZInv.A().a22() * ref_base.y;

    // Reuse heap scratch matrices for H M_g and H M_a to avoid adding large
    // Matrix3F temporaries to the estimator stack.
    heapVars.A12 =
        heapVars.C11 * heapVars.M1Gyr
        + heapVars.C12 * heapVars.M2Gyr
        + heapVars.C13 * heapVars.M3Gyr;
    heapVars.A13 =
        heapVars.C11 * heapVars.M1Acc
        + heapVars.C12 * heapVars.M2Acc
        + heapVars.C13 * heapVars.M3Acc;

    const Vector3F measurement_residual = measurement - muHat;
    state.gyr_bias_correction =
        heapVars.A12.mul_transpose(measurement_residual) * gain_gyr_bias;
    state.acc_bias_correction =
        heapVars.A13.mul_transpose(measurement_residual) * gain_acc_bias;

    // Finite-step safeguard for the isolated bias-gradient update. Avoid a
    // 6x6 eigenvalue calculation onboard by using the conservative bound
    // lambda_max <= k_bg ||H M_g||_F^2 + k_ba ||H M_a||_F^2.
    const ftype gyr_sensor_sensitivity_sq =
        heapVars.A12.a.length_squared()
        + heapVars.A12.b.length_squared()
        + heapVars.A12.c.length_squared();
    const ftype acc_sensor_sensitivity_sq =
        heapVars.A13.a.length_squared()
        + heapVars.A13.b.length_squared()
        + heapVars.A13.c.length_squared();

    const ftype bias_lambda_bound =
        gain_gyr_bias * gyr_sensor_sensitivity_sq
        + gain_acc_bias * acc_sensor_sensitivity_sq;

    constexpr ftype bias_rho = 0.5;
    ftype h_bias = h;
    if (bias_lambda_bound > 1.0e-12) {
        h_bias = MIN(h_bias, (2.0 * bias_rho) / bias_lambda_bound);
    }

    saturate_bias(state.gyr_bias_correction, state.gyr_bias,
                  gains.sat_gyr_bias.get(), h_bias);
    saturate_bias(state.acc_bias_correction, state.acc_bias,
                  gains.sat_acc_bias.get(), h_bias);

    // Finite-map counterpart of the continuous M^+=(A-KC)M update. A_f is
    // induced by the actual finite auxiliary factor B; K_f uses the actual
    // finite translation coefficient lambda.
    heapVars.A11 = heapVars.I3;
    heapVars.A12.zero();
    heapVars.A13.zero();
    heapVars.A21 = -Matrix3F::skew_symmetric(U1);
    heapVars.A22 = heapVars.I3 * A_step.a11();
    heapVars.A23 = heapVars.I3 * A_step.a21();
    heapVars.A31 = -Matrix3F::skew_symmetric(U2);
    heapVars.A32 = heapVars.I3 * A_step.a12();
    heapVars.A33 = heapVars.I3 * A_step.a22();

    heapVars.K11 = Matrix3F::skew_symmetric(b) * (4.0 * gain_R * h);
    heapVars.K21 = heapVars.I3 * (lambda * c.x);
    heapVars.K31 = heapVars.I3 * (lambda * c.y);

    state.gyr_bias_gain_mat.rot = (heapVars.A11 - heapVars.K11 * heapVars.C11) * heapVars.M1Gyr;
    state.gyr_bias_gain_mat.vel = (heapVars.A21 - heapVars.K21 * heapVars.C11) * heapVars.M1Gyr;
    state.gyr_bias_gain_mat.pos = (heapVars.A31 - heapVars.K31 * heapVars.C11) * heapVars.M1Gyr;
    state.acc_bias_gain_mat.rot = (heapVars.A11 - heapVars.K11 * heapVars.C11) * heapVars.M1Acc;
    state.acc_bias_gain_mat.vel = (heapVars.A21 - heapVars.K21 * heapVars.C11) * heapVars.M1Acc;
    state.acc_bias_gain_mat.pos = (heapVars.A31 - heapVars.K31 * heapVars.C11) * heapVars.M1Acc;

    state.gyr_bias_gain_mat.rot += (heapVars.A12 - heapVars.K11 * heapVars.C12) * heapVars.M2Gyr;
    state.gyr_bias_gain_mat.vel += (heapVars.A22 - heapVars.K21 * heapVars.C12) * heapVars.M2Gyr;
    state.gyr_bias_gain_mat.pos += (heapVars.A32 - heapVars.K31 * heapVars.C12) * heapVars.M2Gyr;
    state.acc_bias_gain_mat.rot += (heapVars.A12 - heapVars.K11 * heapVars.C12) * heapVars.M2Acc;
    state.acc_bias_gain_mat.vel += (heapVars.A22 - heapVars.K21 * heapVars.C12) * heapVars.M2Acc;
    state.acc_bias_gain_mat.pos += (heapVars.A32 - heapVars.K31 * heapVars.C12) * heapVars.M2Acc;

    state.gyr_bias_gain_mat.rot += (heapVars.A13 - heapVars.K11 * heapVars.C13) * heapVars.M3Gyr;
    state.gyr_bias_gain_mat.vel += (heapVars.A23 - heapVars.K21 * heapVars.C13) * heapVars.M3Gyr;
    state.gyr_bias_gain_mat.pos += (heapVars.A33 - heapVars.K31 * heapVars.C13) * heapVars.M3Gyr;
    state.acc_bias_gain_mat.rot += (heapVars.A13 - heapVars.K11 * heapVars.C13) * heapVars.M3Acc;
    state.acc_bias_gain_mat.vel += (heapVars.A23 - heapVars.K21 * heapVars.C13) * heapVars.M3Acc;
    state.acc_bias_gain_mat.pos += (heapVars.A33 - heapVars.K31 * heapVars.C13) * heapVars.M3Acc;

	const SIM23 correction =
        Z_before * D * ZInv;

    // SIM23 stores W1=velocity, W2=position.
    // Gal3 constructor order is R, position, velocity, tau.
    const Gal3F correction_gal(
        correction.R(),
        correction.W2(),
        correction.W1(),
        0.0);

    state.XHat =
        correction_gal * state.XHat;

    state.ZHat =
        Z_before * B;

    // Pair the bias-state jump with the post-map sensitivity M^+. This mirrors
    // the continuous observer's M^+ delta_b injection but uses the finite-map
    // sensitivity transition above.
    heapVars.Omega_Delta =
        (state.gyr_bias_gain_mat.rot * state.gyr_bias_correction
         + state.acc_bias_gain_mat.rot * state.acc_bias_correction) * h_bias;
    heapVars.W_Delta1 =
        (state.gyr_bias_gain_mat.vel * state.gyr_bias_correction
         + state.acc_bias_gain_mat.vel * state.acc_bias_correction) * h_bias;
    heapVars.W_Delta2 =
        (state.gyr_bias_gain_mat.pos * state.gyr_bias_correction
         + state.acc_bias_gain_mat.pos * state.acc_bias_correction) * h_bias;

    heapVars.Delta_SIM23 = SIM23(
        Matrix3F::from_angular_velocity(heapVars.Omega_Delta),
        heapVars.W_Delta1,
        heapVars.W_Delta2,
        GL2::identity());
    heapVars.ZInv = state.ZHat.inverse();
    heapVars.Delta_SIM23 =
        state.ZHat * heapVars.Delta_SIM23 * heapVars.ZInv;

    state.XHat = Gal3F(
        heapVars.Delta_SIM23.R(),
        heapVars.Delta_SIM23.W2(),
        heapVars.Delta_SIM23.W1(),
        0.0) * state.XHat;

    state.gyr_bias += state.gyr_bias_correction * h_bias;
    state.acc_bias += state.acc_bias_correction * h_bias;

    return measurementErrorSq;
}

ftype AP_CINS::update_attitude_measurement_discrete(
    const Vector3F &measurement,
    const Vector3F &reference,
    const ftype &gain_R,
    const ftype &gain_gyr_bias,
    const ftype &gain_acc_bias,
    const ftype h_requested)
{
    // Attitude-only measurement:
    //
    // mu = R * mu0
    //
    // For magnetometer C = 0, therefore:
    // c = 0
    // mu_Z = 0
    // A_step = I
    // U = 0
    // W = 0
    //
    // The finite correction therefore only requires Q.

    const Vector3F muHat =
        state.XHat.rot() * reference;

    const ftype measurementErrorSq =
        (measurement - muHat).length_squared();

    const bool attitude_update_enabled = is_positive(gain_R);
    const bool bias_update_enabled =
        is_positive(gain_gyr_bias) || is_positive(gain_acc_bias);

    // Allow a bias-only magnetometer update (for example MGATT=0, MAGGB>0).
    // This mirrors the continuous observer, where magnetometer bias adaptation
    // does not require a nonzero direct attitude gain.
    if (!is_positive(h_requested) ||
        (!attitude_update_enabled && !bias_update_enabled)) {
        return measurementErrorSq;
    }

    const SIM23 Z_before = state.ZHat;
    const SIM23 ZInv = Z_before.inverse();

    // For C = 0:
    // a = measurement
    // b = muHat
    const Vector3F &a = measurement;
    const Vector3F &b = muHat;

    // omega = 4 k_R (b x a)
    const Vector3F omega =
        (b % a) * (4.0 * gain_R);

    const ftype omega_norm = omega.length();
    const ftype b_norm_sq = b.length_squared();

    // For the attitude-only map the finite-step bound reduces to:
    //
    // C_mag = 2 k_R ||b||^2
    //
    // h <= min(
    //     sigma / C_mag,
    //     pi / (2 ||omega||)
    // )
    //
    // with sigma = 2/pi.

    constexpr ftype sigma =
        2.0 / 3.14159265358979323846;

    const ftype C_bound =
        2.0 * gain_R * b_norm_sq;

    constexpr ftype rho = 0.5;

    ftype h = h_requested;

    if (C_bound > 1.0e-12) {
        h = MIN(
            h,
            rho * sigma / C_bound);
    }

    if (omega_norm > 1.0e-12) {
        constexpr ftype half_pi =
            1.57079632679489661923;

        h = MIN(
            h,
            rho * half_pi / omega_norm);
    }

    if (!is_positive(h)) {
        return measurementErrorSq;
    }

    // ---------------------------------------------------------------------
    // Bias-augmented finite attitude map
    // ---------------------------------------------------------------------
    // Snapshot M^- before the native magnetometer jump. For C=0 the
    // measurement Jacobian contains only C11, but both gyro- and accel-bias
    // sensitivities may contribute through their attitude blocks.
    heapVars.M1Gyr = state.gyr_bias_gain_mat.rot;
    heapVars.M2Gyr = state.gyr_bias_gain_mat.vel;
    heapVars.M3Gyr = state.gyr_bias_gain_mat.pos;
    heapVars.M1Acc = state.acc_bias_gain_mat.rot;
    heapVars.M2Acc = state.acc_bias_gain_mat.vel;
    heapVars.M3Acc = state.acc_bias_gain_mat.pos;

    // For the attitude-only measurement, ref_base=0 and mu_Z=0, hence
    // C11 = -skew(muHat) R_Z and C12=C13=0.
    heapVars.C11 =
        -Matrix3F::skew_symmetric(muHat) * Z_before.R();

    // Reuse heap scratch matrices for H M_g and H M_a.
    heapVars.A12 = heapVars.C11 * heapVars.M1Gyr;
    heapVars.A13 = heapVars.C11 * heapVars.M1Acc;

    const Vector3F measurement_residual = measurement - muHat;
    state.gyr_bias_correction =
        heapVars.A12.mul_transpose(measurement_residual) * gain_gyr_bias;
    state.acc_bias_correction =
        heapVars.A13.mul_transpose(measurement_residual) * gain_acc_bias;

    // Same conservative finite bias-step bound used by the finite GPS maps:
    // lambda_max <= k_bg ||H M_g||_F^2 + k_ba ||H M_a||_F^2.
    const ftype gyr_sensor_sensitivity_sq =
        heapVars.A12.a.length_squared()
        + heapVars.A12.b.length_squared()
        + heapVars.A12.c.length_squared();
    const ftype acc_sensor_sensitivity_sq =
        heapVars.A13.a.length_squared()
        + heapVars.A13.b.length_squared()
        + heapVars.A13.c.length_squared();

    const ftype bias_lambda_bound =
        gain_gyr_bias * gyr_sensor_sensitivity_sq
        + gain_acc_bias * acc_sensor_sensitivity_sq;

    constexpr ftype bias_rho = 0.5;
    ftype h_bias = h;
    if (bias_lambda_bound > 1.0e-12) {
        h_bias = MIN(h_bias, (2.0 * bias_rho) / bias_lambda_bound);
    }

    saturate_bias(state.gyr_bias_correction, state.gyr_bias,
                  gains.sat_gyr_bias.get(), h_bias);
    saturate_bias(state.acc_bias_correction, state.acc_bias,
                  gains.sat_acc_bias.get(), h_bias);

#if HAL_LOGGING_ENABLED
    if (detail_log_rate.get() > 0) {
        // Reuse CIDU:
        // Kind 0 = GPS position
        // Kind 1 = GPS velocity
        // Kind 2 = magnetometer
        AP::logger().WriteStreaming(
            "CIDU",
            "TimeUS,Kind,HReq,H,CBnd,Omega,CNorm",
            "s-ss---",
            "F-00---",
            "QBfffff",
            AP::dal().micros64(),
            uint8_t(2),
            float(h_requested),
            float(h),
            float(C_bound),
            float(omega_norm),
            0.0f);
    }
#endif

    // Q = exp(h omega^x)
    const Matrix3F Q =
        Matrix3F::from_angular_velocity(omega * h);

    // D = [Q  0
    //      0  I]
    const SIM23 D(
        Q,
        heapVars.zero_vector,
        heapVars.zero_vector,
        GL2::identity());

    // Finite attitude-map sensitivity transition. Since C=0 gives B=I,
    // the lower translation rows are unchanged. The attitude block is the
    // finite counterpart of the continuous (I-K11*C11) update. This remains
    // the identity when MGATT=0, allowing a pure bias-adaptation event.
    heapVars.K11 =
        Matrix3F::skew_symmetric(b) * (4.0 * gain_R * h);
    heapVars.A11 = heapVars.I3 - heapVars.K11 * heapVars.C11;

    state.gyr_bias_gain_mat.rot = heapVars.A11 * heapVars.M1Gyr;
    state.gyr_bias_gain_mat.vel = heapVars.M2Gyr;
    state.gyr_bias_gain_mat.pos = heapVars.M3Gyr;
    state.acc_bias_gain_mat.rot = heapVars.A11 * heapVars.M1Acc;
    state.acc_bias_gain_mat.vel = heapVars.M2Acc;
    state.acc_bias_gain_mat.pos = heapVars.M3Acc;

    // Apply the exact finite group action:
    //
    // XHat+ = Z D Z^-1 XHat
    //
    // Note that although D itself contains no translational component,
    // conjugating by Z can produce velocity/position components in the
    // correction applied to XHat.
    const SIM23 correction =
        Z_before * D * ZInv;

	const Gal3F correction_gal(
		correction.R(),
		correction.W2(),
		correction.W1(),
		0.0);

	// Save state immediately before MAG jump
	const Vector3F vel_before = state.XHat.vel();
	const Vector3F pos_before = state.XHat.pos();

	state.XHat =
		correction_gal * state.XHat;

    // Pair the bias-state jump with the post-map sensitivity M^+. ZHat is
    // unchanged by the attitude-only native map, so use the same Z frame for
    // the conjugated navigation correction.
    heapVars.Omega_Delta =
        (state.gyr_bias_gain_mat.rot * state.gyr_bias_correction
         + state.acc_bias_gain_mat.rot * state.acc_bias_correction) * h_bias;
    heapVars.W_Delta1 =
        (state.gyr_bias_gain_mat.vel * state.gyr_bias_correction
         + state.acc_bias_gain_mat.vel * state.acc_bias_correction) * h_bias;
    heapVars.W_Delta2 =
        (state.gyr_bias_gain_mat.pos * state.gyr_bias_correction
         + state.acc_bias_gain_mat.pos * state.acc_bias_correction) * h_bias;

    heapVars.Delta_SIM23 = SIM23(
        Matrix3F::from_angular_velocity(heapVars.Omega_Delta),
        heapVars.W_Delta1,
        heapVars.W_Delta2,
        GL2::identity());
    heapVars.Delta_SIM23 =
        Z_before * heapVars.Delta_SIM23 * ZInv;

    state.XHat = Gal3F(
        heapVars.Delta_SIM23.R(),
        heapVars.Delta_SIM23.W2(),
        heapVars.Delta_SIM23.W1(),
        0.0) * state.XHat;

    state.gyr_bias += state.gyr_bias_correction * h_bias;
    state.acc_bias += state.acc_bias_correction * h_bias;

	const Vector3F delta_vel =
		state.XHat.vel() - vel_before;

	const Vector3F delta_pos =
		state.XHat.pos() - pos_before;

	const ftype delta_theta =
		h * omega_norm;

	#if HAL_LOGGING_ENABLED
	if (detail_log_rate.get() > 0) {
		AP::logger().WriteStreaming(
			"CIMJ",
			"TimeUS,DTh,DVN,DVE,DVD,DPN,DPE,DPD",
			"s-------",
			"F-------",
			"Qfffffff",
			AP::dal().micros64(),
			float(delta_theta),
			float(delta_vel.x),
			float(delta_vel.y),
			float(delta_vel.z),
			float(delta_pos.x),
			float(delta_pos.y),
			float(delta_pos.z));
	}
	#endif

	// IMPORTANT: no ZHat update here.
	// For C = 0, B = I and Z is unchanged.

	return measurementErrorSq;
}

void AP_CINS::update_Kq_discrete(const ftype h)
{
    if (!is_positive(h)) {
        return;
    }

    const GL2 Kq(
        gains.Q11.get(), 0.0,
        0.0, gains.Q22.get());

    // Mq = A_Z^T Kq A_Z
    const GL2 Mq =
        state.ZHat.A().transposed()
        * Kq
        * state.ZHat.A();

    // N = I + h Mq
    const GL2 N =
        GL2::identity() + h * Mq;

	const ftype det_N = N.det();

    if (det_N <= 0.0) {
        // Should never occur for valid Kq >= 0.
        return;
    }

    const ftype root_det =
        sqrtF(det_N);

    const ftype denom =
        sqrtF(N.trace() + 2.0 * root_det);

    const GL2 sqrt_N =
        (1.0 / denom)
        * (N + root_det * GL2::identity());

    const GL2 Hq =
        sqrt_N.inverse();

	const SIM23 Bq(
        heapVars.I3,
        heapVars.zero_vector,
        heapVars.zero_vector,
        Hq);

    // Keep the bias sensitivity in the coordinates induced by the same exact
    // finite Kq right action applied to ZHat.
    compute_bias_update_imu(Bq.inverse());

    state.ZHat =
        state.ZHat * Bq;
}

/*
  initialise yaw from compass, if available
 */
bool AP_CINS::init_yaw(void)
{
    // CINS starts with XHat = identity(), so its roll/pitch are not yet usable
    // for compass tilt compensation. Initialise roll and pitch from the
    // stationary accelerometer first, then compute magnetic yaw using that
    // explicit attitude.
    auto &dal = AP::dal();
    const auto &ins = dal.ins();

    const uint8_t accel_idx = ins.get_first_usable_accel();
    const Vector3f accel = ins.get_accel(accel_idx);
    const float accel_norm = accel.length();

    // Only initialise attitude when the measured acceleration is close to 1 g.
    // This avoids treating translational acceleration as vehicle tilt.
    if (!isfinite(accel_norm) || accel_norm < 9.0f || accel_norm > 11.0f) {
        return false;
    }

    const ftype pitch_rad =
        atan2f(accel.x, norm(accel.y, accel.z));
    const ftype roll_rad =
        atan2f(-accel.y, -accel.z);

    Matrix3F initial_attitude;
    initial_attitude.from_euler(roll_rad, pitch_rad, 0.0);

    ftype mag_yaw, dt;
    if (!get_compass_yaw(initial_attitude, mag_yaw, dt)) {
        return false;
    }

    state.XHat.rot().from_euler(roll_rad, pitch_rad, mag_yaw);

#if HAL_LOGGING_ENABLED
    AP::logger().WriteEstimatorReset(AP::dal().micros64(), AP::dal().millis(), 1, DAL_CORE(0),
                                    AP_Logger::EstimatorReset::ATTITUDE, 255);
#endif

    return true;
}

/*
  get yaw from compass
 */
bool AP_CINS::get_compass_yaw(const Matrix3F &attitude, ftype &yaw_rad, ftype &dt)
{
    auto &dal = AP::dal();
    const auto &compass = dal.compass();
    if (compass.get_num_enabled() == 0) {
        return false;
    }
    const uint8_t mag_idx = compass.get_first_usable();
    if (!compass.healthy(mag_idx)) {
        return false;
    }
    if (!state.have_origin) {
        return false;
    }
    const auto &field = compass.get_field(mag_idx);
    const uint32_t last_us = compass.last_update_usec(mag_idx);
    if (last_us == last_mag_us) {
        // no new data
        return false;
    }
    dt = (last_us - last_mag_us) * 1.0e-6;
    last_mag_us = last_us;

    const float declination_rad = dal.compass().get_declination();
    if (is_zero(declination_rad)) {
        // wait for declination
        return false;
    }

    // Tilt-compensate the magnetic field using the explicitly supplied
    // roll/pitch attitude. Do not use state.XHat here: during startup XHat has
    // only just been reset and does not yet contain a valid tilt estimate.
    const float cos_pitch_sq =
        1.0f - attitude.c.x * attitude.c.x;

    const float headY =
        field.y * attitude.c.z - field.z * attitude.c.y;

    // Tilt-compensated magnetic field X component.
    const float headX =
        field.x * cos_pitch_sq
        - attitude.c.x *
            (field.y * attitude.c.y + field.z * attitude.c.z);

    const float heading =
        constrain_float(atan2f(-headY, headX), -M_PI, M_PI);

    yaw_rad = wrap_PI(heading + declination_rad);

    return true;
}


bool AP_CINS::get_compass_vector(Vector3F &mag_vec, Vector3F &mag_ref, ftype &dt)
{
    auto &dal = AP::dal();
    const auto &compass = dal.compass();
    if (compass.get_num_enabled() == 0) {
        return false;
    }
    const uint8_t mag_idx = compass.get_first_usable();
    if (!compass.healthy(mag_idx)) {
        return false;
    }
    if (!state.have_origin) {
        return false;
    }
    mag_vec = compass.get_field(mag_idx).toftype();
    const uint32_t last_us = compass.last_update_usec(mag_idx);
    if (last_us == last_mag_us) {
        // no new data
        return false;
    }
    dt = (last_us - last_mag_us) * 1.0e-6;
    last_mag_us = last_us;

    const Location loc = get_location();
    mag_ref = AP_Declination::get_earth_field_ga(loc).toftype();
    if (mag_ref.is_zero()) {
        // wait for declination
        return false;
    }

    return true;
}

void AP_CINS::update_attitude_from_compass()
{
	if (!done_yaw_init) {
        return;
    }

    ftype dt;
    Vector3F mag_vec, mag_ref;
    if (!get_compass_vector(mag_vec, mag_ref, dt)) {
        return;
    }
    mag_vec *= 1.e-3; // Convert mag measurement from milliGauss to Gauss


    ftype magErrorSq;

	if (gps_correction_mode() == 2) {

		// Finite event-based magnetometer correction.
		// get_compass_vector() already ensures this only runs
		// once for each fresh magnetometer sample.
		magErrorSq =
			update_attitude_measurement_discrete(
				mag_ref,
				mag_vec,
				gains.mag_att.get(),
				gains.mag_gyr_bias.get(),
				gains.mag_acc_bias.get(),
				dt);

	} else {

		// Existing continuous correction
		const Vector2F zero_2(0.0, 0.0);

		magErrorSq =
			update_vector_measurement_cts(
				mag_ref,
				mag_vec,
				zero_2,
				gains.mag_att.get(),
				0.0,
				gains.mag_gyr_bias.get(),
				gains.mag_acc_bias.get(),
				dt);
	}

	state.variances.magTest = magErrorSq / (state.variances.magVar * gains.variance_scale.get());
    state.variances.magVar =  gains.variance_lowpass.get() * magErrorSq + (1.-gains.variance_lowpass.get()) * state.variances.magVar;
}

void AP_CINS::compute_bias_update_imu(const SIM23& Gamma)
{
    // Compute the bias update for IMU inputs

    heapVars.GammaInv = Gamma.inverse();
    heapVars.A11 = Gamma.R();
    heapVars.A12.zero(); // Zero matrix
    heapVars.A13.zero(); // Zero matrix
    heapVars.A21 = - Matrix3F::skew_symmetric(heapVars.GammaInv.W1()) * Gamma.R();
    heapVars.A22 = Gamma.R() * heapVars.GammaInv.A().a11();
    heapVars.A23 = Gamma.R() * heapVars.GammaInv.A().a21();
    heapVars.A31 = - Matrix3F::skew_symmetric(heapVars.GammaInv.W2()) * Gamma.R();
    heapVars.A32 = Gamma.R() * heapVars.GammaInv.A().a12();
    heapVars.A33 = Gamma.R() * heapVars.GammaInv.A().a22();

    // Implement M(t+) = A M(t)
    heapVars.M1Gyr = state.gyr_bias_gain_mat.rot;
    heapVars.M2Gyr = state.gyr_bias_gain_mat.vel;
    heapVars.M3Gyr = state.gyr_bias_gain_mat.pos;
    state.gyr_bias_gain_mat.rot = heapVars.A11 * heapVars.M1Gyr;
    state.gyr_bias_gain_mat.rot += heapVars.A12 * heapVars.M2Gyr;
    state.gyr_bias_gain_mat.rot += heapVars.A13 * heapVars.M3Gyr;
    state.gyr_bias_gain_mat.vel = heapVars.A21 * heapVars.M1Gyr;
    state.gyr_bias_gain_mat.vel += heapVars.A22 * heapVars.M2Gyr;
    state.gyr_bias_gain_mat.vel += heapVars.A23 * heapVars.M3Gyr;
    state.gyr_bias_gain_mat.pos = heapVars.A31 * heapVars.M1Gyr;
    state.gyr_bias_gain_mat.pos += heapVars.A32 * heapVars.M2Gyr;
    state.gyr_bias_gain_mat.pos += heapVars.A33 * heapVars.M3Gyr;

    heapVars.M1Acc = state.acc_bias_gain_mat.rot;
    heapVars.M2Acc = state.acc_bias_gain_mat.vel;
    heapVars.M3Acc = state.acc_bias_gain_mat.pos;
    state.acc_bias_gain_mat.rot = heapVars.A11 * heapVars.M1Acc;
    state.acc_bias_gain_mat.rot += heapVars.A12 * heapVars.M2Acc;
    state.acc_bias_gain_mat.rot += heapVars.A13 * heapVars.M3Acc;
    state.acc_bias_gain_mat.vel = heapVars.A21 * heapVars.M1Acc;
    state.acc_bias_gain_mat.vel += heapVars.A22 * heapVars.M2Acc;
    state.acc_bias_gain_mat.vel += heapVars.A23 * heapVars.M3Acc;
    state.acc_bias_gain_mat.pos = heapVars.A31 * heapVars.M1Acc;
    state.acc_bias_gain_mat.pos += heapVars.A32 * heapVars.M2Acc;
    state.acc_bias_gain_mat.pos += heapVars.A33 * heapVars.M3Acc;
}


void AP_CINS::saturate_bias(Vector3F& bias_correction, const Vector3F& current_bias, const ftype& saturation, const ftype& dt) const
{
    // Ensure that no part of the bias exceeds the saturation limit
    if (abs(bias_correction.x) > 1E-8) {
        bias_correction.x *= MIN(1., (saturation - abs(current_bias.x)) / (dt * abs(bias_correction.x)));
    }
    if (abs(bias_correction.y) > 1E-8) {
        bias_correction.y *= MIN(1., (saturation - abs(current_bias.y)) / (dt * abs(bias_correction.y)));
    }
    if (abs(bias_correction.z) > 1E-8) {
        bias_correction.z *= MIN(1., (saturation - abs(current_bias.z)) / (dt * abs(bias_correction.z)));
    }
}


Vector3F computeRotationCorrection(const Vector3F& v1, const Vector3F& v2, const ftype& gain, const ftype& dt)
{
    // The correction is given by Omega = - skew(v1) * v2 * gain, where v2 = R v0 for some const. v0.
    // It is applied as dR/dt = skew(Omega) R.
    // The key point is that Omega changes R which changes Omega.
    // We use a first order expansion to capture this effect here.

    const Vector3F omega1 = - Matrix3F::skew_symmetric(v1) * v2 * gain;
    const Vector3F omega2 = - Matrix3F::skew_symmetric(v1) * Matrix3F::skew_symmetric(omega1) * v2 * gain;
    const Vector3F omega3 = - Matrix3F::skew_symmetric(v1) * Matrix3F::skew_symmetric(omega2) * v2 * gain
                            - Matrix3F::skew_symmetric(v1) * Matrix3F::skew_symmetric(omega1) * Matrix3F::skew_symmetric(omega1) * v2 * gain;
    return omega1 + omega2*dt + omega3*0.5*dt*dt;
}

bool AP_CINS::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    hgtVar = 0.;
    tasVar = 0.;
    if (state.variances.steps_to_initialise > 0) {
        velVar = 0.;
        posVar = 0.;
        magVar.zero();
        return false;
    }

    velVar = state.variances.velTest;
    posVar = state.variances.posTest;
    magVar.x = state.variances.magTest;
    magVar.y = state.variances.magTest;
    magVar.z = state.variances.magTest;

    return true;
}


#endif // AP_EXTERNAL_AHRS_CINS_ENABLED
