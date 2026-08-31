/*
  CINS state estimator for AP_AHRS
 */

#pragma once

#include <AP_ExternalAHRS/AP_ExternalAHRS_config.h>

#if AP_EXTERNAL_AHRS_CINS_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_Math/LieGroups.h>
#include <AP_Common/Location.h>

#ifndef CINS_MAX_HISTORY
#define CINS_MAX_HISTORY 100
#endif

// Store one delay-history increment for every N IMU updates. Board targets may
// use this to trade sub-step delay resolution for heap headroom while retaining
// the same total history duration.
#ifndef CINS_HISTORY_DECIMATION
#define CINS_HISTORY_DECIMATION 1
#endif

// A 32-sample history covers 0.32 seconds at a 100 Hz barometer rate. Samples
// are stored as floats so this experimental feature has a fixed 256-byte cost
// on both float and double CINS builds.
#ifndef CINS_BARO_HISTORY_SIZE
#define CINS_BARO_HISTORY_SIZE 32
#endif

static_assert(CINS_HISTORY_DECIMATION > 0, "CINS history decimation must be positive");
static_assert(CINS_BARO_HISTORY_SIZE > 0, "CINS barometer history must not be empty");
static_assert(CINS_BARO_HISTORY_SIZE <= 255, "CINS barometer history count must fit in uint8_t");

class AP_CINS
{
public:
    AP_CINS();
    void init(void);
    void update();

    struct {
        AP_Float gpspos_att;
        AP_Float gpsvel_att;
        AP_Float gps_lag;
        AP_Float gps_pos;
        AP_Float gps_vel;
        AP_Float mag_att;
        AP_Float Q11;
        AP_Float Q22;
        AP_Float gps_pos_gyr_bias;
        AP_Float gps_vel_gyr_bias;
        AP_Float mag_gyr_bias;
        AP_Float sat_gyr_bias;
        AP_Float gps_pos_acc_bias;
        AP_Float gps_vel_acc_bias;
        AP_Float mag_acc_bias;
        AP_Float sat_acc_bias;
        AP_Float variance_lowpass;
        AP_Float variance_scale;
        AP_Int8 gps_correction_mode;
    } gains;

    AP_Int16 detail_log_rate;
    AP_Int8 memory_log_rate;
    struct {
        AP_Int8 mode;
        AP_Float lag;
        AP_Float max_age;
        AP_Float gate;
    } baro_config;

    Vector3f get_accel() const
    {
        return state.accel.tofloat();
    }
    Vector3f get_gyro() const
    {
        return state.gyro.tofloat();
    }
    Quaternion get_quat() const
    {
        QuaternionF quat;
        quat.from_rotation_matrix(state.XHat.rot());
        return quat.tofloat();
    }
    Location get_location() const
    {
        Location loc = state.origin;
        loc.offset(state.XHat.pos().x, state.XHat.pos().y);
        loc.alt -= state.XHat.pos().z * 100;
        return loc;
    }
    Vector3f get_velocity() const
    {
        return state.XHat.vel().tofloat();
    }
    bool healthy(void) const
    {
        return state.have_origin && done_yaw_init && !delayer.buffer_failed &&
               !delayer.history_released_for_compass_calibration;
    }
    bool initialised(void) const
    {
        return healthy();
    }
    bool delay_buffer_failed(void) const
    {
        return delayer.buffer_failed;
    }
    // Release delay history before the compass calibrator allocates its sample
    // buffer and thread. Compass calibration requires a reboot, so history is
    // rebuilt only on the next boot.
    void prepare_for_compass_calibration(void);
    bool get_origin(Location &loc)
    {
        loc = state.origin;
        return state.have_origin;
    }

    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const;

    static const struct AP_Param::GroupInfo var_info[];

private:
    void update_imu(const Vector3F &gyro_rads, const Vector3F &accel_mss, const ftype dt);
    void update_gps(const Vector3F &pos, const Vector3F &vel, const ftype gps_dt);
    ftype update_vector_measurement_cts(const Vector3F &measurement, const Vector3F& reference, const Vector2F &ref_base, const ftype& gain_R, const ftype& gain_V, const ftype& gain_gyr_bias, const ftype& gain_acc_bias, const ftype dt);
    void update_attitude_from_compass();
    bool init_yaw(void);
    bool get_compass_yaw(ftype &yaw_rad, ftype &dt);
    bool get_compass_vector(Vector3F &mag_vec, Vector3F &mag_ref, ftype &dt);
    void update_baro_history();
    void reset_baro_composite();
    bool apply_baro_composite(Vector3F &gps_pos, uint32_t gps_timestamp_ms);

    void compute_bias_update_imu(const SIM23& Gamma);
    void saturate_bias(Vector3F& bias_correction, const Vector3F& current_bias, const ftype& saturation, const ftype& dt) const;
    void write_memory_diagnostic(uint64_t time_us);

    struct {
        Vector3F accel;
        Vector3F gyro;
        Location origin;
        bool have_origin;

        //XHat and ZHat for CINS
        Gal3F XHat; // State estimate containing attitude, velocity, position
        SIM23 ZHat; // Auxiliary state for estimator maths

        // Gyro Bias
        Vector3F gyr_bias;
        Vector3F gyr_bias_correction;
        struct {
            Matrix3F rot;
            Matrix3F vel;
            Matrix3F pos;
        } gyr_bias_gain_mat;

        // Accel Bias
        Vector3F acc_bias;
        Vector3F acc_bias_correction;
        struct {
            Matrix3F rot;
            Matrix3F vel;
            Matrix3F pos;
        } acc_bias_gain_mat;

        // Variance States
        struct {
            uint8_t steps_to_initialise;
            ftype velVar;
            ftype posVar;
            ftype magVar;
            ftype velTest;
            ftype posTest;
            ftype magTest;
        } variances;

    } state;

    // Variables to be kept on the heap
    struct {
        Matrix3F A11;
        Matrix3F A12;
        Matrix3F A13;
        Matrix3F A21;
        Matrix3F A22;
        Matrix3F A23;
        Matrix3F A31;
        Matrix3F A32;
        Matrix3F A33;
        Matrix3F M1Gyr;
        Matrix3F M2Gyr;
        Matrix3F M3Gyr;
        Matrix3F M1Acc;
        Matrix3F M2Acc;
        Matrix3F M3Acc;
        Matrix3F C11;
        Matrix3F C12;
        Matrix3F C13;
        Matrix3F K11;
        Matrix3F K21;
        Matrix3F K31;

        SIM23 Delta_SIM23;
        SIM23 ZInv;
        Matrix3F I3;
        Vector3F zero_vector;
        Vector3F gravity_vector;

        GL2 S_Gamma;
        Vector3F W_Gamma_1;
        Vector3F W_Gamma_2;
        SIM23 GammaInv;

        Vector3F Omega_Delta;
        Vector3F W_Delta1;
        Vector3F W_Delta2;
    } heapVars;

    // Delay variables required to handle delayed measurements.

    struct stamped_Gal3F {
        // The history is only retained for the bounded CINS GPS lag interval,
        // so a wrapping microsecond timestamp is sufficient. Keeping this at
        // 32 bits avoids padding every history entry to an 8-byte boundary.
        uint32_t timestamp_us;
        QuaternionF rot;
        Vector3F pos;
        Vector3F vel;
        ftype tau;
    };

    struct {
        double internal_time;
        ObjectBuffer<stamped_Gal3F> stamped_inputs{CINS_MAX_HISTORY};
        Vector3F gps_vel;
        Vector3F gps_pos;
        Gal3F YR; // Primary right delay matrix
        Gal3F YR_stepper; // right delay matrix to apply at new GPS measurement.
        Gal3F history_increment; // accumulated increment awaiting history storage
        uint16_t history_increment_count;
        ftype gps_correction_dt;
        bool gps_correction_pending;
        bool buffer_failed;
        bool history_released_for_compass_calibration;
    } delayer;

    struct baro_sample {
        uint32_t timestamp_ms;
        float down;
    };
    static_assert(sizeof(baro_sample) == 8, "CINS barometer sample size changed");

    struct {
        baro_sample history[CINS_BARO_HISTORY_SIZE];
        ftype offset;
        ftype offset_sum;
        uint32_t last_update_ms;
        uint8_t next;
        uint8_t count;
        uint8_t offset_sample_count;
        uint8_t primary;
        bool healthy;
        bool offset_valid;
        bool last_sample_accepted;
    } baro_state;

    uint32_t last_gps_update_ms;
    uint32_t last_detail_log_ms;
    uint32_t last_memory_log_ms;
    bool done_yaw_init;
    uint32_t last_mag_us;
};

#endif // AP_EXTERNAL_AHRS_CINS_ENABLED
