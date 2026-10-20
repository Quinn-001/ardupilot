#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include "../AP_CINS_BaroDatum.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static CINSBaroDatum::Config config()
{
    return {0.2f, 0.08f, 0.1f, 10.0f, 3.0f, 0.5f, 2.0f, 0.5f, 1, 2, 1, 0, 0, 0, 1};
}

static uint32_t settle(CINSBaroDatum &datum, uint32_t start = 1000, float down = 21.0f)
{
    uint32_t t = start;
    for (unsigned i = 0; i < 16; ++i, t += 200) {
        datum.pair(t, 0, down, config(), true);
        datum.prediction(t, 0);
    }
    return t - 200;
}

TEST(BaroDatum, GroundAndClimbShareReference)
{
    CINSBaroDatum datum;
    const uint32_t t = settle(datum);
    ASSERT_TRUE(datum.prepare(t));
    const float offset = datum.offset();
    EXPECT_FLOAT_EQ(offset, -21);
    datum.commit(t);
    EXPECT_TRUE(datum.frozen_valid());
    EXPECT_FLOAT_EQ(-4.0f - offset, 17.0f); // Origin180, home159, altitude163.
    datum.pair(t + 200, -4, 17, config(), true);
    EXPECT_FLOAT_EQ(datum.offset(), offset);
}

TEST(BaroDatum, SettlingIsTrackedBeforeFreeze)
{
    CINSBaroDatum datum;
    for (uint32_t t = 1000; t < 10000; t += 200) {
        datum.pair(t, 0, (t - 1000) * .003f, config(), true);
        datum.prediction(t, 0);
        EXPECT_FALSE(datum.ready(t));
    }
    for (uint32_t t = 10000; t < 20000; t += 200) {
        datum.pair(t, 0, 30, config(), true);
        datum.prediction(t, 0);
    }
    EXPECT_TRUE(datum.ready(19800));
    EXPECT_NEAR(datum.offset(), -30, .001);
}

TEST(BaroDatum, DuplicatePacketsDoNotCompleteDwell)
{
    CINSBaroDatum datum;
    for (unsigned i = 0; i < 1000; ++i) {
        datum.pair(1000, 0, 21, config(), true);
        datum.prediction(1000, 0);
    }
    EXPECT_EQ(datum.count(), 1);
    EXPECT_FALSE(datum.prepare(1000));
}

TEST(BaroDatum, StaleOrUnqualifiedInputCannotArm)
{
    CINSBaroDatum datum;
    uint32_t t = settle(datum);
    EXPECT_FALSE(datum.prepare(t + 501));
    datum.pair(t + 200, 0, 21, config(), false);
    datum.prediction(t + 200, 0);
    EXPECT_FALSE(datum.ready(t + 200));
    EXPECT_FALSE(datum.prepare(t + 200));
}

TEST(BaroDatum, ObserverMustConvergeAndPairMustBeConsumed)
{
    CINSBaroDatum datum;
    uint32_t t = settle(datum);
    datum.pair(t + 200, 0, 21, config(), true);
    EXPECT_FALSE(datum.prepare(t + 200));
    datum.prediction(t + 200, 1.01f);
    EXPECT_FALSE(datum.prepare(t + 200));
}

TEST(BaroDatum, PreparationChangesGenerationWithoutChangingOffset)
{
    CINSBaroDatum datum;
    uint32_t t = settle(datum);
    const auto generation = datum.generation();
    const float before = datum.offset();
    ASSERT_TRUE(datum.prepare(t));
    EXPECT_NE(datum.generation(), generation);
    EXPECT_FLOAT_EQ(datum.offset(), before);
    datum.reset(); // Arming rejected by a later check.
    EXPECT_EQ(datum.phase(), CINSBaroDatum::Phase::Tracking);
    EXPECT_FALSE(datum.ready(t));
    t = settle(datum, t + 200, 31);
    EXPECT_TRUE(datum.prepare(t));
    EXPECT_FLOAT_EQ(datum.offset(), -31);
}

TEST(BaroDatum, ArmedCalibrationOrConfigurationChangeDoesNotRelearn)
{
    CINSBaroDatum datum;
    const uint32_t t = settle(datum);
    ASSERT_TRUE(datum.prepare(t));
    datum.commit(t);
    const float frozen = datum.offset();
    auto cfg = config();
    ++cfg.calibration;
    datum.pair(t + 200, 5, 30, cfg, true);
    EXPECT_FALSE(datum.frozen_valid());
    EXPECT_FLOAT_EQ(datum.offset(), frozen);
    datum.pair(t + 400, 10, 40, config(), true);
    EXPECT_FALSE(datum.frozen_valid());
    EXPECT_FLOAT_EQ(datum.offset(), frozen);
}

TEST(BaroDatum, UnpreparedArmCannotLearnInFlight)
{
    CINSBaroDatum datum;
    datum.commit(1000);
    for (uint32_t t = 1200; t < 10000; t += 200) {
        datum.pair(t, 0, 21, config(), true);
        datum.prediction(t, 0);
    }
    EXPECT_FALSE(datum.frozen_valid());
    EXPECT_FALSE(datum.has_offset());
}

TEST(BaroDatum, DwellSurvivesClockWrap)
{
    CINSBaroDatum datum;
    const uint32_t t = settle(datum, UINT32_MAX - 1500);
    EXPECT_TRUE(datum.prepare(t));
}

TEST(BaroDatum, TightConsistencyGateRequiresTighterAlignment)
{
    CINSBaroDatum datum;
    auto cfg = config();
    cfg.consistency_gate = .1f;
    for (uint32_t t = 1000; t < 10000; t += 200) {
        datum.pair(t, 0, (t - 1000) * .0001f, cfg, true);
        datum.prediction(t, 0);
    }
    EXPECT_FALSE(datum.prepare(9800));
}

TEST(BaroDatum, PrimaryAndLagChangesInvalidateReadiness)
{
    CINSBaroDatum datum;
    uint32_t t = settle(datum);
    auto cfg = config();
    cfg.primary = 1;
    datum.pair(t + 200, 0, 21, cfg, true);
    datum.prediction(t + 200, 0);
    EXPECT_FALSE(datum.prepare(t + 200));
    datum.reset();
    t = settle(datum, t + 400);
    ASSERT_TRUE(datum.prepare(t));
    datum.commit(t);
    cfg = config();
    cfg.baro_lag = .1f;
    datum.check_config(cfg);
    EXPECT_FALSE(datum.frozen_valid());
    EXPECT_FLOAT_EQ(datum.offset(), -21);
}

TEST(BaroDatum, LaterGpsQualityDoesNotRelearnFrozenOffset)
{
    CINSBaroDatum datum;
    const uint32_t t = settle(datum);
    ASSERT_TRUE(datum.prepare(t));
    datum.commit(t);
    datum.pair(t + 200, 0, 40, config(), false);
    EXPECT_TRUE(datum.frozen_valid());
    EXPECT_FLOAT_EQ(datum.offset(), -21);
}

TEST(BaroDatum, SourcePolicyChangeInvalidatesPreparedDatum)
{
    CINSBaroDatum datum;
    const uint32_t t = settle(datum);
    ASSERT_TRUE(datum.prepare(t));
    auto cfg = config();
    cfg.source_policy = 1;
    EXPECT_FALSE(cfg == config());
    datum.check_config(cfg);
    EXPECT_FALSE(datum.ready(t));
    EXPECT_FLOAT_EQ(datum.offset(), -21);
}

TEST(BaroDatum, ObserverOneMetreToleranceKeepsTwoSecondDwell)
{
    for (const float innovation : {-1.0f, 1.0f}) {
        CINSBaroDatum datum;
        for (uint32_t t = 1000; t <= 4000; t += 200) {
            datum.pair(t, 0, 21, config(), true);
            datum.prediction(t, innovation);
            if (t < 3800) {
                EXPECT_FALSE(datum.ready(t));
            }
        }
        ASSERT_TRUE(datum.prepare(4000));
        EXPECT_FLOAT_EQ(datum.offset(), -21);
        datum.commit(4000);
        EXPECT_TRUE(datum.frozen_valid());
        datum.pair(4200, 0, 30, config(), true);
        EXPECT_FLOAT_EQ(datum.offset(), -21);
    }
}

TEST(BaroDatum, ObserverAboveOneMetreRestartsDwell)
{
    CINSBaroDatum datum;
    const uint32_t t = settle(datum);
    ASSERT_TRUE(datum.ready(t));
    datum.pair(t + 200, 0, 21, config(), true);
    datum.prediction(t + 200, 1.01f);
    EXPECT_EQ(datum.reason(t + 200), CINSBaroDatum::Reason::Prediction);
    EXPECT_FALSE(datum.prepare(t + 200));
    for (uint32_t elapsed = 400; elapsed <= 2400; elapsed += 200) {
        datum.pair(t + elapsed, 0, 21, config(), true);
        datum.prediction(t + elapsed, .75f);
        if (elapsed < 2400) {
            EXPECT_FALSE(datum.ready(t + elapsed));
        }
    }
    EXPECT_TRUE(datum.prepare(t + 2400));
}

TEST(BaroDatum, DatumQuarterMetreLimitPreservedDuringPreparation)
{
    CINSBaroDatum datum;
    const uint32_t t = settle(datum);
    ASSERT_TRUE(datum.prepare(t));
    datum.pair(t + 200, .25f, 21, config(), true);
    datum.prediction(t + 200, .75f);
    EXPECT_TRUE(datum.ready(t + 200));
    EXPECT_FLOAT_EQ(datum.offset(), -21);
    datum.pair(t + 400, .251f, 21, config(), true);
    datum.prediction(t + 400, .75f);
    EXPECT_EQ(datum.reason(t + 400), CINSBaroDatum::Reason::Pair);
    EXPECT_FALSE(datum.ready(t + 400));
    EXPECT_FLOAT_EQ(datum.offset(), -21);
}

AP_GTEST_MAIN()
