#include <AP_gtest.h>
#include <AP_HAL/AP_HAL.h>
#include "../AP_CINS_BaroAdmission.h"
#include "../AP_CINS_BaroBias.h"
#include <limits>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(BaroAdmission, SymmetricGroundEffectGateAndBoundaries)
{
    EXPECT_TRUE(CINSBaroAdmission::evaluate(true, 0.5f, true, 3, 0.5f).accepted);
    EXPECT_TRUE(CINSBaroAdmission::evaluate(true, -0.5f, true, 3, 0.5f).accepted);
    EXPECT_TRUE(CINSBaroAdmission::evaluate(true, 0.0f, true, 3, 0.5f).accepted);
    EXPECT_TRUE(CINSBaroAdmission::evaluate(true, -0.6f, false, 3, 0.5f).accepted);
    const auto rise = CINSBaroAdmission::evaluate(true, -0.6f, true, 3, 0.5f);
    EXPECT_FALSE(rise.accepted);
    EXPECT_TRUE(rise.ground_effect_bad);
    EXPECT_FALSE(rise.innovation_bad);
    EXPECT_TRUE(CINSBaroAdmission::evaluate(true, 0.6f, false, 3, 0.5f).accepted);
    const auto wash = CINSBaroAdmission::evaluate(true, 0.6f, true, 3, 0.5f);
    EXPECT_FALSE(wash.accepted);
    EXPECT_TRUE(wash.ground_effect_bad);
    EXPECT_FALSE(wash.innovation_bad);
}

TEST(BaroAdmission, GenericGateAndInvalidCandidates)
{
    EXPECT_TRUE(CINSBaroAdmission::evaluate(true, -3.0, false, 3, 0.5f).accepted);
    EXPECT_FALSE(CINSBaroAdmission::evaluate(true, -3.01, false, 3, 0.5f).accepted);
    EXPECT_FALSE(CINSBaroAdmission::evaluate(true, 3.01, false, 3, 0.5f).accepted);
    EXPECT_FALSE(CINSBaroAdmission::evaluate(false, 0.0f, false, 3, 0.5f).accepted);
    EXPECT_FALSE(CINSBaroAdmission::evaluate(true, std::numeric_limits<float>::quiet_NaN(),
                                           false, 3, 0.5f).accepted);
    EXPECT_FALSE(CINSBaroAdmission::evaluate(true, std::numeric_limits<double>::infinity(),
                                           true, 3, 0.5f).accepted);
}

TEST(BaroAdmission, AcceptedFlaggedPacketsLearnAndRejectedPacketsHold)
{
    CINSBaroBias bias;
    const CINSBaroBias::Config cfg{60, 0.05f, 5, 1, 3};
    const bool accepted = CINSBaroAdmission::evaluate(true, 0.1f, true, 3, 0.5f).accepted;
    for (uint32_t t = 100; t <= 4100; t += 200) {
        bias.update(cfg, t, -1, 0, 0, .4025f, accepted);
    }
    EXPECT_LT(bias.bias(), 0);
    const float before = bias.bias();
    const bool rejected = CINSBaroAdmission::evaluate(true, 0.6f, true, 3, 0.5f).accepted;
    for (uint32_t t = 4300; t <= 6100; t += 200) {
        bias.update(cfg, t, -1, 0, 0, .4025f, rejected);
    }
    EXPECT_FLOAT_EQ(bias.bias(), before);
    EXPECT_EQ(bias.update(cfg, 6300, -1, 0, 0, .4025f, accepted), CINSBaroBias::Result::Baseline);
    EXPECT_FLOAT_EQ(bias.bias(), before);
    for (uint32_t t = 6500; t < 9300; t += 200) {
        EXPECT_EQ(bias.update(cfg, t, -1, 0, 0, .4025f, accepted), CINSBaroBias::Result::Qualifying);
    }
    EXPECT_EQ(bias.update(cfg, 9300, -1, 0, 0, .4025f, accepted), CINSBaroBias::Result::Updated);
    EXPECT_LT(bias.bias(), before);
}


TEST(BaroAdmission, RejectsFlybeeFalseClimbWithoutChangingGeneralGate)
{
    const auto launch = CINSBaroAdmission::evaluate(true, -3.6, true, 5, 0.5f);
    EXPECT_FALSE(launch.accepted);
    EXPECT_TRUE(launch.ground_effect_bad);
    EXPECT_FALSE(launch.innovation_bad);
    EXPECT_TRUE(CINSBaroAdmission::evaluate(true, -3.6, false, 5, 0.5f).accepted);
    for (double sign : {-1.0, 1.0}) {
        EXPECT_TRUE(CINSBaroAdmission::evaluate(true, sign * 0.5, true, 5, 0.5f).accepted);
        EXPECT_FALSE(CINSBaroAdmission::evaluate(true, sign * 0.500001, true, 5, 0.5f).accepted);
        const auto general = CINSBaroAdmission::evaluate(true, sign * 0.3, true, 0.2f, 0.5f);
        EXPECT_FALSE(general.accepted);
        EXPECT_TRUE(general.innovation_bad);
        EXPECT_FALSE(general.ground_effect_bad);
    }
}

TEST(BaroAdmission, NegativeGroundEffectRejectionHoldsLearningAndRestartsDwell)
{
    CINSBaroBias bias;
    const CINSBaroBias::Config cfg{60, 0.05f, 5, 1, 3};
    for (uint32_t t = 100; t <= 4100; t += 200) {
        bias.update(cfg, t, -1, 0, 0, .4025f, true);
    }
    ASSERT_LT(bias.bias(), 0);
    const float before = bias.bias();
    const auto launch = CINSBaroAdmission::evaluate(true, -1.2f, true, 5, 0.5f);
    ASSERT_FALSE(launch.accepted);
    ASSERT_TRUE(launch.ground_effect_bad);
    for (uint32_t t = 4300; t <= 6100; t += 200) {
        bias.update(cfg, t, -1, 0, 0, .4025f, launch.accepted);
    }
    EXPECT_FLOAT_EQ(bias.bias(), before);
    EXPECT_EQ(bias.update(cfg, 6300, -1, 0, 0, .4025f, true), CINSBaroBias::Result::Baseline);
    for (uint32_t t = 6500; t < 9300; t += 200) {
        EXPECT_EQ(bias.update(cfg, t, -1, 0, 0, .4025f, true), CINSBaroBias::Result::Qualifying);
        EXPECT_FLOAT_EQ(bias.bias(), before);
    }
    EXPECT_EQ(bias.update(cfg, 9300, -1, 0, 0, .4025f, true), CINSBaroBias::Result::Updated);
    EXPECT_LT(bias.bias(), before);
}

AP_GTEST_MAIN()
