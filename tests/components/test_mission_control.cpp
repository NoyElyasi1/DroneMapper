#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/MissionControlImpl.h>

using namespace drone_mapper;
using ::testing::Return;
using ::testing::InSequence;

class MockDroneControl : public IDroneControl {
public:
    MOCK_METHOD(types::DroneStepResult, step, (), (override));
    MOCK_METHOD(types::DroneState, state, (), (const, override));
};

namespace {
types::MissionConfigData makeMission(std::size_t steps = 10) {
    types::MissionConfigData m;
    m.max_steps = steps;
    m.gps_resolution = 10.0 * cm;
    m.output_mapping_resolution_factor = 1.0;
    return m;
}
} // namespace

// ---- Completion ----

TEST(MissionControl, CompletesWhenDroneReturnsCompleted) {
    MockDroneControl ctrl;
    EXPECT_CALL(ctrl, step())
        .WillOnce(Return(types::DroneStepResult{types::DroneStepStatus::Completed, {}}));

    MissionControlImpl mc{makeMission(), ctrl};
    const auto result = mc.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 1u);
    EXPECT_TRUE(result.errors.empty());
}

TEST(MissionControl, CompletesAfterMultipleSteps) {
    MockDroneControl ctrl;
    {
        InSequence seq;
        EXPECT_CALL(ctrl, step()).Times(3)
            .WillRepeatedly(Return(types::DroneStepResult{types::DroneStepStatus::Continue, {}}));
        EXPECT_CALL(ctrl, step())
            .WillOnce(Return(types::DroneStepResult{types::DroneStepStatus::Completed, {}}));
    }
    MissionControlImpl mc{makeMission(20), ctrl};
    const auto result = mc.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 4u);
}

// ---- MaxSteps ----

TEST(MissionControl, StopsAtMaxSteps) {
    MockDroneControl ctrl;
    EXPECT_CALL(ctrl, step())
        .WillRepeatedly(Return(types::DroneStepResult{types::DroneStepStatus::Continue, {}}));

    MissionControlImpl mc{makeMission(5), ctrl};
    const auto result = mc.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 5u);
}

TEST(MissionControl, ZeroMaxStepsNeverCallsStep) {
    MockDroneControl ctrl;
    EXPECT_CALL(ctrl, step()).Times(0);

    MissionControlImpl mc{makeMission(0), ctrl};
    const auto result = mc.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 0u);
}

TEST(MissionControl, ExactlyMaxStepsExecuted) {
    MockDroneControl ctrl;
    EXPECT_CALL(ctrl, step()).Times(7)
        .WillRepeatedly(Return(types::DroneStepResult{types::DroneStepStatus::Continue, {}}));

    MissionControlImpl mc{makeMission(7), ctrl};
    const auto result = mc.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 7u);
}

// ---- Error ----

TEST(MissionControl, ErrorPropagated) {
    MockDroneControl ctrl;
    EXPECT_CALL(ctrl, step())
        .WillOnce(Return(types::DroneStepResult{types::DroneStepStatus::Error, "bang"}));

    MissionControlImpl mc{makeMission(), ctrl};
    const auto result = mc.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::Error);
    ASSERT_FALSE(result.errors.empty());
    EXPECT_EQ(result.errors[0].code, "DRONE_ERROR");
}

TEST(MissionControl, ErrorMessagePreservedInErrorRef) {
    MockDroneControl ctrl;
    EXPECT_CALL(ctrl, step())
        .WillOnce(Return(types::DroneStepResult{types::DroneStepStatus::Error, "motors failed"}));

    MissionControlImpl mc{makeMission(), ctrl};
    const auto result = mc.runMission();
    ASSERT_FALSE(result.errors.empty());
    EXPECT_EQ(result.errors[0].message, "motors failed");
}

TEST(MissionControl, ErrorOnFirstStepHasStepCount1) {
    MockDroneControl ctrl;
    EXPECT_CALL(ctrl, step())
        .WillOnce(Return(types::DroneStepResult{types::DroneStepStatus::Error, ""}));

    MissionControlImpl mc{makeMission(100), ctrl};
    const auto result = mc.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::Error);
    EXPECT_EQ(result.steps, 1u);
}

TEST(MissionControl, StopsAfterErrorDoesNotContinue) {
    MockDroneControl ctrl;
    // Error on step 2; step() should not be called more than 2 times.
    EXPECT_CALL(ctrl, step())
        .WillOnce(Return(types::DroneStepResult{types::DroneStepStatus::Continue, {}}))
        .WillOnce(Return(types::DroneStepResult{types::DroneStepStatus::Error, "fail"}));

    MissionControlImpl mc{makeMission(100), ctrl};
    const auto result = mc.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::Error);
    EXPECT_EQ(result.steps, 2u);
}
