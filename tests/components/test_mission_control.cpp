#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/MissionControlImpl.h>

using namespace drone_mapper;
using ::testing::Return;

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

TEST(MissionControl, StopsAtMaxSteps) {
    MockDroneControl ctrl;
    EXPECT_CALL(ctrl, step())
        .WillRepeatedly(Return(types::DroneStepResult{types::DroneStepStatus::Continue, {}}));

    MissionControlImpl mc{makeMission(5), ctrl};
    const auto result = mc.runMission();
    EXPECT_EQ(result.status, types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 5u);
}

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
