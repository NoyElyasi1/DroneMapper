#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/SimulationManager.h>

using namespace drone_mapper;
using ::testing::Return;
using ::testing::_;

class MockRun : public ISimulationRun {
public:
    MOCK_METHOD(types::SimulationResult, run, (), (override));
};

class MockFactory : public ISimulationRunFactory {
public:
    MOCK_METHOD(std::unique_ptr<ISimulationRun>, create,
                (const types::SimulationConfigData&,
                 const types::MissionConfigData&,
                 const types::DroneConfigData&,
                 const types::LidarConfigData&,
                 const std::filesystem::path&),
                (override));
};

namespace {

types::SimulationCompositionData makeComposition() {
    types::SimulationCompositionData comp;
    comp.composition_file = "test.yaml";
    comp.simulations.push_back({});
    types::MissionConfigData m; m.max_steps=10; m.gps_resolution=10.0*cm; m.output_mapping_resolution_factor=1.0;
    comp.missions.push_back(m);
    comp.drones.push_back({});
    comp.lidars.push_back({});
    return comp;
}

} // namespace

TEST(SimulationManager, RejectsNullFactory) {
    EXPECT_THROW(SimulationManager{nullptr}, std::invalid_argument);
}

TEST(SimulationManager, RunProducesReport) {
    auto factory = std::make_unique<MockFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .WillOnce([](const auto&, const auto&, const auto&, const auto&, const auto&) {
            auto run = std::make_unique<MockRun>();
            types::SimulationResult sr;
            sr.mission_score = 80.0;
            sr.mission_results.push_back({types::MissionRunStatus::Completed, 5, {}});
            EXPECT_CALL(*run, run()).WillOnce(Return(sr));
            return run;
        });

    SimulationManager manager{std::move(factory)};
    const auto report = manager.run(makeComposition(), std::filesystem::temp_directory_path());

    EXPECT_EQ(report.runs.size(), 1u);
    EXPECT_NEAR(report.runs[0].mission_score, 80.0, 1e-6);
    EXPECT_FALSE(report.generated_at_utc.empty());
}
