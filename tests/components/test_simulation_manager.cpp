#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <drone_mapper/SimulationManager.h>

using namespace drone_mapper;
using ::testing::Return;
using ::testing::_;
using ::testing::Throw;

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

types::SimulationCompositionData makeComposition(int n_sims = 1,
                                                  int n_missions = 1,
                                                  int n_drones = 1,
                                                  int n_lidars = 1)
{
    types::SimulationCompositionData comp;
    comp.composition_file = "test.yaml";

    types::MissionConfigData m;
    m.max_steps = 10; m.gps_resolution = 10.0*cm;
    m.output_mapping_resolution_factor = 1.0;
    std::vector<types::MissionConfigData> mission_list;
    for (int i = 0; i < n_missions; ++i) mission_list.push_back(m);

    for (int i = 0; i < n_sims; ++i) {
        comp.simulation_mission_groups.emplace_back(types::SimulationConfigData{}, mission_list);
    }
    for (int i = 0; i < n_drones; ++i) comp.drones.push_back({});
    for (int i = 0; i < n_lidars; ++i) comp.lidars.push_back({});
    return comp;
}

types::SimulationResult makeResult(double score = 75.0,
                                    types::MissionRunStatus st = types::MissionRunStatus::Completed)
{
    types::SimulationResult sr;
    sr.mission_score = score;
    sr.mission_results.push_back({st, 5, {}});
    return sr;
}

} // namespace

// ---- Construction ----

TEST(SimulationManager, RejectsNullFactory) {
    EXPECT_THROW(SimulationManager{nullptr}, std::invalid_argument);
}

// ---- Happy path ----

TEST(SimulationManager, RunProducesReport) {
    auto factory = std::make_unique<MockFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .WillOnce([](const auto&, const auto&, const auto&, const auto&, const auto&) {
            auto run = std::make_unique<MockRun>();
            EXPECT_CALL(*run, run()).WillOnce(Return(makeResult(80.0)));
            return run;
        });

    SimulationManager manager{std::move(factory)};
    const auto report = manager.run(makeComposition(), std::filesystem::temp_directory_path());

    EXPECT_EQ(report.runs.size(), 1u);
    EXPECT_NEAR(report.runs[0].mission_score, 80.0, 1e-6);
    EXPECT_FALSE(report.generated_at_utc.empty());
}

TEST(SimulationManager, ReportContainsCompositionFile) {
    auto factory = std::make_unique<MockFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .WillOnce([](const auto&, const auto&, const auto&, const auto&, const auto&) {
            auto run = std::make_unique<MockRun>();
            EXPECT_CALL(*run, run()).WillOnce(Return(makeResult()));
            return run;
        });

    SimulationManager manager{std::move(factory)};
    auto comp = makeComposition();
    comp.composition_file = "/path/to/sim.yaml";
    const auto report = manager.run(comp, std::filesystem::temp_directory_path());

    EXPECT_EQ(report.composition_file, std::filesystem::path{"/path/to/sim.yaml"});
}

TEST(SimulationManager, ReportHasGeneratedTimestamp) {
    auto factory = std::make_unique<MockFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .WillOnce([](const auto&, const auto&, const auto&, const auto&, const auto&) {
            auto run = std::make_unique<MockRun>();
            EXPECT_CALL(*run, run()).WillOnce(Return(makeResult()));
            return run;
        });
    SimulationManager manager{std::move(factory)};
    const auto report = manager.run(makeComposition(), std::filesystem::temp_directory_path());
    // Timestamp should be non-empty and look like an ISO 8601 timestamp
    EXPECT_FALSE(report.generated_at_utc.empty());
    EXPECT_NE(std::string::npos, report.generated_at_utc.find('T'));
    EXPECT_NE(std::string::npos, report.generated_at_utc.find('Z'));
}

TEST(SimulationManager, CombinatorialRunCount) {
    // 2 sims × 2 missions × 1 drone × 1 lidar = 4 runs
    auto factory = std::make_unique<MockFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(4)
        .WillRepeatedly([](const auto&, const auto&, const auto&, const auto&, const auto&) {
            auto run = std::make_unique<MockRun>();
            EXPECT_CALL(*run, run()).WillOnce(Return(makeResult(60.0)));
            return run;
        });

    SimulationManager manager{std::move(factory)};
    const auto report = manager.run(makeComposition(2, 2, 1, 1),
                                    std::filesystem::temp_directory_path());

    EXPECT_EQ(report.runs.size(), 4u);
}

TEST(SimulationManager, AllDroneLidarCombinationsRun) {
    // 1 sim × 1 mission × 2 drones × 2 lidars = 4 runs
    auto factory = std::make_unique<MockFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(4)
        .WillRepeatedly([](const auto&, const auto&, const auto&, const auto&, const auto&) {
            auto run = std::make_unique<MockRun>();
            EXPECT_CALL(*run, run()).WillOnce(Return(makeResult(55.0)));
            return run;
        });

    SimulationManager manager{std::move(factory)};
    const auto report = manager.run(makeComposition(1, 1, 2, 2),
                                    std::filesystem::temp_directory_path());

    EXPECT_EQ(report.runs.size(), 4u);
}

// ---- Error handling ----

TEST(SimulationManager, FactoryThrowProducesErrorRun) {
    auto factory = std::make_unique<MockFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .WillOnce([](const auto&, const auto&, const auto&, const auto&, const auto&)
                      -> std::unique_ptr<ISimulationRun> {
            throw std::runtime_error("Map file not found");
        });

    SimulationManager manager{std::move(factory)};
    const auto report = manager.run(makeComposition(), std::filesystem::temp_directory_path());

    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_NEAR(report.runs[0].mission_score, -1.0, 1e-6);
    ASSERT_FALSE(report.runs[0].mission_results.empty());
    EXPECT_EQ(report.runs[0].mission_results[0].status, types::MissionRunStatus::Error);
}

TEST(SimulationManager, ErrorRunKeepsOtherRunsRunning) {
    // First run throws; second should still proceed.
    auto factory = std::make_unique<MockFactory>();
    int call_count = 0;
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .Times(2)
        .WillRepeatedly([&call_count](const auto&, const auto&, const auto&, const auto&, const auto&)
                            -> std::unique_ptr<ISimulationRun> {
            ++call_count;
            if (call_count == 1) throw std::runtime_error("first fails");
            auto run = std::make_unique<MockRun>();
            EXPECT_CALL(*run, run()).WillOnce(Return(makeResult(90.0)));
            return run;
        });

    SimulationManager manager{std::move(factory)};
    const auto report = manager.run(makeComposition(2), std::filesystem::temp_directory_path());

    ASSERT_EQ(report.runs.size(), 2u);
    EXPECT_NEAR(report.runs[0].mission_score, -1.0, 1e-6);
    EXPECT_NEAR(report.runs[1].mission_score, 90.0, 1e-6);
}

TEST(SimulationManager, MetricFieldIsSet) {
    auto factory = std::make_unique<MockFactory>();
    EXPECT_CALL(*factory, create(_, _, _, _, _))
        .WillOnce([](const auto&, const auto&, const auto&, const auto&, const auto&) {
            auto run = std::make_unique<MockRun>();
            EXPECT_CALL(*run, run()).WillOnce(Return(makeResult()));
            return run;
        });
    SimulationManager manager{std::move(factory)};
    const auto report = manager.run(makeComposition(), std::filesystem::temp_directory_path());
    EXPECT_EQ(report.metric, "output_map_accuracy");
    EXPECT_EQ(report.error_score, -1);
}
