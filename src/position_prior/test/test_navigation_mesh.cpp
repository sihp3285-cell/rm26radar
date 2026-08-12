#include "position_prior/blind_zone_prior.hpp"
#include "position_prior/navigation_mesh.hpp"
#include "position_prior/prior_gate.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>

namespace position_prior {
namespace {

std::string generated_path(const std::string& filename) {
    return (std::filesystem::path(POSITION_PRIOR_GENERATED_DIR) / filename).string();
}

class NavigationMeshTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        mesh.load(generated_path("RB2026_navgrid_v1.json"));
    }

    static NavigationMesh mesh;
};

NavigationMesh NavigationMeshTest::mesh;

TEST_F(NavigationMeshTest, MatchesGeneratedGoldenShortestPaths) {
    const YAML::Node golden =
        YAML::LoadFile(generated_path("RB2026_navgrid_golden_v1.json"));
    const std::string role = golden["profile"].as<std::string>();
    for (const auto& test_case : golden["cases"]) {
        const Point2d start{
            test_case["start"][0].as<double>(),
            test_case["start"][1].as<double>()};
        const Point2d goal{
            test_case["goal"][0].as<double>(),
            test_case["goal"][1].as<double>()};
        const double snap_radius =
            test_case["maximum_snap_distance_m"].as<double>();
        const auto routes = mesh.route_map(role, start, snap_radius);
        NavigationSnap goal_snap;
        const double distance =
            mesh.path_distance(routes, goal, snap_radius, &goal_snap);
        const bool expected_reachable =
            test_case["expected"]["reachable"].as<bool>();

        SCOPED_TRACE(test_case["name"].as<std::string>());
        EXPECT_EQ(std::isfinite(distance), expected_reachable);
        if (expected_reachable) {
            EXPECT_NEAR(
                distance,
                test_case["expected"]["path_distance_m"].as<double>(),
                golden["distance_tolerance_m"].as<double>());
            EXPECT_EQ(routes.component_id,
                test_case["expected"]["component_id"].as<int>());
        }
    }
}

TEST_F(NavigationMeshTest, MeshPathRejectsStraightLineShortcut) {
    const auto routes = mesh.route_map("hero", {2.0, 2.0}, 0.6);
    ASSERT_TRUE(routes.valid);
    const double path_distance =
        mesh.path_distance(routes, {26.0, 13.0}, 0.6);
    EXPECT_NEAR(path_distance, 38.681832585697954, 1e-6);
    EXPECT_GT(path_distance, std::hypot(24.0, 11.0) + 12.0);
}

TEST_F(NavigationMeshTest, CommonAndEngineerOnlyBlindZonesAreRoleAware) {
    BlindZoneBiasConfig blind_config;
    blind_config.trigger_distance_m = 1.2;
    blind_config.maximum_probability_mass = 0.65;
    BlindZonePrior blind(blind_config);
    blind.load(
        {generated_path("home.yaml"), generated_path("gully.yaml")},
        generated_path("engineer.yaml"));
    ASSERT_TRUE(blind.loaded());
    ASSERT_EQ(blind.zone_count(), 4u);

    PriorDistribution base;
    base.valid = true;
    base.role = "hero";
    base.horizon_seconds = 2;
    base.local_weight = 1.0;
    base.candidates = {{1, {14.0, 6.5}, 1.0}};

    // 工程专用盲区中心附近（中心翻转后的敌侧多边形）：英雄不注入，工程会注入。
    const Point2d last{12.8, 8.2};
    const auto hero_routes = mesh.route_map("hero", last, 0.6);
    auto hero_distribution = base;
    const auto hero_bias = blind.apply(
        "hero", last, last, 4.0, mesh, hero_routes, hero_distribution);
    EXPECT_FALSE(hero_bias.applied);

    const auto engineer_routes = mesh.route_map("engineer", last, 0.6);
    ASSERT_TRUE(engineer_routes.valid);
    auto engineer_distribution = base;
    engineer_distribution.role = "engineer";
    const auto engineer_bias = blind.apply(
        "engineer", last, last, 4.0,
        mesh, engineer_routes, engineer_distribution);
    ASSERT_TRUE(engineer_bias.applied);
    EXPECT_GT(engineer_bias.injected_candidate_count, 0u);
    EXPECT_GT(engineer_bias.injected_probability_mass, 0.0);
    EXPECT_TRUE(std::any_of(
        engineer_distribution.candidates.begin(),
        engineer_distribution.candidates.end(),
        [](const PriorCandidate& candidate) {
            return candidate.from_blind_zone;
        }));
}

TEST_F(NavigationMeshTest, FlippedViewUsesXMirroredBlindZones) {
    BlindZoneBiasConfig blind_config;
    blind_config.trigger_distance_m = 1.2;
    blind_config.maximum_probability_mass = 0.65;
    BlindZonePrior blind(blind_config);
    blind.load(
        {generated_path("home.yaml"), generated_path("gully.yaml")},
        generated_path("engineer.yaml"));
    blind.set_flipped_view(true);

    PriorDistribution base;
    base.valid = true;
    base.role = "hero";
    base.horizon_seconds = 2;
    base.local_weight = 1.0;
    base.candidates = {{1, {14.0, 6.5}, 1.0}};

    // 翻转视角下工程盲区按 x=14 轴左右翻转，锚点取翻转后多边形中心附近；
    // 英雄仍不注入（非工程区域），工程会注入。
    const Point2d last{15.0, 8.1};
    const auto hero_routes = mesh.route_map("hero", last, 0.6);
    auto hero_distribution = base;
    const auto hero_bias = blind.apply(
        "hero", last, last, 4.0, mesh, hero_routes, hero_distribution);
    EXPECT_FALSE(hero_bias.applied);

    const auto engineer_routes = mesh.route_map("engineer", last, 0.6);
    ASSERT_TRUE(engineer_routes.valid);
    auto engineer_distribution = base;
    engineer_distribution.role = "engineer";
    const auto engineer_bias = blind.apply(
        "engineer", last, last, 4.0,
        mesh, engineer_routes, engineer_distribution);
    ASSERT_TRUE(engineer_bias.applied);
    EXPECT_GT(engineer_bias.injected_candidate_count, 0u);
    EXPECT_TRUE(std::any_of(
        engineer_distribution.candidates.begin(),
        engineer_distribution.candidates.end(),
        [](const PriorCandidate& candidate) {
            return candidate.from_blind_zone;
        }));
}

TEST_F(NavigationMeshTest, GateUsesPathDistanceAndBlindZoneCandidates) {
    BlindZonePrior blind;
    blind.load(
        {generated_path("home.yaml"), generated_path("gully.yaml")},
        generated_path("engineer.yaml"));

    PriorGateConfig config;
    config.max_speed_mps = 3.0;
    config.max_guess_distance_m = 4.0;
    config.minimum_confidence = 0.0;
    PriorGate gate(config);
    gate.set_navigation_mesh(&mesh);
    gate.set_blind_zone_prior(&blind);

    PriorDistribution input;
    input.valid = true;
    input.role = "engineer";
    input.horizon_seconds = 2;
    input.fallback_level = FallbackLevel::LOCAL_ZONE;
    input.local_weight = 1.0;
    input.candidates = {{1, {12.8, 8.2}, 1.0}};

    const auto result = gate.apply(
        input, {12.8, 8.2}, {0.0, 0.0}, 2.0, 1.0, "engineer");
    ASSERT_TRUE(result.valid) << result.rejection_reason;
    EXPECT_TRUE(result.mesh_used);
    EXPECT_TRUE(result.blind_zone_biased);
    EXPECT_GT(result.blind_zone_probability_mass, 0.0);
    EXPECT_TRUE(std::any_of(
        result.candidates.begin(), result.candidates.end(),
        [](const GatedCandidate& candidate) {
            return candidate.prior.from_blind_zone && candidate.reachable;
        }));
}

TEST_F(NavigationMeshTest, CentralGullyOcclusionStaysNearLastVisiblePosition) {
    BlindZonePrior blind;
    blind.load(
        {generated_path("home.yaml"), generated_path("gully.yaml")},
        generated_path("engineer.yaml"));

    PriorGateConfig config;
    config.max_speed_mps = 3.0;
    config.max_guess_distance_m = 4.0;
    config.minimum_confidence = 0.0;
    PriorGate gate(config);
    gate.set_navigation_mesh(&mesh);
    gate.set_blind_zone_prior(&blind);

    PriorDistribution input;
    input.valid = true;
    input.role = "sentry";
    input.horizon_seconds = 10;
    input.fallback_level = FallbackLevel::LOCAL_ZONE;
    input.local_weight = 1.0;
    // shadow 日志中的真实哨兵样本：最后可见 canonical=(12.314,11.926)，
    // 76.6 s 后重识别只移动 0.194 m；旧猜点却偏了约 1.03 m。该点位于
    // gully 中心对称侧，模型 10 s 档 stay_probability 约为 0.8085。
    input.stay_probability = 0.808498;
    input.candidates = {{1, {13.25, 11.25}, 1.0}};

    const Point2d last{12.314342, 11.926017};
    const auto result = gate.apply(
        // 即使最后一帧残留 0.5 m/s 速度，76.6 s 后也不应匀速外推到边界。
        input, last, {0.5, 0.0}, 76.6, 1.0, "sentry");
    ASSERT_TRUE(result.valid) << result.rejection_reason;
    ASSERT_TRUE(result.blind_zone_biased);
    EXPECT_GE(result.stay_anchor_probability_mass, 0.70);
    EXPECT_LT(std::hypot(
        result.predicted_canonical.x - last.x,
        result.predicted_canonical.y - last.y), 0.20);

    const auto last_snap = mesh.snap_to_walkable("sentry", last, 0.6);
    const auto predicted_snap = mesh.snap_to_walkable(
        "sentry", result.predicted_canonical, 0.05);
    ASSERT_TRUE(last_snap.valid);
    ASSERT_TRUE(predicted_snap.valid);
    EXPECT_NEAR(
        mesh.cell_height_m(predicted_snap.cell_index),
        mesh.cell_height_m(last_snap.cell_index), 0.08 + 1e-9);

    bool saw_stay_anchor = false;
    for (const auto& candidate : result.candidates) {
        if (!candidate.prior.stay_anchor) {
            continue;
        }
        EXPECT_TRUE(candidate.prior.from_blind_zone);
        EXPECT_TRUE(candidate.reachable);
        EXPECT_GE(candidate.prior.probability, 0.70);
        saw_stay_anchor = true;
    }
    EXPECT_TRUE(saw_stay_anchor);
}

}  // namespace
}  // namespace position_prior
