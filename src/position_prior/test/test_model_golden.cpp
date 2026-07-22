#include "position_prior/position_prior_model.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <string>

namespace position_prior {
namespace {

constexpr const char* EXPECTED_SHA256 =
    "b1a7384042939aca3fcfa3d45a5bbd3cd8b7e3bdeaa658aac24d17f36e2df53e";

std::string toolkit_path(const std::string& filename) {
    return (std::filesystem::path(POSITION_PRIOR_TOOLKIT_DIR) / "run_v1" / filename).string();
}

class ModelGoldenTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        model.load(toolkit_path("04_rmuc2026_position_prior_v1.yaml"), EXPECTED_SHA256);
        cases = YAML::LoadFile(toolkit_path("06_golden_cases.json"));
    }

    static PositionPriorModel model;
    static YAML::Node cases;
};

PositionPriorModel ModelGoldenTest::model;
YAML::Node ModelGoldenTest::cases;

TEST_F(ModelGoldenTest, MetadataAndHashAreValidated) {
    ASSERT_TRUE(model.loaded());
    EXPECT_EQ(model.model_sha256(), EXPECTED_SHA256);
    EXPECT_EQ(model.model_version(), 1);
    EXPECT_EQ(model.map_id(), "RMUC2026");
    EXPECT_DOUBLE_EQ(model.field_length(), 28.0);
    EXPECT_DOUBLE_EQ(model.field_width(), 15.0);
    EXPECT_EQ(model.horizons_seconds(), (std::vector<int>{2, 5, 10}));
}

TEST_F(ModelGoldenTest, AllPythonGoldenCasesMatchCppLoader) {
    ASSERT_TRUE(cases.IsSequence());
    ASSERT_EQ(cases.size(), 15u);
    for (const auto& test_case : cases) {
        const Point2d current{
            test_case["canonical_current"][0].as<double>(),
            test_case["canonical_current"][1].as<double>()};
        const auto result = model.query(
            test_case["role"].as<std::string>(), current,
            test_case["horizon_seconds"].as<int>(),
            test_case["context"].as<std::string>(), 5);
        SCOPED_TRACE(test_case["role"].as<std::string>() + "/" +
            std::to_string(test_case["horizon_seconds"].as<int>()));
        ASSERT_TRUE(result.valid) << result.error;
        EXPECT_EQ(result.zone_index, test_case["expected_zone_index"].as<int>());
        const auto expected = test_case["expected_candidates"];
        ASSERT_EQ(result.candidates.size(), expected.size());
        for (std::size_t index = 0; index < expected.size(); ++index) {
            EXPECT_EQ(result.candidates[index].grid_index,
                expected[index]["grid_index"].as<int>());
            EXPECT_NEAR(result.candidates[index].canonical.x,
                expected[index]["x"].as<double>(), 1e-9);
            EXPECT_NEAR(result.candidates[index].canonical.y,
                expected[index]["y"].as<double>(), 1e-9);
            EXPECT_NEAR(result.candidates[index].probability,
                expected[index]["p"].as<double>(), 1e-9);
        }
    }
}

TEST_F(ModelGoldenTest, BoundaryZoneIndexIsClamped) {
    EXPECT_EQ(model.zone_index({0.0, 0.0}), 0);
    EXPECT_EQ(model.zone_index({28.0, 15.0}), 139);
    EXPECT_EQ(model.zone_index({27.999, 14.999}), 139);
    EXPECT_EQ(model.zone_index({-0.001, 1.0}), -1);
}

TEST_F(ModelGoldenTest, InvalidHashFailsClosed) {
    PositionPriorModel invalid;
    EXPECT_THROW(
        invalid.load(toolkit_path("04_rmuc2026_position_prior_v1.yaml"), "deadbeef"),
        std::runtime_error);
    EXPECT_FALSE(invalid.loaded());
}

}  // namespace
}  // namespace position_prior
