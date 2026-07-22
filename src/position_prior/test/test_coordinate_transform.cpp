#include "position_prior/coordinate_transform.hpp"

#include <gtest/gtest.h>

#include <array>

namespace position_prior {
namespace {

void expect_point(const Point2d& actual, const Point2d& expected) {
    EXPECT_NEAR(actual.x, expected.x, 1e-9);
    EXPECT_NEAR(actual.y, expected.y, 1e-9);
}

TEST(CoordinateTransformTest, RedCanonicalIsFieldIdentity) {
    CoordinateTransform transform;
    const std::array<Point2d, 5> points{{
        {0.0, 0.0}, {28.0, 0.0}, {0.0, 15.0}, {28.0, 15.0}, {14.0, 7.5}}};
    for (const auto& point : points) {
        const auto canonical = transform.field_to_canonical(1, point);
        ASSERT_TRUE(canonical.has_value());
        expect_point(*canonical, point);
    }
}

TEST(CoordinateTransformTest, BlueCanonicalUsesCentralSymmetry) {
    CoordinateTransform transform;
    const auto canonical = transform.field_to_canonical(2, {2.0, 3.0});
    ASSERT_TRUE(canonical.has_value());
    expect_point(*canonical, {26.0, 12.0});
    const auto restored = transform.canonical_to_field(2, *canonical);
    ASSERT_TRUE(restored.has_value());
    expect_point(*restored, {2.0, 3.0});
}

TEST(CoordinateTransformTest, WorldFieldRoundTripWorksInBothMapOrientations) {
    for (const bool toward_blue : {false, true}) {
        CoordinateTransform transform(28.0, 15.0, toward_blue);
        for (const Point2d world : {Point2d{-7.5, -14.0}, Point2d{7.5, 14.0},
                                    Point2d{1.25, -3.5}}) {
            const auto field = transform.world_to_field(world);
            ASSERT_TRUE(field.has_value());
            const auto restored = transform.field_to_world(*field);
            ASSERT_TRUE(restored.has_value());
            expect_point(*restored, world);
        }
    }
}

TEST(CoordinateTransformTest, BlueWorldCanonicalRoundTripPreservesBoundaryCorners) {
    CoordinateTransform transform;
    for (const Point2d canonical : {Point2d{0.0, 0.0}, Point2d{28.0, 0.0},
                                    Point2d{0.0, 15.0}, Point2d{28.0, 15.0}}) {
        const auto world = transform.canonical_to_world(2, canonical);
        ASSERT_TRUE(world.has_value());
        const auto restored = transform.world_to_canonical(2, *world);
        ASSERT_TRUE(restored.has_value());
        expect_point(*restored, canonical);
    }
}

TEST(CoordinateTransformTest, RejectsInvalidTeamAndOutOfFieldInputs) {
    CoordinateTransform transform;
    EXPECT_FALSE(transform.field_to_canonical(0, {1.0, 1.0}).has_value());
    EXPECT_FALSE(transform.field_to_canonical(1, {-0.01, 1.0}).has_value());
    EXPECT_FALSE(transform.canonical_to_world(2, {28.01, 1.0}).has_value());
}

TEST(CoordinateTransformTest, VelocityTransformDoesNotApplyTranslation) {
    CoordinateTransform transform;
    expect_point(transform.world_velocity_to_field({2.0, 3.0}), {3.0, 2.0});
    const auto blue = transform.field_velocity_to_canonical(2, {3.0, 2.0});
    ASSERT_TRUE(blue.has_value());
    expect_point(*blue, {-3.0, -2.0});
}

}  // namespace
}  // namespace position_prior
