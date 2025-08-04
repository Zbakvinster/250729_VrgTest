#include "catch_amalgamated.hpp"
#include <entt/entity/registry.hpp>
#include "../VrgTest/Systems/TargetMinDistanceCheck/TargetMinDistanceCheckSystem.hpp"
#include "../VrgTest/Components/Position.hpp"
#include "../VrgTest/Components/MinDistance.hpp"
#include "../VrgTest/Core/Maths.hpp"

using Catch::Approx;

TEST_CASE("CheckTargetMinDistance updates MinDistance correctly", "[TargetMinDistanceCheckSystem]") {
    entt::registry registry;

    // Create entity with Position and MinDistance
    auto entity = registry.create();
    registry.emplace<Position>(entity, Vec3{ 1.0, 2.0, 3.0 });
    registry.emplace<MinDistance>(entity, 10.0);

    Vec3 targetPos{ 2.0, 2.0, 3.0 }; // distance = 1.0

    CheckTargetMinDistance(registry, targetPos);

    auto& minDist = registry.get<MinDistance>(entity);
    CHECK(minDist.value == Approx(1.0));

    // Run again with closer target, distance = 0.5
    targetPos = Vec3{ 1.5, 2.0, 3.0 };
    CheckTargetMinDistance(registry, targetPos);
    CHECK(minDist.value == Approx(0.5));

    // Run again with farther target, minDist should NOT increase
    targetPos = Vec3{ 5.0, 5.0, 5.0 };
    CheckTargetMinDistance(registry, targetPos);
    CHECK(minDist.value == Approx(0.5)); // unchanged
}
