#include "catch_amalgamated.hpp"
#include <entt/entity/registry.hpp>

#include "../VrgTest/Systems/Physics/PhysicsSystem.hpp"
#include "../VrgTest/Components/Velocity.hpp"
#include "../VrgTest/Components/Mass.hpp"
#include "../VrgTest/Core/Maths.hpp"

using Catch::Approx;

TEST_CASE("ApplyPhysics applies gravity and drag", "[PhysicsSystem]") {
    entt::registry registry;

    auto entity = registry.create();

    // Initial velocity: 100 m/s horizontally
    registry.emplace<Velocity>(entity, Velocity{ Vec3{100.0, 0.0, 0.0} });

    // Mass of typical bullet (e.g. 0.01 kg)
    registry.emplace<Mass>(entity, Mass{ 0.01 });

    // Copy initial velocity for later comparison
    Vec3 initialVelocity = registry.get<Velocity>(entity).value;

    // Simulate for 0.1 seconds
    ApplyPhysics(registry, 0.1);

    const auto& vel = registry.get<Velocity>(entity).value;

    // Expect x component to decrease due to drag
    CHECK(vel.x < initialVelocity.x);

    // Expect y component to decrease due to gravity
    CHECK(vel.y < initialVelocity.y);

    // Expect z component to remain zero
    CHECK(vel.z == Approx(0.0));
}

TEST_CASE("ApplyPhysics with zero velocity doesn't crash", "[PhysicsSystem]") {
    entt::registry registry;

    auto entity = registry.create();
    registry.emplace<Velocity>(entity, Velocity{ Vec3{0.0, 0.0, 0.0} });
    registry.emplace<Mass>(entity, Mass{ 1.0 });

    ApplyPhysics(registry, 0.1);

    const auto& vel = registry.get<Velocity>(entity).value;

    // Should be affected only by gravity (acceleration = -9.81 m/s^2)
    CHECK(vel.x == Approx(0.0));
    CHECK(vel.y == Approx(-9.81 * 0.1).margin(0.01));
    CHECK(vel.z == Approx(0.0));
}

TEST_CASE("ApplyPhysics skips entities without required components", "[PhysicsSystem]") {
    entt::registry registry;

    auto noMass = registry.create();
    registry.emplace<Velocity>(noMass, Velocity{ Vec3{1.0, 1.0, 1.0} });

    auto noVelocity = registry.create();
    registry.emplace<Mass>(noVelocity, Mass{ 1.0 });

    REQUIRE_NOTHROW(ApplyPhysics(registry, 0.1));

    // Check that velocity didn't change for incomplete entity
    CHECK(registry.get<Velocity>(noMass).value == Vec3{ 1.0, 1.0, 1.0 });
}
