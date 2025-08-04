#include "catch_amalgamated.hpp"
#include <entt/entity/registry.hpp>
#include "../VrgTest/Systems/VelocityApplier/VelocityApplierSystem.hpp"
#include "../VrgTest/Components/Position.hpp"
#include "../VrgTest/Components/Velocity.hpp"
#include "../VrgTest/Core/Maths.hpp"

using Catch::Approx;

TEST_CASE("ApplyVelocity updates positions correctly", "[VelocityApplierSystem]") {
    entt::registry registry;

    auto entity = registry.create();
    registry.emplace<Position>(entity, Vec3{ 0.0, 0.0, 0.0 });
    registry.emplace<Velocity>(entity, Vec3{ 1.0, 2.0, 3.0 });

    double dt = 0.5;

    ApplyVelocity(registry, dt);

    const auto& pos = registry.get<Position>(entity);

    CHECK(pos.value.x == Approx(0.5));
    CHECK(pos.value.y == Approx(1.0));
    CHECK(pos.value.z == Approx(1.5));
}
