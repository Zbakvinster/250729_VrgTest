#include "catch_amalgamated.hpp"
#include <entt/entity/registry.hpp>

#include "../VrgTest/Systems/DeadBulletSolver/DeadBulletSolverSystem.hpp"
#include "../VrgTest/Components/DeadBullet.hpp"
#include "../VrgTest/Components/Angle.hpp"
#include "../VrgTest/Components/MinDistance.hpp"
#include "../VrgTest/Components/SimulationData.hpp"
#include "../VrgTest/Components/ActiveBullet.hpp"

using Catch::Approx;

TEST_CASE("SolveDeadBullets updates best angle and distances", "[DeadBulletSolverSystem]") {
    entt::registry registry;

    // SimulationData entity
    auto simEntity = registry.create();
    registry.emplace<SimulationData>(simEntity); // využíváme defaultní hodnoty

    // Dead bullets
    auto e1 = registry.create();
    registry.emplace<DeadBullet>(e1);
    registry.emplace<Angle>(e1, Angle{ 10.0 });
    registry.emplace<MinDistance>(e1, MinDistance{ 30.0 });

    auto e2 = registry.create();
    registry.emplace<DeadBullet>(e2);
    registry.emplace<Angle>(e2, Angle{ 20.0 });
    registry.emplace<MinDistance>(e2, MinDistance{ 25.0 }); // best

    auto e3 = registry.create();
    registry.emplace<DeadBullet>(e3);
    registry.emplace<Angle>(e3, Angle{ 30.0 });
    registry.emplace<MinDistance>(e3, MinDistance{ 27.0 }); // second best

    // Run system
    SolveDeadBullets(registry);

    // Check results
    const auto& sim = registry.get<SimulationData>(simEntity);
    CHECK(sim.bestAngle == Approx(20.0));
    CHECK(sim.minDistance == Approx(25.0));
    CHECK(sim.secondMinDistanceDiff == Approx(2.0));

    CHECK_FALSE(registry.valid(e1));
    CHECK_FALSE(registry.valid(e2));
    CHECK_FALSE(registry.valid(e3));
}

TEST_CASE("SolveDeadBullets does nothing if active bullets exist", "[DeadBulletSolverSystem]") {
    entt::registry registry;

    // Active bullet (prevents processing)
    auto active = registry.create();
    registry.emplace<ActiveBullet>(active);

    // Dead bullet that should be ignored
    auto dead = registry.create();
    registry.emplace<DeadBullet>(dead);
    registry.emplace<Angle>(dead, Angle{ 42.0 });
    registry.emplace<MinDistance>(dead, MinDistance{ 1.0 });

    // Simulation data
    auto simEntity = registry.create();
    registry.emplace<SimulationData>(simEntity); // výchozí hodnoty

    SolveDeadBullets(registry);

    const auto& sim = registry.get<SimulationData>(simEntity);
    CHECK(sim.bestAngle == Approx(0.0));
    CHECK(sim.minDistance == Approx(std::numeric_limits<double>::max()));

    CHECK(registry.valid(dead)); // nebyl znièen
}

TEST_CASE("SolveDeadBullets handles missing SimulationData safely", "[DeadBulletSolverSystem]") {
    entt::registry registry;

    auto e = registry.create();
    registry.emplace<DeadBullet>(e);
    registry.emplace<Angle>(e, Angle{ 15.0 });
    registry.emplace<MinDistance>(e, MinDistance{ 100.0 });

    REQUIRE_NOTHROW(SolveDeadBullets(registry));
    CHECK_FALSE(registry.valid(e)); // dead bullet should still be destroyed
}
