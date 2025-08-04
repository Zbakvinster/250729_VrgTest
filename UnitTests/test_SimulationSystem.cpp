#include "catch_amalgamated.hpp"
#include <entt/entity/registry.hpp>
#include "../VrgTest/Systems/Simulation/SimulationSystem.hpp"
#include "../VrgTest/Components/ActiveBullet.hpp"
#include "../VrgTest/Components/ActiveSimulation.hpp"
#include "../VrgTest/Components/SimulationData.hpp"
#include "../VrgTest/Components/Position.hpp"
#include "../VrgTest/Components/Mass.hpp"
#include "../VrgTest/Components/Velocity.hpp"
#include "../VrgTest/Core/Maths.hpp"
#include <Components/Angle.hpp>
#include <Components/MinDistance.hpp>

using Catch::Approx;

TEST_CASE("SimulationSystem TryStartSimulation creates bullets with correct components", "[SimulationSystem]") {
    entt::registry registry;

    SimulationData simData{};
    simData.theta = 45.0;
    simData.lowAngle = 0.0;
    simData.highAngle = 90.0;
    simData.angleStep = 0.0;
    simData.bestAngle = 45.0;
    simData.minDistance = std::numeric_limits<double>::max();
    simData.secondMinDistanceDiff = std::numeric_limits<double>::max();

    auto simDataEntity = registry.create();
    registry.emplace<SimulationData>(simDataEntity, simData);

    Vec3 shooterPos{ 0,0,0 };
    double bulletMass = 1.0;
    double muzzleVelocity = 10.0;

    TryStartSimulation(registry, shooterPos, bulletMass, muzzleVelocity);

    auto bulletsView = registry.view<ActiveBullet, Mass, Position, Velocity, Angle, MinDistance>();
    CHECK(bulletsView.size_hint() == 10);

    bulletsView.each([&](auto entity, const Mass& mass, const Position& pos, const Velocity& vel, const Angle& angle, const MinDistance& minDist) {
        CHECK(mass.value == Approx(bulletMass));
        CHECK(pos.value.x == Approx(0.0));
        CHECK(pos.value.y == Approx(0.0));
        CHECK(pos.value.z == Approx(0.0));
        CHECK(minDist.value == Approx(std::numeric_limits<double>::max()));
        CHECK(angle.value >= 0.0);
        CHECK(angle.value <= 90.0);
        (void)entity; (void)vel;
    });
}

TEST_CASE("SimulationSystem CheckSimulationEndCondition stops simulation if improvement too low", "[SimulationSystem]") {
    entt::registry registry;

    SimulationData simData{};
    simData.theta = 45.0;
    simData.lowAngle = 0.0;
    simData.highAngle = 90.0;
    simData.angleStep = 0.0;
    simData.bestAngle = 45.0;
    simData.minDistance = 100.0;
    simData.secondMinDistanceDiff = 0.0001;  // less than threshold (0.001)

    auto simDataEntity = registry.create();
    registry.emplace<SimulationData>(simDataEntity, simData);

    auto activeSimEntity = registry.create();
    ActiveSimulation activeSim{};
    activeSim.isActive = true;
    registry.emplace<ActiveSimulation>(activeSimEntity, activeSim);

    CheckSimulationEndCondition(registry);

    const auto& activeSimOut = registry.get<ActiveSimulation>(activeSimEntity);
    CHECK(activeSimOut.isActive == false);
}

TEST_CASE("SimulationSystem GetIsSimulationActive returns correct state", "[SimulationSystem]") {
    entt::registry registry;

    auto activeSimEntity = registry.create();
    ActiveSimulation activeSim{};
    activeSim.isActive = true;
    registry.emplace<ActiveSimulation>(activeSimEntity, activeSim);

    CHECK(GetIsSimulationActive(registry) == true);

    registry.patch<ActiveSimulation>(activeSimEntity, [](ActiveSimulation& sim) { sim.isActive = false; });
    CHECK(GetIsSimulationActive(registry) == false);
}

TEST_CASE("SimulationSystem IterateSimulation updates simulation boundaries and resets minDistance", "[SimulationSystem]") {
    entt::registry registry;

    SimulationData simData{};
    simData.theta = 45.0;
    simData.lowAngle = 10.0;
    simData.highAngle = 80.0;
    simData.angleStep = 5.0;
    simData.bestAngle = 50.0;
    simData.minDistance = 20.0;
    simData.secondMinDistanceDiff = 0.002;

    auto simDataEntity = registry.create();
    registry.emplace<SimulationData>(simDataEntity, simData);

    IterateSimulation(registry);

    auto& simDataOut = registry.get<SimulationData>(simDataEntity);

    CHECK(simDataOut.lowAngle == Approx(std::max(10.0, 50.0 - 5.0)));
    CHECK(simDataOut.highAngle == Approx(std::min(80.0, 50.0 + 5.0)));
    CHECK(simDataOut.minDistance == Approx(std::numeric_limits<double>::max()));
}
