#include "catch_amalgamated.hpp"
#include "../VrgTest/Simulator.hpp"
#include "../VrgTest/Core/Maths.hpp"

using Catch::Approx;

TEST_CASE("RunSimulation finds best angle close to expected", "[Simulator]") {
    Vec3 shooterPosition{ 0.0, 0.0, 0.0 };
    Vec3 targetPosition{ 50.0, 0.0, 0.0 }; // 50 meters directly in front
    double muzzleVelocity = 50.0; // m/s
    double bulletMass = 0.01; // 10 grams
    double dt = 0.01; // simulation step
    double bestAngle = 0.0;
    double minDistance = 0.0;

    bool hit = RunSimulation(shooterPosition, targetPosition, muzzleVelocity, bulletMass, dt, bestAngle, minDistance);

    CHECK(hit == true);
    CHECK(minDistance <= Approx(0.005));
    CHECK(bestAngle > 0.0);
    CHECK(bestAngle < 90.0);
}
