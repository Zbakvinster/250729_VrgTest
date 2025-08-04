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

TEST_CASE("RunSimulation handles no hit scenario", "[Simulator]") {
	Vec3 shooterPosition{ 0.0, 0.0, 0.0 };
	Vec3 targetPosition{ 50.0, 10000.0, 0.0 }; // target is elevated
	double muzzleVelocity = 50.0; // m/s
	double bulletMass = 0.01; // 10 grams
	double dt = 0.01; // simulation step
	double bestAngle = 0.0;
	double minDistance = 0.0;

	bool hit = RunSimulation(shooterPosition, targetPosition, muzzleVelocity, bulletMass, dt, bestAngle, minDistance);

	CHECK(hit == false);
	CHECK(minDistance > 1.0); // should not hit the target
}

TEST_CASE("RunSimulation with zero velocity", "[Simulator]") {
	Vec3 shooterPosition{ 0.0, 0.0, 0.0 };
	Vec3 targetPosition{ 50.0, 0.0, 0.0 }; // target directly in front
	double muzzleVelocity = 0.0; // zero velocity
	double bulletMass = 0.01; // 10 grams
	double dt = 0.01; // simulation step
	double bestAngle = 0.0;
	double minDistance = 0.0;

	bool hit = RunSimulation(shooterPosition, targetPosition, muzzleVelocity, bulletMass, dt, bestAngle, minDistance);
	
	CHECK(hit == false);
	CHECK(minDistance > 50.0); // should not hit the target
}

TEST_CASE("RunSimulation with negative muzzle velocity", "[Simulator]") {
	Vec3 shooterPosition{ 0.0, 0.0, 0.0 };
	Vec3 targetPosition{ 50.0, 0.0, 0.0 }; // target directly in front
	double muzzleVelocity = -50.0; // negative velocity
	double bulletMass = 0.01; // 10 grams
	double dt = 0.01; // simulation step
	double bestAngle = 0.0;
	double minDistance = 0.0;
	
	bool hit = RunSimulation(shooterPosition, targetPosition, muzzleVelocity, bulletMass, dt, bestAngle, minDistance);
	
	CHECK(hit == false);
	CHECK(minDistance > 50.0); // should not hit the target
}

TEST_CASE("RunSimulation with target directly above shooter", "[Simulator]") {
	using Catch::Approx;

	Vec3 shooterPosition{ 0, 0, 0 };
	Vec3 targetPosition{ 0, 10, 0 };
	double muzzleVelocity = 50.0;
	double bulletMass = 0.01;
	double dt = 0.01;

	double bestAngle = 0.0;
	double minDistance = 0.0;

	bool result = RunSimulation(shooterPosition, targetPosition, muzzleVelocity, bulletMass, dt, bestAngle, minDistance);

	REQUIRE(result == true);
	CHECK(minDistance <= Approx(0.005));
}

TEST_CASE("RunSimulation hits extremely close target", "[Simulator]") {
	using Catch::Approx;

	Vec3 shooterPosition{ 0, 0, 0 };
	Vec3 targetPosition{ 0.5, 0, 0 };
	double muzzleVelocity = 100.0;
	double bulletMass = 0.01;
	double dt = 0.01;

	double bestAngle = 0.0;
	double minDistance = 0.0;

	bool result = RunSimulation(shooterPosition, targetPosition, muzzleVelocity, bulletMass, dt, bestAngle, minDistance);

	REQUIRE(result == true);
	REQUIRE(minDistance <= Approx(0.005));
}

TEST_CASE("RunSimulation fails for very distant target", "[Simulator]") {
	using Catch::Approx;

	Vec3 shooterPosition{ 0, 0, 0 };
	Vec3 targetPosition{ 1000, 0, 0 };
	double muzzleVelocity = 30.0;
	double bulletMass = 0.01;
	double dt = 0.01;

	double bestAngle = 0.0;
	double minDistance = 0.0;

	bool result = RunSimulation(shooterPosition, targetPosition, muzzleVelocity, bulletMass, dt, bestAngle, minDistance);

	REQUIRE(result == false);
}

TEST_CASE("RunSimulation hits target below shooter", "[Simulator]") {
	using Catch::Approx;

	Vec3 shooterPosition{ 0, 10, 0 };
	Vec3 targetPosition{ 10, 0, 0 };
	double muzzleVelocity = 100.0;
	double bulletMass = 0.01;
	double dt = 0.01;

	double bestAngle = 0.0;
	double minDistance = 0.0;

	bool result = RunSimulation(shooterPosition, targetPosition, muzzleVelocity, bulletMass, dt, bestAngle, minDistance);

	REQUIRE(result == true);
	REQUIRE(minDistance <= Approx(0.005));
}

TEST_CASE("RunSimulation handles negative dt gracefully", "[Simulator]") {
	using Catch::Approx;

	Vec3 shooterPosition{ 0, 0, 0 };
	Vec3 targetPosition{ 10, 0, 0 };
	double muzzleVelocity = 100.0;
	double bulletMass = 0.01;
	double dt = -0.01;

	double bestAngle = 0.0;
	double minDistance = 0.0;

	bool result = RunSimulation(shooterPosition, targetPosition, muzzleVelocity, bulletMass, dt, bestAngle, minDistance);

	REQUIRE_FALSE(result);
}
