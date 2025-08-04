#include <Simulator.hpp>
#include <iostream>
#include <entt/entity/registry.hpp>
#include <Components/SimulationData.hpp>
#include <Components/ActiveSimulation.hpp>
#include <Components/ActiveBullet.hpp>
#include <Components/Mass.hpp>
#include <Components/Position.hpp>
#include <Components/Velocity.hpp>
#include <Systems/Simulation/SimulationSystem.hpp>
#include <Systems/Physics/PhysicsSystem.hpp>
#include <Systems/VelocityApplier/VelocityApplierSystem.hpp>
#include <Systems/BulletEndCondition/BulletEndConditionSystem.hpp>
#include <Systems/TargetMinDistanceCheck/TargetMinDistanceCheckSystem.hpp>
#include <Systems/DeadBulletSolver/DeadBulletSolverSystem.hpp>
#include <Systems/Print/PrintSystem.hpp>

constexpr double hitOrMissThreshold = 0.005; // Threshold for considering a hit or miss in meters.

bool RunSimulation(const Vec3& shooterPosition, const Vec3& targetPosition, double muzzleVelocity, double bulletMass, double dt, double& bestAngleOut, double& minDistanceOut)
{
	Vec3 toTarget = targetPosition - shooterPosition;
	double targetHorizontalDistanceSqrt = toTarget.x * toTarget.x + toTarget.z * toTarget.z;

	// Create an entity registry.
	entt::registry registry;

	double lowAngle =
		std::atan2(toTarget.y, std::sqrt(targetHorizontalDistanceSqrt))
		* 180.0
		/ M_PI;

	// Initialize the simulation data.
	const auto simulationDataEntity = registry.create();
	registry.emplace<SimulationData>(
		simulationDataEntity,
		std::atan2(toTarget.z, toTarget.x) * 180.0 / M_PI, // Theta angle in degrees
		lowAngle,
		90.0, // High angle
		5.0, // Angle step
		lowAngle, // Low angle as the initial best angle
		std::numeric_limits<double>::max(), // Minimum distance to the target
		std::numeric_limits<double>::max()); // Previous minimum distance to the target

	const auto activeSimulationEntity = registry.create();
	registry.emplace<ActiveSimulation>(activeSimulationEntity, true); // Start the simulation as active

	while (true)
	{
		TryStartSimulation(registry, shooterPosition, bulletMass, muzzleVelocity);
		ApplyPhysics(registry, dt);
		ApplyVelocity(registry, dt);
		CheckBulletEndCondition(registry, shooterPosition, targetPosition, targetHorizontalDistanceSqrt);
		CheckTargetMinDistance(registry, targetPosition);
		SolveDeadBullets(registry);
		CheckSimulationEndCondition(registry);

		// Check if the simulation is still active.
		if (!GetIsSimulationActive(registry))
			break; // Exit the loop if the simulation is no longer active.

		// Iterate the simulation.
		IterateSimulation(registry);
	}

	auto simulationDataView = registry.view<SimulationData>();

	if (simulationDataView.empty())
	{
		std::cerr << "Error: No simulation data available." << std::endl;
		return false;
	}

	const SimulationData& simulationData = registry.get<SimulationData>(simulationDataView.front());

	bestAngleOut = simulationData.bestAngle;
	minDistanceOut = simulationData.minDistance;

	return simulationData.minDistance <= hitOrMissThreshold;
}

void PrintBulletTrajectory(const Vec3& shooterPosition, const Vec3& targetPosition, double elevationAngle, double muzzleVelocity, double bulletMass, double dt)
{
	// Create an entity registry.
	entt::registry registry;

	// Create a new entity for the bullet.
	auto bulletEntity = registry.create();

	Vec3 toTarget = targetPosition - shooterPosition;
	double targetHorizontalDistanceSqrt = toTarget.x * toTarget.x + toTarget.z * toTarget.z;

	// Init bullet.
	registry.emplace<ActiveBullet>(bulletEntity);
	registry.emplace<Mass>(bulletEntity, bulletMass);
	registry.emplace<Position>(bulletEntity, shooterPosition);
	registry.emplace<Velocity>(
		bulletEntity,
		Vec3::fromSpherical(std::atan2(toTarget.z, toTarget.x) * 180.0 / M_PI, elevationAngle) * muzzleVelocity);

	const auto activeSimulationEntity = registry.create();
	registry.emplace<ActiveSimulation>(activeSimulationEntity, true); // Start the simulation as active

	// Initialize the simulation data.
	const auto simulationDataEntity = registry.create();
	registry.emplace<SimulationData>(
		simulationDataEntity,
		std::atan2(toTarget.z, toTarget.x) * 180.0 / M_PI, // Theta angle in degrees
		0.0,
		90.0, // High angle
		5.0, // Angle step
		0.0, // Low angle as the initial best angle
		std::numeric_limits<double>::max(), // Minimum distance to the target
		std::numeric_limits<double>::max()); // Previous minimum distance to the target

	std::cout << "Trajectory:\n";

	while (true)
	{
		ApplyPhysics(registry, dt);
		ApplyVelocity(registry, dt);
		PrintPositions(registry);
		CheckBulletEndCondition(registry, shooterPosition, targetPosition, targetHorizontalDistanceSqrt);

		// Check if the simulation is still active.
		auto view = registry.view<ActiveBullet>();
		if (view.empty())
			break; // Exit the loop if the simulation is no longer active.
	}
}
