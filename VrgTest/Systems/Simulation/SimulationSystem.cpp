#include "SimulationSystem.hpp"
#include <entt/entity/registry.hpp>
#include "Components/ActiveBullet.hpp"
#include "Core/Maths.hpp"
#include <Components/Position.hpp>
#include <Components/Mass.hpp>
#include <Components/Velocity.hpp>
#include <Components/SimulationData.hpp>
#include <iostream>
#include <Components/ActiveSimulation.hpp>
#include <Components/Angle.hpp>
#include <Components/MinDistance.hpp>

constexpr unsigned int maxActiveBullets = 10; // Maximum number of active bullets in the simulation
constexpr double minDistanceImprovementThreshold = 0.001; // Minimum improvement in distance to keep the simulation going

void TryStartSimulation(entt::registry& registry, Vec3& shooterPosition, double bulletMass, double muzzleVelocity)
{
	// Get a view of entities with ActiveBullet components.
	auto activeBulletsView = registry.view<ActiveBullet>();

	// Check if there are any active bullets in the simulation.
	if (!activeBulletsView.empty())
		return;

	//SimulationData* simulationData;

	// Check if the simulation data entity already exists.
	auto simulationDataView = registry.view<SimulationData>();
	if (simulationDataView.empty())
	{
		// If the simulation data entity does not exist, log an error and return.
		std::cerr << "Error: No simulation data available." << std::endl;
		return;
	}

	// Get the simulation data.
	SimulationData& simulationData = registry.get<SimulationData>(simulationDataView.front());

	// Calculate the step size for the angles.
	simulationData.angleStep = (simulationData.highAngle - simulationData.lowAngle)
		/ (maxActiveBullets - 1);

	// Create bullets for the simulation.
	for (unsigned int i = 0; i < maxActiveBullets; ++i)
	{
		// Create a new entity for the bullet.
		auto bulletEntity = registry.create();

		double bulletAngle = simulationData.lowAngle + i * simulationData.angleStep;

		// Init bullet.
		registry.emplace<ActiveBullet>(bulletEntity);
		registry.emplace<Mass>(bulletEntity, bulletMass);
		registry.emplace<Position>(bulletEntity, shooterPosition);
		registry.emplace<Velocity>(
			bulletEntity,
			Vec3::fromSpherical(simulationData.theta, bulletAngle) * muzzleVelocity);
		registry.emplace<Angle>(bulletEntity, bulletAngle);
		registry.emplace<MinDistance>(bulletEntity, std::numeric_limits<double>::max());
	}
}

void CheckSimulationEndCondition(entt::registry& registry)
{
	// Get a view of entities with ActiveBullet components.
	auto activeBulletsView = registry.view<ActiveBullet>();

	// If there are active bullets, the simulation is still running.
	if (!activeBulletsView.empty())
		return;

	// Get the simulation data.
	auto simulationDataView = registry.view<SimulationData>();
	if (simulationDataView.empty())
	{
		std::cerr << "Error: No simulation data available." << std::endl;
		return;
	}

	// Get the simulation data.
	const SimulationData& simulationData = registry.get<SimulationData>(simulationDataView.front());

	// Check if the minimum distance has improved significantly.
	if (simulationData.secondMinDistanceDiff >= minDistanceImprovementThreshold)
	{
		// If the minimum distance has improved, keep the simulation going.
		return;
	}

	// Stop the simulation.

	// Get a view of entities with ActiveSimulation components.
	auto activeSimulationView = registry.view<ActiveSimulation>();

	// If there is no active simulation, log an error and return.
	if (activeSimulationView.empty())
	{
		std::cerr << "Error: No active simulation available." << std::endl;
		return;
	}

	// Get the active simulation component.
	ActiveSimulation& activeSimulation = registry.get<ActiveSimulation>(activeSimulationView.front());

	// Stop the simulation.
	activeSimulation.isActive = false;
}

bool GetIsSimulationActive(entt::registry& registry)
{
	// Get a view of entities with ActiveSimulation components.
	auto activeSimulationView = registry.view<ActiveSimulation>();

	// If there is no active simulation, log an error and return.
	if (activeSimulationView.empty())
	{
		std::cerr << "Error: No active simulation available." << std::endl;
		return false;
	}

	// Get the active simulation component.
	const ActiveSimulation& activeSimulation = registry.get<ActiveSimulation>(activeSimulationView.front());

	// Return whether the simulation is active.
	return activeSimulation.isActive;
}

void IterateSimulation(entt::registry& registry)
{
	// Get a view of entities with ActiveBullet components.
	auto activeBulletsView = registry.view<ActiveBullet>();

	// If there are active bullets, the simulation is still running.
	if (!activeBulletsView.empty())
		return;

	// Get a view of SimulationData.
	auto simulationDataView = registry.view<SimulationData>();

	// If there is no simulation data, log an error and return.
	if (simulationDataView.empty())
	{
		// Error log: No simulation data available.
		std::cerr << "Error: No simulation data available." << std::endl;
		return;
	}

	// Get the simulation data.
	SimulationData& simulationData = registry.get<SimulationData>(simulationDataView.front());



	std::cout << "Uhel: " << simulationData.bestAngle << " stupnu, "
		<< "lo: " << simulationData.lowAngle << "->" << std::max(simulationData.lowAngle, simulationData.bestAngle - simulationData.angleStep) << ", "
		<< "hi: " << simulationData.highAngle << "->" << std::min(simulationData.highAngle, simulationData.bestAngle + simulationData.angleStep) << std::endl;



	// Set new simulation boundaries.
	simulationData.lowAngle = std::max(simulationData.lowAngle, simulationData.bestAngle - simulationData.angleStep);
	simulationData.highAngle = std::min(simulationData.highAngle, simulationData.bestAngle + simulationData.angleStep);
	//simulationData.prevMinDistance = simulationData.minDistance;
	simulationData.minDistance = std::numeric_limits<double>::max(); // Reset the minimum distance for the next iteration
}
