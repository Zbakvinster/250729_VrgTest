#include "DeadBulletSolverSystem.hpp"
#include <entt/entity/registry.hpp>
#include "Components/DeadBullet.hpp"
#include "Components/Angle.hpp"
#include "Components/MinDistance.hpp"
#include "Components/SimulationData.hpp"
#include <iostream>
#include <Components/ActiveBullet.hpp>

struct MinDistanceInfo
{
	double angle;
	double minDistance;
};

void SolveDeadBullets(entt::registry& registry)
{
	// Get a view of entities with ActiveBullet components.
	auto activeBulletsView = registry.view<ActiveBullet>();

	// If there are active bullets, the simulation is still running.
	if (!activeBulletsView.empty())
		return;

	// Get SimulationDataView.
	auto simulationDataView = registry.view<SimulationData>();
	if (simulationDataView.empty())
	{
		// Error log: No simulation data available.
		std::cerr << "Error: No simulation data available." << std::endl;
		return;
	}

	// Get the simulation data.
	SimulationData& simulationData = registry.get<SimulationData>(simulationDataView.front());

	// Check if there are any dead bullets to process.
	auto deadBulletsView = registry.view<DeadBullet, Angle, MinDistance>();

	std::vector<MinDistanceInfo> deadBulletsInfo;
	deadBulletsInfo.reserve(deadBulletsView.size_hint());

	// Iterate over all dead bullets to find the best angle and distance.
	deadBulletsView.each([&registry, &deadBulletsInfo](
		auto entity,
		const Angle& angle,
		const MinDistance& minDistance)
	{
		//// Check if the bullet reached better distance than the current best.
		//if (minDistance.value < simulationData.minDistance)
		//{
		//	// Update the simulation data with the new best angle and distance.
		//	simulationData.minDistance = minDistance.value;
		//	simulationData.bestAngle = angle.value;
		//}

		deadBulletsInfo.push_back({angle.value, minDistance.value});

		// Destroy the dead bullet entity - it is no longer needed.
		registry.destroy(entity);
	});

	// Order deadBulletsInfo by angle.
	std::sort(
		deadBulletsInfo.begin(),
		deadBulletsInfo.end(),
		[](const MinDistanceInfo& a, const MinDistanceInfo& b) {
			return a.angle < b.angle;
	});

	// Iterate over the sorted deadBulletsInfo to find the best angle and distance.
	double bestAngle = simulationData.bestAngle;
	double bestMinDistance = std::numeric_limits<double>::max();
	double secondMinDistanceDiff = std::numeric_limits<double>::max();
	for (const auto& info : deadBulletsInfo)
	{
		// Check if the current bullet has a better distance than the best found so far.
		if (info.minDistance < bestMinDistance)
		{
			secondMinDistanceDiff = bestMinDistance - info.minDistance; // Update the second minimum distance difference.
			bestMinDistance = info.minDistance;
			bestAngle = info.angle;
		}
		else
		{
			// If the current bullet does not have a better distance, check if it is the second best.
			double diff = info.minDistance - bestMinDistance;
			if (diff < secondMinDistanceDiff)
			{
				secondMinDistanceDiff = diff; // Update the second minimum distance difference.
			}

			break;
		}
	}

	// Update the simulation data with the best angle and distances found.
	simulationData.bestAngle = bestAngle;
	simulationData.minDistance = bestMinDistance;
	simulationData.secondMinDistanceDiff = secondMinDistanceDiff;
}
