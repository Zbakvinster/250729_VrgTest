#include "TargetMinDistanceCheckSystem.hpp"
#include <entt/entity/registry.hpp>
#include "Components/MinDistance.hpp"
#include "Components/Position.hpp"
#include "Core/Maths.hpp"

void CheckTargetMinDistance(entt::registry& registry, const Vec3& targetPosition)
{
	// Get a view of entities with Position and MinDistance components.
	auto view = registry.view<Position, MinDistance>();

	// Iterate over each entity in the view.
	view.each([&registry, &targetPosition](Position& pos, MinDistance& minDist)
	{
		// Calculate the distance to the target position.
		double distance = (pos.value - targetPosition).length();

		// Update the minimum distance if the current distance is smaller.
		if (distance < minDist.value)
		{
			minDist.value = distance;
		}
	});
}