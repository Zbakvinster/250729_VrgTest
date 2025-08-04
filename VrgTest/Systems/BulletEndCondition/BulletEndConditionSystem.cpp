#include "BulletEndConditionSystem.hpp"
#include <entt/entity/registry.hpp>
#include "Core/Maths.hpp"
#include "Components/Position.hpp"
#include "Components/Velocity.hpp"
#include "Components/DeadBullet.hpp"
#include <Components/ActiveBullet.hpp>

void KillBullet(entt::registry& registry, entt::entity entity) {
	// Remove the ActiveBullet component to mark the bullet as no longer active.
	registry.remove<ActiveBullet>(entity);

	// Remove the velocity component to stop further updates.
	registry.remove<Velocity>(entity);

	// Mark the bullet as dead.
	registry.emplace<DeadBullet>(entity);
}

void CheckBulletEndCondition(entt::registry& registry, const Vec3& shooterPosition, const Vec3& targetPosition, const double targetHorizontalDistanceSqrt){
	// Check if any entity has reached the end condition.
	auto view = registry.view<Position, Velocity>();

	view.each([&registry, &shooterPosition, &targetHorizontalDistanceSqrt, &targetPosition](
		const auto entity,
		Position& pos,
		const Velocity& vel)
	{
		// Check if the bullet fell below the ground level (y <= 0.0).
		if (pos.value.y <= 0.0)
		{
			// If the bullet has fallen below ground level, remove it.
			KillBullet(registry, entity);

			return;
		}

		// Calculate how far the bullet has traveled horizontally.
		double horizontalDistanceSqrt =
			(pos.value.x - shooterPosition.x) * (pos.value.x - shooterPosition.x)
			+ (pos.value.z - shooterPosition.z) * (pos.value.z - shooterPosition.z);

		// If the bullet overshoots the target, remove it.
		if (horizontalDistanceSqrt > targetHorizontalDistanceSqrt)
		{
			// Calculate the position where the bullet should be at the end of its trajectory.

			// Get the velocity of the bullet.
			//const auto& vel = view.get<Velocity>(entity);

			// Calculate the time to reach the target position based on the current velocity.
			Vec3 targetDirection = targetPosition - pos.value;
			double t = (targetDirection.x * vel.value.x + targetDirection.z * vel.value.z) / (vel.value.x * vel.value.x + vel.value.z * vel.value.z);

			// Correct the position to where the bullet would be at the end of its trajectory.
			pos.value += vel.value * t;

			// Mark the bullet as dead.
			KillBullet(registry, entity);

			return;
		}
	});
}