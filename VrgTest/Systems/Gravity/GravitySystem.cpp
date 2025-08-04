#include "GravitySystem.hpp"
#include <entt/entity/registry.hpp>
#include "Components/Velocity.hpp"
#include "Components/Mass.hpp"

constexpr double gravity = -9.81; // Gravitational acceleration in m/s^2

void ApplyGravity(entt::registry& registry, float dt)
{
	// Get a view of entities with Velocity and Mass components.
	auto view = registry.view<Velocity, Mass>();

	// Iterate over each entity in the view and update its velocity based on gravity.
	view.each([dt](Velocity& vel, const Mass& mass)
	{
		// Update the vertical component of velocity based on gravity.
		vel.value.y += gravity * dt;
	});
}