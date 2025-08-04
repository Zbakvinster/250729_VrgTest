#include "VelocityApplierSystem.hpp"
#include <entt/entity/registry.hpp>
#include "Components/Position.hpp"
#include "Components/Velocity.hpp"

void ApplyVelocity(entt::registry& registry, double dt)
{
	// Get a view of entities with Position and Velocity components.
	auto view = registry.view<Position, Velocity>();

	// Iterate over each entity in the view and update its position based on its velocity.
	view.each([dt](Position& pos, const Velocity& vel)
	{
		pos.value += vel.value * dt;
	});
}