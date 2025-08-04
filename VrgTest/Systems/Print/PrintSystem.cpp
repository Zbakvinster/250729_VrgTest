#include "PrintSystem.hpp"
#include <entt/entity/registry.hpp>
#include <Components/Position.hpp>
#include <iostream>

void PrintPositions(const entt::registry& registry)
{
	// Get a view of entities with Position components.
	auto view = registry.view<Position>();

	// Iterate over each entity in the view and print its position.
	view.each([](const Position& pos) {
		std::cout << "" << pos.value.x << ", " << pos.value.y << ", " << pos.value.z << "\n";
	});
}