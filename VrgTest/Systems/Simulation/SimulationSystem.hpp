#pragma once

#include <entt/entity/registry.hpp>
#include "Core/Maths.hpp"

void TryStartSimulation(entt::registry& registry, const Vec3& shooterPosition, double bulletMass, double muzzleVelocity);
void CheckSimulationEndCondition(entt::registry& registry);
bool GetIsSimulationActive(entt::registry& registry);
void IterateSimulation(entt::registry& registry);