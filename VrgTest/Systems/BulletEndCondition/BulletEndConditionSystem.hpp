#pragma once

#include "Core/Maths.hpp"
#include <entt/entity/registry.hpp>

void CheckBulletEndCondition(entt::registry& registry, const Vec3& shooterPosition, const Vec3& targetPosition, const double targetHorizontalDistanceSqrt);