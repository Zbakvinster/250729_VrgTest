#include "DragForceSystem.hpp"
#include <entt/entity/registry.hpp>
#include "Components/Velocity.hpp"
#include "Components/Mass.hpp"

constexpr double airDensity = 1.225; // Density of air at sea level in kg/m^3
constexpr double Cd = 0.295; // Drag coefficient for a bullet (typical value for a 7.62mm bullet)
constexpr double bulletRadius = 0.00782; // Radius of a 7.62mm bullet in meters (0.00782 m = 7.62 mm / 2)
constexpr double bulletArea = M_PI * bulletRadius * bulletRadius; // Cross-sectional area of a 7.62mm bullet in m^2

void ApplyDragForce(entt::registry& registry, float dt)
{
	auto view = registry.view<Velocity, Mass>();

	view.each([dt](Velocity& vel, const Mass& mass)
	{
		double vLen = vel.value.length(); // Speed of the bullet
		Vec3 dragDir = -vel.value.normalized(); // Direction of the drag force (opposite to velocity)
		double FdMag = 0.5 * airDensity * Cd * bulletArea * vLen * vLen; // Magnitude of the drag force
		Vec3 Fd = dragDir * FdMag; // Drag force vector

		Vec3 acc = Fd / mass.value; // Acceleration due to drag force (F = ma, so a = F/m)

		vel.value += acc * dt; // Update velocity based on drag force
	});
}