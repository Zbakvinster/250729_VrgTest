#pragma once

#include <Core/Maths.hpp>

// Run Simulation function with bool as return value and double as out parameter for best angle.
bool RunSimulation(const Vec3& shooterPosition, const Vec3& targetPosition, double muzzleVelocity, double bulletMass, double dt, double& bestAngleOut, double& minDistanceOut);
void PrintBulletTrajectory(const Vec3& shooterPosition, const Vec3& targetPosition, double elevationAngle, double muzzleVelocity, double bulletMass, double dt);