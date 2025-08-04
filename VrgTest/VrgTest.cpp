#include <Systems/Simulation/SimulationSystem.hpp>
#include <entt/entity/registry.hpp>
#include <Components/SimulationData.hpp>
#include <Core/Maths.hpp>
#include <Systems/VelocityApplier/VelocityApplierSystem.hpp>
#include <Systems/BulletEndCondition/BulletEndConditionSystem.hpp>
#include <Systems/DeadBulletSolver/DeadBulletSolverSystem.hpp>
#include <iostream>
#include <Components/ActiveSimulation.hpp>
#include <Systems/Gravity/GravitySystem.hpp>
#include <Systems/Drag/DragForceSystem.hpp>
#include <Systems/Physics/PhysicsSystem.hpp>
#include <Systems/TargetMinDistanceCheck/TargetMinDistanceCheckSystem.hpp>
#include <regex>

bool parseVec3(const std::string& str, Vec3& vec) {
    std::regex vec3Regex(R"(^\s*([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+)\s*$)");
    std::smatch match;
    if (std::regex_match(str, match, vec3Regex)) {
        vec.x = std::stod(match[1]);
        vec.y = std::stod(match[2]);
        vec.z = std::stod(match[3]);
        return true;
    }
    return false;
}

bool parseDouble(const std::string& str, double& value) {
    std::regex doubleRegex(R"(^\s*([-+]?\d*\.?\d+)\s*$)");
    std::smatch match;
    if (std::regex_match(str, match, doubleRegex)) {
        value = std::stod(match[1]);
        return true;
    }
    return false;
}

int main(int argc, char* argv[])
{
    //////////////////////////////////////////////////////////////////////////////////////////
	// Input validation and parsing
    //////////////////////////////////////////////////////////////////////////////////////////

	// Check if the correct number of arguments is provided.
    if (argc != 6) {
		std::cerr << "Error: Invalid number of arguments." << std::endl;
        std::cerr << "Usage: " << " shooterPosition(x,y,z) targetPosition(x,y,z) muzzleVelocity bulletMass dt\n";
		std::cerr << "Example: " << " \"10,50,20\" \"200,350,100\" 850 0.0162 0.01\n";
        return 1;
    }

	// Parse the input arguments.
    Vec3 shooterPosition, targetPosition;
    double muzzleVelocity, bulletMass, dt;

	// Parse the shooter position.
    if (!parseVec3(argv[1], shooterPosition)) {
		std::cerr << "Error: Invalid shooter position format. Expected format: x,y,z" << std::endl;
        return 1;
    }

	// Parse the target position.
    if (!parseVec3(argv[2], targetPosition)) {
		std::cerr << "Error: Invalid target position format. Expected format: x,y,z" << std::endl;
        return 1;
    }

	// Parse the muzzle velocity, bullet mass, and time step.
    if (!parseDouble(argv[3], muzzleVelocity)) {
		std::cerr << "Error: Invalid muzzle velocity format. Expected a number." << std::endl;
        return 1;
    }

	// Parse the bullet mass.
    if (!parseDouble(argv[4], bulletMass)) {
		std::cerr << "Error: Invalid bullet mass format. Expected a number." << std::endl;
        return 1;
    }

	// Parse the time step.
    if (!parseDouble(argv[5], dt)) {
		std::cerr << "Error: Invalid time step format. Expected a number." << std::endl;
        return 1;
    }

	// Shooter can't be under ground.
	if (shooterPosition.y < 0.0)
	{
		std::cerr << "Error: Shooter position cannot be below ground level." << std::endl;
		return 1;
	}

	// Target can't be under ground.
	if (targetPosition.y < 0.0)
	{
		std::cerr << "Error: Target position cannot be below ground level." << std::endl;
		return 1;
	}

    //////////////////////////////////////////////////////////////////////////////////////////
	// Simulation Initialization
    //////////////////////////////////////////////////////////////////////////////////////////

	Vec3 toTarget = targetPosition - shooterPosition;

	double targetHorizontalDistanceSqrt = toTarget.x * toTarget.x + toTarget.z * toTarget.z;

	// Create an entity registry.
	entt::registry registry;

    double lowAngle =
        std::atan2(toTarget.y, std::sqrt(targetHorizontalDistanceSqrt))
        * 180.0
        / M_PI;

	// Initialize the simulation data.
    const auto simulationDataEntity = registry.create();
    registry.emplace<SimulationData>(
        simulationDataEntity,
		std::atan2(toTarget.z, toTarget.x) * 180.0 / M_PI, // Theta angle in degrees
        lowAngle,
        90.0, // High angle
        5.0, // Angle step
		lowAngle, // Low angle as the initial best angle
		std::numeric_limits<double>::max(), // Minimum distance to the target
		std::numeric_limits<double>::max()); // Previous minimum distance to the target

	const auto activeSimulationEntity = registry.create();
    registry.emplace<ActiveSimulation>(activeSimulationEntity, true); // Start the simulation as active

    while (true)
    {
        TryStartSimulation(registry, shooterPosition, bulletMass, muzzleVelocity);
		ApplyPhysics(registry, dt);
		ApplyVelocity(registry, dt);
        CheckBulletEndCondition(registry, shooterPosition, targetPosition, targetHorizontalDistanceSqrt);
		CheckTargetMinDistance(registry, targetPosition);
		SolveDeadBullets(registry);
		CheckSimulationEndCondition(registry);

		// Check if the simulation is still active.
		if (!GetIsSimulationActive(registry))
			break; // Exit the loop if the simulation is no longer active.

		// Iterate the simulation.
		IterateSimulation(registry);
    }

	auto simulationDataView = registry.view<SimulationData>();

	if (simulationDataView.empty())
	{
		std::cerr << "Error: No simulation data available." << std::endl;
		return 1;
	}

	const SimulationData& simulationData = registry.get<SimulationData>(simulationDataView.front());

	std::cout << "Nejlepsi uhel: theta = " << simulationData.theta
		<< ", phi = " << simulationData.bestAngle << " stupnu\n";
	std::cout << "Nejblizsi vzdalenost k cili: " << simulationData.minDistance << " m\n";

	return 0;
}
