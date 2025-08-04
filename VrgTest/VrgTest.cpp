#include <iostream>
#include <Core/Maths.hpp>
#include <regex>
#include <Simulator.hpp>

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

	double bestAngle;
	double minDistance;
	bool success = RunSimulation(
		shooterPosition,
		targetPosition,
		muzzleVelocity,
		bulletMass,
		dt,
		bestAngle,
		minDistance);

	std::cout.precision(8); // Set precision for floating-point output
	std::cout << std::fixed; // Use fixed-point notation for better readability

	std::cout << "With following parameters:\n"
		<< "Shooter position: (" << shooterPosition.x << ", " << shooterPosition.y << ", " << shooterPosition.z << ")\n"
		<< "Target position: (" << targetPosition.x << ", " << targetPosition.y << ", " << targetPosition.z << ")\n"
		<< "Muzzle velocity: " << muzzleVelocity << " m/s\n"
		<< "Bullet mass: " << bulletMass << " kg\n"
		<< "Time step: " << dt << " seconds\n\n";

	if (success) {
		std::cout << "Simulation completed successfully.\n";
		std::cout << "- Best angle: " << bestAngle << " degrees\n";
		std::cout << "- Minimum distance to target: " << minDistance << " meters\n";
	}
	else {
		std::cout << "Simulation failed to find a valid solution.\n";
	}

	std::cout << std::endl;

	PrintBulletTrajectory(
		shooterPosition,
		targetPosition,
		bestAngle,
		muzzleVelocity,
		bulletMass,
		dt);

	return 0;
}
