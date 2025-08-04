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
        //shooterPosition, // Shooter position
        //targetPosition, // Target position
        //850.0, // Muzzle velocity
        //0.0162, // Bullet mass
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
		//ApplyGravity(registry, dt);
		//ApplyDragForce(registry, dt);
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







    /*if (argc != 10) {
        std::cerr << "Použití: " << argv[0] << " shooterX shooterY shooterZ targetX targetY targetZ muzzleVelocity mass dt\n";
        return 1;
    }

    Vec3 shooter = { std::atof(argv[1]), std::atof(argv[2]), std::atof(argv[3]) };
    Vec3 target = { std::atof(argv[4]), std::atof(argv[5]), std::atof(argv[6]) };
    double muzzleVelocity = std::atof(argv[7]);
    double mass = std::atof(argv[8]);
    double dt = std::atof(argv[9]);

    SimulationResult result = findBestTrajectory(shooter, target, muzzleVelocity, mass, dt);
    std::cout << "Nejlepsi uhel: theta = " << std::atan2(target.z - shooter.z, target.x - shooter.x) * 180.0 / M_PI
        << ", phi = " << result.phi << " stupnu\n";
    std::cout << "Nejblizsi vzdalenost k cili: " << result.minDistance << " m\n";

    return 0;*/
}




























//#include <iostream>
//#include <cmath>
//#include <limits>
//#include <future>
//#include <vector>
//#include <algorithm>
//#include <thread>
//#include <cstdlib>
//
//#ifndef M_PI
//#define M_PI 3.14159265358979323846
//#endif
//
//constexpr double g = 9.81;
//constexpr double airDensity = 1.225;
//constexpr double Cd = 0.295;
//constexpr double bulletRadius = 0.00782;
//constexpr double bulletArea = M_PI * bulletRadius * bulletRadius;
//constexpr double minDistanceImprovementThreshold = 0.001; // Minimální zlepšení minDistance mezi iteracemi
//
//struct Vec3 {
//    double x, y, z;
//
//    Vec3 operator+(const Vec3& v) const { return { x + v.x, y + v.y, z + v.z }; }
//    Vec3 operator-(const Vec3& v) const { return { x - v.x, y - v.y, z - v.z }; }
//    Vec3 operator-() const { return { -x, -y, -z }; }
//    Vec3 operator*(double s)     const { return { x * s, y * s, z * s }; }
//    Vec3 operator/(double s)     const { return { x / s, y / s, z / s }; }
//
//    double length() const {
//        return std::sqrt(x * x + y * y + z * z);
//    }
//
//    Vec3 normalized() const {
//        double len = length();
//        return (len > 0.0) ? (*this) / len : Vec3{ 0.0, 0.0, 0.0 };
//    }
//};
//
//struct SimulationResult {
//    double minDistance;
//    Vec3 closestPosition;
//    double phi;
//};
//
//SimulationResult simulateTrajectory(Vec3 shooter, Vec3 target, Vec3 direction, double muzzleVelocity, double mass, double phi, double dt = 0.01) {
//    Vec3 pos = shooter;
//    Vec3 vel = direction.normalized() * muzzleVelocity;
//
//    double minDistance = (pos - target).length();
//    Vec3 closestPos = pos;
//
//    Vec3 toTarget = target - shooter;
//    double targetFlatDist = std::sqrt(toTarget.x * toTarget.x + toTarget.z * toTarget.z);
//
//    while (true) {
//        Vec3 Fg = { 0.0, -mass * g, 0.0 };
//
//        double vLen = vel.length();
//        Vec3 dragDir = -vel.normalized();
//        double FdMag = 0.5 * airDensity * Cd * bulletArea * vLen * vLen;
//        Vec3 Fd = dragDir * FdMag;
//
//        Vec3 F = Fg + Fd;
//        Vec3 acc = F / mass;
//
//        vel = vel + acc * dt;
//        Vec3 nextPos = pos + vel * dt;
//
//        double dist = (nextPos - target).length();
//        if (dist < minDistance) {
//            minDistance = dist;
//            closestPos = nextPos;
//        }
//
//        pos = nextPos;
//
//        double flatDistanceNow = std::sqrt((pos.x - shooter.x) * (pos.x - shooter.x) + (pos.z - shooter.z) * (pos.z - shooter.z));
//
//        if (flatDistanceNow > targetFlatDist) {
//            Vec3 toTargetHoriz = Vec3{ target.x - pos.x, 0, target.z - pos.z };
//            Vec3 velHoriz = Vec3{ vel.x, vel.y, vel.z };
//
//            double velLenSquared = vel.x * vel.x + vel.y * vel.y + vel.z * vel.z;
//            if (velLenSquared > 1e-6) {
//                double t = (toTargetHoriz.x * vel.x + toTargetHoriz.z * vel.z) / (vel.x * vel.x + vel.z * vel.z);
//                Vec3 proj = pos + vel * t;
//
//                double projDist = (proj - target).length();
//                if (projDist < minDistance) {
//                    minDistance = projDist;
//                    closestPos = proj;
//                }
//            }
//            break;
//        }
//
//        if (pos.y <= 0.0) break;
//    }
//
//    return { minDistance, closestPos, phi };
//}
//
//Vec3 fromSpherical(double thetaDeg, double phiDeg) {
//    double theta = thetaDeg * M_PI / 180.0;
//    double phi = phiDeg * M_PI / 180.0;
//
//    double x = cos(phi) * cos(theta);
//    double y = sin(phi);
//    double z = cos(phi) * sin(theta);
//
//    return Vec3{ x, y, z };
//}
//
//SimulationResult findBestTrajectory(Vec3 shooter, Vec3 target, double muzzleVelocity, double mass, double dt) {
//    Vec3 toTarget = target - shooter;
//    double theta = std::atan2(toTarget.z, toTarget.x) * 180.0 / M_PI;
//    double flatDist = std::sqrt(toTarget.x * toTarget.x + toTarget.z * toTarget.z);
//    double phiGuess = std::atan2(toTarget.y, flatDist) * 180.0 / M_PI;
//
//    double lo = phiGuess;
//    double hi = 90.0;
//    unsigned int divisions = std::max(4u, std::thread::hardware_concurrency());
//
//    SimulationResult bestResult{ std::numeric_limits<double>::max(), {}, 0.0 };
//    double lastBestDistance = std::numeric_limits<double>::max();
//
//    while (true) {
//        std::vector<std::future<SimulationResult>> futures;
//        double step = (hi - lo) / (divisions - 1);
//
//        for (unsigned int i = 0; i < divisions; ++i) {
//            double phi = lo + i * step;
//            Vec3 dir = fromSpherical(theta, phi);
//            futures.push_back(std::async(std::launch::async, simulateTrajectory, shooter, target, dir, muzzleVelocity, mass, phi, dt));
//        }
//
//        std::vector<SimulationResult> results;
//        for (auto& fut : futures) {
//            results.push_back(fut.get());
//        }
//
//        bestResult = results[0];
//        for (size_t i = 1; i < results.size(); ++i) {
//            if (results[i].minDistance < bestResult.minDistance) {
//                bestResult = results[i];
//            }
//            else {
//                break; // první lokální minimum nalezeno
//            }
//        }
//
//        if (std::abs(lastBestDistance - bestResult.minDistance) <= minDistanceImprovementThreshold)
//            break;
//
//        lastBestDistance = bestResult.minDistance;
//
//        int idx = -1;
//        for (size_t i = 0; i < results.size(); ++i) {
//            if (results[i].phi == bestResult.phi) {
//                idx = static_cast<int>(i);
//                break;
//            }
//        }
//
//        int lower = std::max(0, idx - 1);
//        int upper = std::min(static_cast<int>(divisions) - 1, idx + 1);
//
//        lo = lo + lower * step;
//        hi = lo + (upper - lower) * step;
//    }
//
//    return bestResult;
//}
//
//int main(int argc, char* argv[]) {
//    if (argc != 10) {
//        std::cerr << "Použití: " << argv[0] << " shooterX shooterY shooterZ targetX targetY targetZ muzzleVelocity mass dt\n";
//        return 1;
//    }
//
//    Vec3 shooter = { std::atof(argv[1]), std::atof(argv[2]), std::atof(argv[3]) };
//    Vec3 target = { std::atof(argv[4]), std::atof(argv[5]), std::atof(argv[6]) };
//    double muzzleVelocity = std::atof(argv[7]);
//    double mass = std::atof(argv[8]);
//    double dt = std::atof(argv[9]);
//
//    SimulationResult result = findBestTrajectory(shooter, target, muzzleVelocity, mass, dt);
//    std::cout << "Nejlepsi uhel: theta = " << std::atan2(target.z - shooter.z, target.x - shooter.x) * 180.0 / M_PI
//        << ", phi = " << result.phi << " stupnu\n";
//    std::cout << "Nejblizsi vzdalenost k cili: " << result.minDistance << " m\n";
//
//    return 0;
//}


























/*
// VrgTest.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <sstream>
#include <entt/entity/registry.hpp>
#include <entt/entt.hpp>



struct Vector3
{
	float x, y, z;

    Vector3() : x(0), y(0), z(0) {}
    Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vector3 operator+(const Vector3& other) const { return Vector3(x + other.x, y + other.y, z + other.z); }
    Vector3 operator-(const Vector3& other) const { return Vector3(x - other.x, y - other.y, z - other.z); }
    Vector3 operator*(float scalar) const { return Vector3(x * scalar, y * scalar, z * scalar); }
    Vector3 operator/(float scalar) const { return Vector3(x / scalar, y / scalar, z / scalar); }

    Vector3& operator+=(const Vector3& other)
    {
        x += other.x; y += other.y; z += other.z;
        return *this;
    }

    Vector3& operator-=(const Vector3& other)
    {
        x -= other.x; y -= other.y; z -= other.z;
        return *this;
    }

    Vector3& operator*=(float scalar)
    {
        x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }

    Vector3& operator/=(float scalar)
    {
        x /= scalar; y /= scalar; z /= scalar;
        return *this;
    }

	float length() const { return std::sqrt(x * x + y * y + z * z); }

	Vector3 normalize() const
    {
		float len = length();
		return Vector3(x / len, y / len, z / len);
	}
};

struct Position
{
    Vector3 value;
};

struct Velocity
{
	Vector3 value;
};

struct Mass
{
    float value;
};

// Constants for physics simulation
constexpr double G = -9.81;                  // Gravitational acceleration in m/s^2
constexpr double AIR_DENSITY = 1.225;       // Density of air at sea level in kg/m^3
constexpr double C_d = 0.295;               // Drag coefficient for a bullet (typical value for a 7.62mm bullet)
constexpr double BULLET_AREA = 0.000456;    // Cross-sectional area of a 7.62mm bullet in m^2 (calculated as π * (d/2)^2, where d is the diameter in meters)

















void findBestTrajectory(Vec3 shooter, Vec3 target, double muzzleVelocity, double mass) {
    Vec3 flatDir = target - shooter;
    flatDir.y = 0.0;
    double theta = std::atan2(flatDir.z, flatDir.x) * 180.0 / M_PI;

    double bestPhi = 0.0;
    double bestDist = std::numeric_limits<double>::max();

    for (double phi = 5.0; phi <= 60.0; phi += 0.5) {
        Vec3 dir = fromSpherical(theta, phi);

        auto result = simulateTrajectory(shooter, target, dir, muzzleVelocity, mass);

        if (result.minDistance < bestDist) {
            bestDist = result.minDistance;
            bestPhi = phi;
        }
    }

    std::cout << "Nejlepsi uhel: theta = " << theta << ", phi = " << bestPhi << " stupnu\n";
    std::cout << "Nejblizsi vzdalenost k cili: " << bestDist << " m\n";
}





void update_physics(entt::registry& registry, float dt) {
    const float gravity = -9.81f;
    auto view = registry.view<Position, Velocity>();
    view.each([dt, gravity](Position& pos, Velocity& vel) {
        vel.value.z += gravity * dt;
        pos.value.x += vel.value.x * dt;
        pos.value.y += vel.value.y * dt;
        pos.value.z += vel.value.z * dt;
        });
}



bool TryParseStringToVector(const std::string str, Vector3& vec)
{
	// Create a copy of the string to avoid modifying the original.
	std::string strCopy(str);

	// Prepare string for istringstream - replace commas with spaces.
    std::replace(strCopy.begin(), strCopy.end(), ',', ' ');

	// Create an istringstream object to parse the string.
    std::istringstream iss(strCopy);

	// Populate the vector with the parsed values.
    iss >> vec.x >> vec.y >> vec.z;

	// Check if the parsing was successful.
    return !iss.fail();
}


int main(int argc, char* argv[])
{
    Vector3 shooter = { 10, 50, 20 };
    Vector3 target = { 200, 350, 100 };
    double muzzleVelocity = 850.0; // m/s
    double mass = 0.0162;          // kg

    findBestTrajectory(shooter, target, muzzleVelocity, mass);








    return 0;












	// Check if the correct number of arguments is provided.
    if (argc != 6)
    {
        std::cerr << "Usage: sX,sY,sZ tX,tY,tZ BulletSpeed BulletWeight SimulationTimestep\n";
        return 1;
    }

	// Create an instance of the registry.
    entt::registry registry;

	// Create entities for shooter, target, and projectile.
    //auto shooter = registry.create();
    auto target = registry.create();
    auto projectile = registry.create();

    Vector3 vec;
    if (!TryParseStringToVector(argv[1], vec))
    {
        std::cerr << "Usage: sX,sY,sZ tX,tY,tZ BulletSpeed BulletWeight SimulationTimestep\n";
        return 1;
    }

    //registry.emplace<Position>(shooter, vec);
    registry.emplace<Position>(projectile, vec);

    if (!TryParseStringToVector(argv[2], vec))
    {
        std::cerr << "Usage: sX,sY,sZ tX,tY,tZ BulletSpeed BulletWeight SimulationTimestep\n";
        return 1;
    }

    registry.emplace<Position>(target, vec);

    float value;
    try
    {
		std::string str = argv[3];
        size_t idx;
        value = std::stof(str, &idx);
        if (idx != str.size())
        {
            std::cerr << "Usage: sX,sY,sZ tX,tY,tZ BulletSpeed BulletWeight SimulationTimestep\n";
            return 1;
        }
    }
    catch (...)
    {
        std::cerr << "Usage: sX,sY,sZ tX,tY,tZ BulletSpeed BulletWeight SimulationTimestep\n";
        return 1;
    }

	// TODO: Add muzzle velocity to the projectile's velocity component.
    registry.emplace<Velocity>(projectile, 0.0f, 0.0f, 0.0f);

    try
    {
        std::string str = argv[4];
        size_t idx;
        value = std::stof(str, &idx);
        if (idx != str.size())
        {
            std::cerr << "Usage: sX,sY,sZ tX,tY,tZ BulletSpeed BulletWeight SimulationTimestep\n";
            return 1;
        }
    }
    catch (...)
    {
        std::cerr << "Usage: sX,sY,sZ tX,tY,tZ BulletSpeed BulletWeight SimulationTimestep\n";
        return 1;
    }

    registry.emplace<Mass>(projectile, value);

    float dt;
    try
    {
        std::string str = argv[4];
        size_t idx;
        dt = std::stof(str, &idx);
        if (idx != str.size())
        {
            std::cerr << "Usage: sX,sY,sZ tX,tY,tZ BulletSpeed BulletWeight SimulationTimestep\n";
            return 1;
        }
    }
    catch (...)
    {
        std::cerr << "Usage: sX,sY,sZ tX,tY,tZ BulletSpeed BulletWeight SimulationTimestep\n";
        return 1;
    }


    return 0;








	//// Simulate a shot
	//auto& pos = registry.get<Position>(projectile);
	//auto& vel = registry.get<Velocity>(projectile);
	//vel.x = 50.0f; // Initial horizontal velocity
	//vel.y = 0.0f;  // No vertical velocity
	//vel.z = 0.0f;  // No vertical velocity
	//const float dt = 0.1f; // Time step for simulation
	//for (int i = 0; i < 100; ++i) {
	//	update_physics(registry, dt);
	//	std::cout << "Projectile Position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
	//}
	//return 0;
}
// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
*/
