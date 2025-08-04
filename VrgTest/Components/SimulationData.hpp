#pragma once

struct SimulationData
{
	double theta;
	double lowAngle = 0.0; // Lower angle bound for the simulation
	double highAngle = 90.0; // Upper angle bound for the simulation
	double angleStep; // Step size for angle increments
	double bestAngle; // Best angle found during the simulation
	double minDistance = std::numeric_limits<double>::max(); // Minimum distance to the target
	//double prevMinDistance = std::numeric_limits<double>::max(); // Previous minimum distance to the target
	double secondMinDistanceDiff = std::numeric_limits<double>::max(); // Difference between the best and second-best minimum distances
};