// VrgTest.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <entt/entity/registry.hpp>
#include <entt/entt.hpp>




struct Position {
    float x, y, z;
};

struct Velocity {
    float x, y, z;
};

struct Mass {
    float value;
};




void update_physics(entt::registry& registry, float dt) {
    const float gravity = -9.81f;
    auto view = registry.view<Position, Velocity>();
    view.each([dt, gravity](Position& pos, Velocity& vel) {
        vel.z += gravity * dt;
        pos.x += vel.x * dt;
        pos.y += vel.y * dt;
        pos.z += vel.z * dt;
        });
}



int main()
{
    std::cout << "Hello World!\n";

	entt::registry registry;



    auto shooter = registry.create();
    registry.emplace<Position>(shooter, 0.0f, 0.0f, 0.0f);

    auto target = registry.create();
    registry.emplace<Position>(target, 100.0f, 0.0f, 10.0f);

    auto projectile = registry.create();
    registry.emplace<Position>(projectile, 0.0f, 0.0f, 0.0f);
    registry.emplace<Velocity>(projectile, 0.0f, 0.0f, 0.0f);
    registry.emplace<Mass>(projectile, 0.01f);


	// Simulate a shot
	auto& pos = registry.get<Position>(projectile);
	auto& vel = registry.get<Velocity>(projectile);
	vel.x = 50.0f; // Initial horizontal velocity
	vel.y = 0.0f;  // No vertical velocity
	vel.z = 0.0f;  // No vertical velocity
	const float dt = 0.1f; // Time step for simulation
	for (int i = 0; i < 100; ++i) {
		update_physics(registry, dt);
		std::cout << "Projectile Position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
	}
	return 0;
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
