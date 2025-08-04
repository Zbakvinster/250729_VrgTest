#include "catch_amalgamated.hpp"
#include <entt/entity/registry.hpp>

#include "../VrgTest/Systems/BulletEndCondition/BulletEndConditionSystem.hpp"
#include "../VrgTest/Components/Position.hpp"
#include "../VrgTest/Components/Velocity.hpp"
#include "../VrgTest/Components/ActiveBullet.hpp"
#include "../VrgTest/Components/DeadBullet.hpp"

TEST_CASE("Bullet is killed when it falls below ground", "[BulletEndConditionSystem]") {
    entt::registry registry;

    auto bullet = registry.create();
    registry.emplace<Position>(bullet, Vec3{ 0.0, -1.0, 0.0 });
    registry.emplace<Velocity>(bullet, Vec3{ 1.0, 0.0, 0.0 });
    registry.emplace<ActiveBullet>(bullet);

    Vec3 shooterPos{ 0.0, 0.0, 0.0 };
    Vec3 targetPos{ 10.0, 0.0, 0.0 };
    double targetDistSqr = 100.0;

    CheckBulletEndCondition(registry, shooterPos, targetPos, targetDistSqr);

    REQUIRE_FALSE(registry.any_of<ActiveBullet>(bullet));
    REQUIRE_FALSE(registry.any_of<Velocity>(bullet));
    REQUIRE(registry.all_of<DeadBullet>(bullet));
}

TEST_CASE("Bullet is killed when overshooting target", "[BulletEndConditionSystem]") {
    entt::registry registry;

    auto bullet = registry.create();
    registry.emplace<Position>(bullet, Vec3{ 11.0, 1.0, 0.0 });
    registry.emplace<Velocity>(bullet, Vec3{ 1.0, 0.0, 0.0 });
    registry.emplace<ActiveBullet>(bullet);

    Vec3 shooterPos{ 0.0, 0.0, 0.0 };
    Vec3 targetPos{ 10.0, 0.0, 0.0 };
    double targetDistSqr = 100.0;

    CheckBulletEndCondition(registry, shooterPos, targetPos, targetDistSqr);

    REQUIRE_FALSE(registry.any_of<ActiveBullet>(bullet));
    REQUIRE_FALSE(registry.any_of<Velocity>(bullet));
    REQUIRE(registry.all_of<DeadBullet>(bullet));

    auto& pos = registry.get<Position>(bullet).value;
    REQUIRE(pos.x < 11.0); // ensure bullet was moved before being killed
}

TEST_CASE("Bullet stays active when within bounds and above ground", "[BulletEndConditionSystem]") {
    entt::registry registry;

    auto bullet = registry.create();
    registry.emplace<Position>(bullet, Vec3{ 5.0, 1.0, 0.0 });
    registry.emplace<Velocity>(bullet, Vec3{ 1.0, 0.0, 0.0 });
    registry.emplace<ActiveBullet>(bullet);

    Vec3 shooterPos{ 0.0, 0.0, 0.0 };
    Vec3 targetPos{ 10.0, 0.0, 0.0 };
    double targetDistSqr = 100.0;

    CheckBulletEndCondition(registry, shooterPos, targetPos, targetDistSqr);

    REQUIRE(registry.all_of<ActiveBullet>(bullet));
    REQUIRE(registry.all_of<Velocity>(bullet));
    REQUIRE_FALSE(registry.any_of<DeadBullet>(bullet));
}
