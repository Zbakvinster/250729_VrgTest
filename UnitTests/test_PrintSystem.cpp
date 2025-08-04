#include "catch_amalgamated.hpp"
#include <entt/entity/registry.hpp>
#include <sstream>
#include <iostream>

#include "../VrgTest/Systems/Print/PrintSystem.hpp"
#include "../VrgTest/Components/Position.hpp"

TEST_CASE("PrintPositions prints correct positions", "[PrintSystem]") {
    entt::registry registry;

    auto e1 = registry.create();
    registry.emplace<Position>(e1, Position{ {1.0, 2.0, 3.0} });

    auto e2 = registry.create();
    registry.emplace<Position>(e2, Position{ {-1.5, 0.0, 4.2} });

    // Zachytíme výstup do std::cout
    std::ostringstream buffer;
    std::streambuf* oldCout = std::cout.rdbuf();
    std::cout.rdbuf(buffer.rdbuf());

    PrintPositions(registry);

    // Obnovíme std::cout
    std::cout.rdbuf(oldCout);

    std::string output = buffer.str();

    // Ovìøíme, že výstup obsahuje obì pozice
    CHECK(output.find("1, 2, 3") != std::string::npos);
    CHECK(output.find("-1.5, 0, 4.2") != std::string::npos);
}
