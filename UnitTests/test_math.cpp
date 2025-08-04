#include "catch_amalgamated.hpp"
#include "../VrgTest/Core/Maths.hpp"

using Catch::Approx;

constexpr double epsilon = 1e-9;

TEST_CASE("Vec3 basic arithmetic", "[Vec3]") {
    Vec3 a{ 1.0, 2.0, 3.0 };
    Vec3 b{ 4.0, 5.0, 6.0 };

    SECTION("Addition") {
        Vec3 result = a + b;
        REQUIRE(result.x == Approx(5.0));
        REQUIRE(result.y == Approx(7.0));
        REQUIRE(result.z == Approx(9.0));
    }

    SECTION("Subtraction") {
        Vec3 result = b - a;
        REQUIRE(result.x == Approx(3.0));
        REQUIRE(result.y == Approx(3.0));
        REQUIRE(result.z == Approx(3.0));
    }

    SECTION("Negation") {
        Vec3 result = -a;
        REQUIRE(result.x == Approx(-1.0));
        REQUIRE(result.y == Approx(-2.0));
        REQUIRE(result.z == Approx(-3.0));
    }
}

TEST_CASE("Vec3 scalar and element-wise multiplication", "[Vec3]") {
    Vec3 v{ 1.0, -2.0, 3.0 };
    double scalar = 2.0;

    SECTION("Scalar multiplication") {
        Vec3 result = v * scalar;
        REQUIRE(result.x == Approx(2.0));
        REQUIRE(result.y == Approx(-4.0));
        REQUIRE(result.z == Approx(6.0));
    }

    SECTION("Element-wise multiplication") {
        Vec3 w{ 2.0, 3.0, 4.0 };
        Vec3 result = v * w;
        REQUIRE(result.x == Approx(2.0));
        REQUIRE(result.y == Approx(-6.0));
        REQUIRE(result.z == Approx(12.0));
    }
}

TEST_CASE("Vec3 scalar and element-wise division", "[Vec3]") {
    Vec3 v{ 2.0, 4.0, 8.0 };
    double scalar = 2.0;

    SECTION("Scalar division") {
        Vec3 result = v / scalar;
        REQUIRE(result.x == Approx(1.0));
        REQUIRE(result.y == Approx(2.0));
        REQUIRE(result.z == Approx(4.0));
    }

    SECTION("Element-wise division") {
        Vec3 w{ 2.0, 2.0, 2.0 };
        Vec3 result = v / w;
        REQUIRE(result.x == Approx(1.0));
        REQUIRE(result.y == Approx(2.0));
        REQUIRE(result.z == Approx(4.0));
    }
}

TEST_CASE("Vec3 compound assignment", "[Vec3]") {
    SECTION("operator+=") {
        Vec3 v{ 1, 2, 3 };
        v += Vec3{ 4, 5, 6 };
        REQUIRE(v.x == Approx(5));
        REQUIRE(v.y == Approx(7));
        REQUIRE(v.z == Approx(9));
    }

    SECTION("operator-=") {
        Vec3 v{ 4, 5, 6 };
        v -= Vec3{ 1, 2, 3 };
        REQUIRE(v.x == Approx(3));
        REQUIRE(v.y == Approx(3));
        REQUIRE(v.z == Approx(3));
    }

    SECTION("operator*=(scalar)") {
        Vec3 v{ 1, 2, 3 };
        v *= 2.0;
        REQUIRE(v.x == Approx(2));
        REQUIRE(v.y == Approx(4));
        REQUIRE(v.z == Approx(6));
    }

    SECTION("operator*=(vector)") {
        Vec3 v{ 1, 2, 3 };
        v *= Vec3{ 2, 2, 2 };
        REQUIRE(v.x == Approx(2));
        REQUIRE(v.y == Approx(4));
        REQUIRE(v.z == Approx(6));
    }

    SECTION("operator/=(scalar)") {
        Vec3 v{ 2, 4, 6 };
        v /= 2.0;
        REQUIRE(v.x == Approx(1));
        REQUIRE(v.y == Approx(2));
        REQUIRE(v.z == Approx(3));
    }

    SECTION("operator/=(vector)") {
        Vec3 v{ 2, 4, 6 };
        v /= Vec3{ 2, 2, 2 };
        REQUIRE(v.x == Approx(1));
        REQUIRE(v.y == Approx(2));
        REQUIRE(v.z == Approx(3));
    }
}

TEST_CASE("Vec3 length and normalization", "[Vec3]") {
    Vec3 v{ 3.0, 4.0, 0.0 };

    SECTION("Length squared") {
        REQUIRE(v.lengthSqrt() == Approx(25.0));
    }

    SECTION("Length") {
        REQUIRE(v.length() == Approx(5.0));
    }

    SECTION("Normalized") {
        Vec3 norm = v.normalized();
        REQUIRE(norm.length() == Approx(1.0));
        REQUIRE(norm.x == Approx(0.6));
        REQUIRE(norm.y == Approx(0.8));
        REQUIRE(norm.z == Approx(0.0));
    }

    SECTION("Normalization of zero vector") {
        Vec3 zero{ 0.0, 0.0, 0.0 };
        Vec3 norm = zero.normalized();
        REQUIRE(norm.x == Approx(0.0));
        REQUIRE(norm.y == Approx(0.0));
        REQUIRE(norm.z == Approx(0.0));
    }
}

TEST_CASE("Vec3 from spherical coordinates", "[Vec3]") {
    SECTION("fromSpherical returns unit vector") {
        Vec3 v = Vec3::fromSpherical(0.0, 0.0);
        REQUIRE(v.x == Approx(1.0));
        REQUIRE(v.y == Approx(0.0));
        REQUIRE(v.z == Approx(0.0));
    }

    SECTION("fromSpherical at 90° phi (up)") {
        Vec3 v = Vec3::fromSpherical(45.0, 90.0);
        REQUIRE(v.x == Approx(0.0).margin(epsilon));
        REQUIRE(v.y == Approx(1.0).margin(epsilon));
        REQUIRE(v.z == Approx(0.0).margin(epsilon));
    }

    SECTION("fromSpherical at 180° theta, 0° phi (back)") {
        Vec3 v = Vec3::fromSpherical(180.0, 0.0);
        REQUIRE(v.x == Approx(-1.0).margin(epsilon));
        REQUIRE(v.y == Approx(0.0).margin(epsilon));
        REQUIRE(v.z == Approx(0.0).margin(epsilon));
    }
}
