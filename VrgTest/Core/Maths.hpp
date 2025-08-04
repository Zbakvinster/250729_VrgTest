#pragma once

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Vec3 {
    double x, y, z;

    Vec3 operator+(const Vec3& v) const { return { x + v.x, y + v.y, z + v.z }; }
	Vec3 operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vec3 operator-(const Vec3& v) const { return { x - v.x, y - v.y, z - v.z }; }
	Vec3 operator-=(const Vec3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Vec3 operator-() const { return { -x, -y, -z }; }
	Vec3 operator*(const Vec3& v) const { return { x * v.x, y * v.y, z * v.z }; }
    Vec3 operator*(double s) const { return { x * s, y * s, z * s }; }
    Vec3 operator*=(const Vec3& v) { x *= v.x; y *= v.y; z *= v.z; return *this; }
	Vec3 operator*=(double s) { x *= s; y *= s; z *= s; return *this; }
	Vec3 operator/(const Vec3& v) const { return { x / v.x, y / v.y, z / v.z }; }
    Vec3 operator/(double s) const { return { x / s, y / s, z / s }; }
	Vec3 operator/=(const Vec3& v) { x /= v.x; y /= v.y; z /= v.z; return *this; }
	Vec3 operator/=(double s) { x /= s; y /= s; z /= s; return *this; }
	bool operator==(const Vec3& v) const { return x == v.x && y == v.y && z == v.z; }

    double lengthSqrt() const { return x * x + y * y + z * z; }
	double length() const { return std::sqrt(lengthSqrt()); }

    Vec3 normalized() const {
        double len = length();
        return (len > 0.0) ? (*this) / len : Vec3{ 0.0, 0.0, 0.0 };
    }

	static Vec3 fromSpherical(double thetaDeg, double phiDeg) {
		double theta = thetaDeg * M_PI / 180.0;
		double phi = phiDeg * M_PI / 180.0;
		double x = std::cos(phi) * std::cos(theta);
		double y = std::sin(phi);
		double z = std::cos(phi) * std::sin(theta);
		return Vec3{ x, y, z };
	}
};