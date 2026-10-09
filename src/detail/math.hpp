#pragma once

/*
 * Fixed-size arithmetic adapted from CHAI3D 3.2.0 solely to retain the legacy
 * controller's operation order and singular-inverse behavior. No CHAI3D
 * headers, library, or runtime are required.
 *
 * Software License Agreement (BSD License)
 * Copyright (c) 2003-2016, CHAI3D. (www.chai3d.org)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of CHAI3D nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <array>
#include <cmath>
#include <cstddef>

namespace soft_actuator::detail {

struct Vec3 {
    std::array<double, 3> data{};
    Vec3() = default;
    Vec3(double x, double y, double z) : data{{x, y, z}} {}
    explicit Vec3(const std::array<double, 3>& v) : data(v) {}
    double& operator()(std::size_t i) { return data[i]; }
    double operator()(std::size_t i) const { return data[i]; }
    double x() const { return data[0]; }
    double y() const { return data[1]; }
    double z() const { return data[2]; }
    void set(double x, double y, double z) { data = {{x, y, z}}; }
    void zero() { set(0.0, 0.0, 0.0); }
    double length() const {
        return std::sqrt(data[0] * data[0] + data[1] * data[1] + data[2] * data[2]);
    }
    void normalize() {
        const double len = length();
        if (len == 0.0) return;
        const double factor = 1.0 / len;
        for (double& value : data) value *= factor;
    }
    Vec3& operator*=(double scale) {
        for (double& value : data) value *= scale;
        return *this;
    }
    Vec3& operator+=(const Vec3& other) {
        for (std::size_t i = 0; i < 3; ++i) data[i] += other(i);
        return *this;
    }
};
inline Vec3 operator+(const Vec3& a, const Vec3& b) {
    return Vec3(a(0) + b(0), a(1) + b(1), a(2) + b(2));
}
inline Vec3 operator-(const Vec3& a, const Vec3& b) {
    return Vec3(a(0) - b(0), a(1) - b(1), a(2) - b(2));
}
inline Vec3 operator*(const Vec3& a, double scale) {
    return Vec3(a(0) * scale, a(1) * scale, a(2) * scale);
}
inline Vec3 operator/(const Vec3& a, double scale) {
    return Vec3(a(0) / scale, a(1) / scale, a(2) / scale);
}
inline double clamp(double value, double low, double high) {
    // Preserve legacy NaN propagation; std::min/max composition need not do so.
    if (value < low) return low;
    if (value > high) return high;
    return value;
}
inline bool isFiniteVector(const Vec3& v) {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

struct Mat3 {
    std::array<double, 9> data{};
    double& operator()(std::size_t row, std::size_t col) { return data[3 * row + col]; }
    double operator()(std::size_t row, std::size_t col) const { return data[3 * row + col]; }
    void set(double a00, double a01, double a02,
             double a10, double a11, double a12,
             double a20, double a21, double a22) {
        data = {{a00, a01, a02, a10, a11, a12, a20, a21, a22}};
    }
    bool invert() {
        const double determinant = (
              (*this)(0,0) * (*this)(1,1) * (*this)(2,2)
            + (*this)(0,1) * (*this)(1,2) * (*this)(2,0)
            + (*this)(0,2) * (*this)(1,0) * (*this)(2,1)
            - (*this)(2,0) * (*this)(1,1) * (*this)(0,2)
            - (*this)(2,1) * (*this)(1,2) * (*this)(0,0)
            - (*this)(2,2) * (*this)(1,0) * (*this)(0,1));
        constexpr double legacyTiny = 1e-49;
        if (determinant < legacyTiny && determinant > -legacyTiny) {
            // The original callers intentionally ignore false. Leave J as-is.
            return false;
        }
        const double m00 =  ((*this)(1,1) * (*this)(2,2) - (*this)(2,1)*(*this)(1,2)) / determinant;
        const double m01 = -((*this)(0,1) * (*this)(2,2) - (*this)(2,1)*(*this)(0,2)) / determinant;
        const double m02 =  ((*this)(0,1) * (*this)(1,2) - (*this)(1,1)*(*this)(0,2)) / determinant;
        const double m10 = -((*this)(1,0) * (*this)(2,2) - (*this)(2,0)*(*this)(1,2)) / determinant;
        const double m11 =  ((*this)(0,0) * (*this)(2,2) - (*this)(2,0)*(*this)(0,2)) / determinant;
        const double m12 = -((*this)(0,0) * (*this)(1,2) - (*this)(1,0)*(*this)(0,2)) / determinant;
        const double m20 =  ((*this)(1,0) * (*this)(2,1) - (*this)(2,0)*(*this)(1,1)) / determinant;
        const double m21 = -((*this)(0,0) * (*this)(2,1) - (*this)(2,0)*(*this)(0,1)) / determinant;
        const double m22 =  ((*this)(0,0) * (*this)(1,1) - (*this)(1,0)*(*this)(0,1)) / determinant;
        set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
        return true;
    }
};
inline Mat3 operator*(const Mat3& a, const Mat3& b) {
    Mat3 result;
    for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t col = 0; col < 3; ++col)
            result(row, col) = a(row, 0) * b(0, col)
                + a(row, 1) * b(1, col) + a(row, 2) * b(2, col);
    return result;
}
inline Vec3 operator*(const Mat3& a, const Vec3& b) {
    return Vec3(a(0,0) * b(0) + a(0,1) * b(1) + a(0,2) * b(2),
                a(1,0) * b(0) + a(1,1) * b(1) + a(1,2) * b(2),
                a(2,0) * b(0) + a(2,1) * b(1) + a(2,2) * b(2));
}

} // namespace soft_actuator::detail