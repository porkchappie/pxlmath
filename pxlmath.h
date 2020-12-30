#ifndef PXLMATH_H
#define PXLMATH_H

#define PXLMATH_USE_RADIAN 0
#if PXLMATH_USE_RADIAN
#define PXLMATH_RAD2DEG
#else
#define PXLMATH_RAD2DEG * mathf::rad2deg
#define PXLMATH_DEG2RAD * mathf::deg2rad
#endif

#include <cmath>

namespace pxl {
	struct mat4f;
	union quatf;
	struct vec4f;
	struct vec3f;
	namespace mathf {
		static constexpr float epsilon{ 1e-6f }; // TODO ??
		static constexpr float infinity{ INFINITY };
		static constexpr float pi{ 3.14159265359f };
		static constexpr float halfpi{ 1.57079632679f };
		static constexpr float deg2rad{ 0.01745329252f };
		static constexpr float rad2deg{ 57.295779513f };
		inline static constexpr float rsqrt(const float& x) {
			int i = *(int*)&x;
			i = 0x5f3759df - (i >> 1);
			const float y = *(float*)&i;
			return y * (1.5f - 0.5f * x * y * y);
		}
		inline static constexpr float min(const float& a, const float& b) { return a < b ? a : b; }
		inline static constexpr float max(const float& a, const float& b) { return a > b ? a : b; }
		inline static constexpr float clamp(const float& a, float x, float y) { return x > y ? (a > x ? x : a < y ? y : a) : (a < x ? x : a > y ? y : a); }
		inline static constexpr float clamp01(const float& a) { return a < 0 ? 0 : a > 1 ? 1 : a; }
		inline static constexpr float unclamped_lerp(const float& x, const float& y, const float& a) { return x * (1 - a) + y * a; }
		inline static constexpr float lerp(const float& x, const float& y, const float& a) { return unclamped_lerp(x, y, clamp01(a)); }
	}
	struct vec2f {
		float x, y;
		inline constexpr float dot() const { return x * x + y * y; }
		inline constexpr float magnitude() const { const float& d = dot(); return d < mathf::epsilon ? 0 : sqrtf(d); }
		inline constexpr vec2f& normalize() {
			const float& d = dot();
			if (d < mathf::epsilon) {
				x = y = 0;
			}
			else {
				const float& rmag = mathf::rsqrt(d);
				x *= rmag; y *= rmag;
			}
			return *this;
		}
		inline constexpr vec2f normalized() const {
			const float& d = dot();
			if (d < mathf::epsilon) return { 0 };
			const float& rmag = mathf::rsqrt(d);
			return { x * rmag, y * rmag };
		}
		inline constexpr vec2f operator+() const { return *this; }
		inline constexpr vec2f operator-() const { return (*this) * -1; }
		inline vec2f& operator++() { ++x; ++y; return *this; }
		inline vec2f& operator--() { --x; --y; return *this; }
		inline constexpr vec2f operator++(int) { return { x++, y++ }; }
		inline constexpr vec2f operator--(int) { return { x--, y-- }; }
		inline constexpr vec2f operator+(const float& o) const { return { x + o, y + o }; }
		inline constexpr vec2f operator-(const float& o) const { return { x - o, y - o }; }
		inline constexpr vec2f operator*(const float& o) const { return { x * o, y * o }; }
		inline constexpr vec2f operator/(const float& o) const { return { x / o, y / o }; }
		inline friend constexpr vec2f operator+(const float& l, const vec2f& r) { return { l + r.x, l + r.y }; }
		inline friend constexpr vec2f operator-(const float& l, const vec2f& r) { return { l - r.x, l - r.y }; }
		inline friend constexpr vec2f operator*(const float& l, const vec2f& r) { return { l * r.x, l * r.y }; }
		inline friend constexpr vec2f operator/(const float& l, const vec2f& r) { return { l / r.x, l / r.y }; }
		inline constexpr vec2f operator+(const vec2f& o) const { return { x + o.x, y + o.y }; }
		inline constexpr vec2f operator-(const vec2f& o) const { return { x - o.x, y - o.y }; }
		inline vec2f& operator=(const float& o) { x = y = o; return *this; }
		inline vec2f& operator+=(const float& o) { x += o; y += o; return *this; }
		inline vec2f& operator-=(const float& o) { x -= o; y -= o; return *this; }
		inline vec2f& operator*=(const float& o) { x *= o; y *= o; return *this; }
		inline vec2f& operator/=(const float& o) { x /= o; y /= o; return *this; }
		inline vec2f& operator+=(const vec2f& o) { x += o.x; y += o.y; return *this; }
		inline vec2f& operator-=(const vec2f& o) { x -= o.x; y -= o.y; return *this; }
		inline constexpr bool operator==(const vec2f& o) const { return x == o.x && y == o.y; }
		inline constexpr bool operator!=(const vec2f& o) const { return x != o.x || y != o.y; }
		inline float& operator[](const size_t& i) const { return ((float*)this)[i]; }
		inline constexpr operator vec3f() const;
		inline constexpr operator vec4f() const;
		// static constants
		inline static constexpr vec2f zero() { return { 0, 0 }; }
		inline static constexpr vec2f one() { return { 1, 1 }; }
		inline static constexpr vec2f left() { return { -1, 0 }; }
		inline static constexpr vec2f right() { return { 1, 0 }; }
		inline static constexpr vec2f up() { return { 0, 1 }; }
		inline static constexpr vec2f down() { return { 0, -1 }; }
		inline static constexpr vec2f infinity() { return { mathf::infinity, mathf::infinity }; }
		// static functions
		inline static float angle(const vec2f& a, const vec2f& b) { const float& d = a.dot() * b.dot(); return d < mathf::epsilon ? mathf::halfpi : acosf(dot(a, b) / sqrtf(d)) PXLMATH_RAD2DEG; }
		inline static float signed_angle(const vec2f& a, const vec2f& b) { return cross(a, b) < 0 ? -angle(a, b) : angle(a, b); }
		inline static constexpr vec2f clamped_magnitude(const vec2f& a, const float& b) { const float& d = a.dot(); return b * b < d ? d < mathf::epsilon ? vec2f::zero() : a * b * mathf::rsqrt(d) : a; }
		inline static constexpr vec2f min(const vec2f& a, const vec2f& b) { return { mathf::min(a.x, b.x), mathf::min(a.y, b.y) }; }
		inline static constexpr vec2f max(const vec2f& a, const vec2f& b) { return { mathf::max(a.x, b.x), mathf::max(a.y, b.y) }; }
		inline static constexpr float dot(const vec2f& a, const vec2f& b) { return a.x * b.x + a.y * b.y; }
		inline static constexpr float cross(const vec2f& a, const vec2f& b) { return a.x * b.y - a.y * b.x; }
		inline static constexpr float distance(const vec2f& a, const vec2f& b) { return (a - b).magnitude(); }
		inline static constexpr vec2f reflect(const vec2f& a, const vec2f& b) { return a - b * dot(a, b) * 2; }
		inline static constexpr vec2f unclamped_lerp(const vec2f& x, const vec2f& y, const float& a) { return x * (1 - a) + y * a; }
		inline static constexpr vec2f lerp(const vec2f& x, const vec2f& y, const float& a) { return unclamped_lerp(x, y, mathf::clamp01(a)); }
		inline static constexpr vec2f move_towards(const vec2f& x, const vec2f& y, const float& a) {
			const vec2f z = y - x;
			const float& d = z.dot();
			if (a && d > a * a) return x + z * mathf::rsqrt(d) * a;
			return y;
		}
	};
	struct vec3f {
		float x, y, z;
		inline constexpr float dot() const { return x * x + y * y + z * z; }
		inline constexpr float magnitude() const { const float& d = dot(); return d < mathf::epsilon ? 0 : sqrtf(d); }
		inline constexpr vec3f& normalize() {
			const float& d = dot();
			if (d < mathf::epsilon) {
				x = y = z = 0;
			}
			else {
				const float& rmag = mathf::rsqrt(d);
				x *= rmag; y *= rmag; z *= rmag;
			}
			return *this;
		}
		inline constexpr vec3f normalized() const {
			const float& d = dot();
			if (d < mathf::epsilon) return { 0 };
			const float& rmag = mathf::rsqrt(d);
			return { x * rmag, y * rmag, z * rmag };
		}
		inline constexpr vec3f operator+() const { return *this; }
		inline constexpr vec3f operator-() const { return (*this) * -1; }
		inline vec3f& operator++() { ++x; ++y; ++z; return *this; }
		inline vec3f& operator--() { --x; --y; --z; return *this; }
		inline constexpr vec3f operator++(int) { return { x++, y++, z++ }; }
		inline constexpr vec3f operator--(int) { return { x--, y--, z-- }; }
		inline constexpr vec3f operator+(const float& o) const { return { x + o, y + o, z + o }; }
		inline constexpr vec3f operator-(const float& o) const { return { x - o, y - o, z - o }; }
		inline constexpr vec3f operator*(const float& o) const { return { x * o, y * o, z * o }; }
		inline constexpr vec3f operator/(const float& o) const { return { x / o, y / o, z / o }; }
		inline friend constexpr vec3f operator+(const float& l, const vec3f& r) { return { l + r.x, l + r.y, l + r.z }; }
		inline friend constexpr vec3f operator-(const float& l, const vec3f& r) { return { l - r.x, l - r.y, l - r.z }; }
		inline friend constexpr vec3f operator*(const float& l, const vec3f& r) { return { l * r.x, l * r.y, l * r.z }; }
		inline friend constexpr vec3f operator/(const float& l, const vec3f& r) { return { l / r.x, l / r.y, l / r.z }; }
		inline constexpr vec3f operator+(const vec3f& o) const { return { x + o.x, y + o.y, z + o.z }; }
		inline constexpr vec3f operator-(const vec3f& o) const { return { x - o.x, y - o.y, z - o.z }; }
		inline vec3f& operator=(const float& o) { x = y = z = o; return *this; }
		inline vec3f& operator+=(const float& o) { x += o; y += o; z += o; return *this; }
		inline vec3f& operator-=(const float& o) { x -= o; y -= o; z -= o; return *this; }
		inline vec3f& operator*=(const float& o) { x *= o; y *= o; z *= o; return *this; }
		inline vec3f& operator/=(const float& o) { x /= o; y /= o; z /= o; return *this; }
		inline vec3f& operator+=(const vec3f& o) { x += o.x; y += o.y; z += o.z; return *this; }
		inline vec3f& operator-=(const vec3f& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
		inline constexpr bool operator==(const vec3f& o) const { return x == o.x && y == o.y && z == o.z; }
		inline constexpr bool operator!=(const vec3f& o) const { return x != o.x || y != o.y || z != o.z; }
		inline float& operator[](const size_t& i) const { return ((float*)this)[i]; }
		inline constexpr operator vec2f() const;
		inline constexpr operator vec4f() const;
		// static constants
		inline static constexpr vec3f zero() { return { 0, 0, 0 }; }
		inline static constexpr vec3f one() { return { 1, 1, 1 }; }
		inline static constexpr vec3f left() { return { -1, 0, 0 }; }
		inline static constexpr vec3f right() { return { 1, 0, 0 }; }
		inline static constexpr vec3f up() { return { 0, 1, 0 }; }
		inline static constexpr vec3f down() { return { 0, -1, 0 }; }
		inline static constexpr vec3f forward() { return { 0, 0, 1 }; }
		inline static constexpr vec3f back() { return { 0, 0, -1 }; }
		inline static constexpr vec3f infinity() { return { mathf::infinity, mathf::infinity, mathf::infinity }; }
		// static functions
		inline static float angle(const vec3f& a, const vec3f& b) { const float& d = a.dot() * b.dot(); return d < mathf::epsilon ? mathf::halfpi : acosf(dot(a, b) / sqrtf(d)) PXLMATH_RAD2DEG; }
		inline static float signed_angle(const vec3f& a, const vec3f& b, const vec3f& c) { return dot(cross(a, b), c) < 0 ? -angle(a, b) : angle(a, b); }
		inline static constexpr vec3f clamped_magnitude(const vec3f& a, const float& b) { const float& d = a.dot(); return b * b < d ? d < mathf::epsilon ? vec3f::zero() : a * b * mathf::rsqrt(d) : a; }
		inline static constexpr vec3f min(const vec3f& a, const vec3f& b) { return { mathf::min(a.x, b.x), mathf::min(a.y, b.y), mathf::min(a.z, b.z) }; }
		inline static constexpr vec3f max(const vec3f& a, const vec3f& b) { return { mathf::max(a.x, b.x), mathf::max(a.y, b.y), mathf::max(a.z, b.z) }; }
		inline static constexpr float dot(const vec3f& a, const vec3f& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
		inline static constexpr float distance(const vec3f& a, const vec3f& b) { return (a - b).magnitude(); }
		inline static constexpr vec3f cross(const vec3f& a, const vec3f& b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
		inline static constexpr vec3f reflect(const vec3f& a, const vec3f& b) { return a - b * dot(a, b) * 2; }
		inline static constexpr vec3f project(const vec3f& a, const vec3f& b) { const float& c = b.dot(); return c < mathf::epsilon ? vec3f::zero() : dot(a, b) * b / c; }
		inline static constexpr vec3f project_on_plane(const vec3f& a, const vec3f& b) { return a - project(a, b); }
		inline static constexpr vec3f unclamped_lerp(const vec3f& x, const vec3f& y, const float& a) { return x * (1 - a) + y * a; }
		inline static constexpr vec3f lerp(const vec3f& x, const vec3f& y, const float& a) { return unclamped_lerp(x, y, mathf::clamp01(a)); }
		inline static constexpr vec3f move_towards(const vec3f& x, const vec3f& y, const float& a) { const vec3f z = y - x; const float& d = z.dot(); return a > mathf::epsilon && d > a * a ? x + z * mathf::rsqrt(d) * a : y; }
	};
	struct vec4f {
		float x, y, z, w;
		inline constexpr float dot() const { return x * x + y * y + z * z + w * w; }
		inline constexpr float magnitude() const { const float& d = dot(); return d < mathf::epsilon ? 0 : sqrtf(d); }
		inline constexpr vec4f& normalize() {
			const float& d = dot();
			if (d < mathf::epsilon) {
				x = y = z = w = 0;
			}
			else {
				const float& rmag = mathf::rsqrt(d);
				x *= rmag; y *= rmag; z *= rmag; w *= rmag;
			}
			return *this;
		}
		inline constexpr vec4f normalized() const {
			const float& d = dot();
			if (d < mathf::epsilon) return { 0 };
			const float& rmag = mathf::rsqrt(d);
			return { x * rmag, y * rmag, z * rmag, w * rmag };
		}
		inline constexpr vec4f operator+() const { return *this; }
		inline constexpr vec4f operator-() const { return (*this) * -1; }
		inline vec4f& operator++() { ++x; ++y; ++z; ++w; return *this; }
		inline vec4f& operator--() { --x; --y; --z; --w; return *this; }
		inline constexpr vec4f operator++(int) { return { x++, y++, z++, w++ }; }
		inline constexpr vec4f operator--(int) { return { x--, y--, z--, w-- }; }
		inline constexpr vec4f operator+(const float& o) const { return { x + o, y + o, z + o, w + o }; }
		inline constexpr vec4f operator-(const float& o) const { return { x - o, y - o, z - o, w - o }; }
		inline constexpr vec4f operator*(const float& o) const { return { x * o, y * o, z * o, w * o }; }
		inline constexpr vec4f operator/(const float& o) const { return { x / o, y / o, z / o, w / o }; }
		inline friend constexpr vec4f operator+(const float& l, const vec4f& r) { return { l + r.x, l + r.y, l + r.z, l + r.w }; }
		inline friend constexpr vec4f operator-(const float& l, const vec4f& r) { return { l - r.x, l - r.y, l - r.z, l - r.w }; }
		inline friend constexpr vec4f operator*(const float& l, const vec4f& r) { return { l * r.x, l * r.y, l * r.z, l * r.w }; }
		inline friend constexpr vec4f operator/(const float& l, const vec4f& r) { return { l / r.x, l / r.y, l / r.z, l / r.w }; }
		inline constexpr vec4f operator+(const vec4f& o) const { return { x + o.x, y + o.y, z + o.z, w + o.w }; }
		inline constexpr vec4f operator-(const vec4f& o) const { return { x - o.x, y - o.y, z - o.z, w - o.w }; }
		inline vec4f& operator=(const float& o) { x = y = z = w = o; return *this; }
		inline vec4f& operator+=(const float& o) { x += o; y += o; z += o; w += o; return *this; }
		inline vec4f& operator-=(const float& o) { x -= o; y -= o; z -= o; w -= o; return *this; }
		inline vec4f& operator*=(const float& o) { x *= o; y *= o; z *= o; w *= o; return *this; }
		inline vec4f& operator/=(const float& o) { x /= o; y /= o; z /= o; w /= o; return *this; }
		inline vec4f& operator+=(const vec4f& o) { x += o.x; y += o.y; z += o.z; w += o.w; return *this; }
		inline vec4f& operator-=(const vec4f& o) { x -= o.x; y -= o.y; z -= o.z; w -= o.w; return *this; }
		inline constexpr bool operator==(const vec4f& o) const { return x == o.x && y == o.y && z == o.z && w == o.w; }
		inline constexpr bool operator!=(const vec4f& o) const { return x != o.x || y != o.y || z != o.z || w != o.w; }
		inline float& operator[](const size_t& i) const { return ((float*)this)[i]; }
		inline constexpr operator vec2f() const;
		inline constexpr operator vec3f() const;
		// static constants
		inline static constexpr vec4f zero() { return { 0, 0, 0, 0 }; }
		inline static constexpr vec4f one() { return { 1, 1, 1, 1 }; }
		inline static constexpr vec4f infinity() { return { mathf::infinity, mathf::infinity, mathf::infinity, mathf::infinity }; }
		// static functions
		inline static constexpr vec4f min(const vec4f& a, const vec4f& b) { return { mathf::min(a.x, b.x), mathf::min(a.y, b.y), mathf::min(a.z, b.z), mathf::min(a.w, b.w) }; }
		inline static constexpr vec4f max(const vec4f& a, const vec4f& b) { return { mathf::max(a.x, b.x), mathf::max(a.y, b.y), mathf::max(a.z, b.z), mathf::max(a.w, b.w) }; }
		inline static constexpr float dot(const vec4f& a, const vec4f& b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w + b.w; }
		inline static constexpr float distance(const vec4f& a, const vec4f& b) { return (a - b).magnitude(); }
		inline static constexpr vec4f cross(const vec4f& a, const vec4f& b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x, 1 }; }
		inline static constexpr vec4f reflect(const vec4f& a, const vec4f& b) { return a - b * dot(a, b) * 2; }
		inline static constexpr vec4f project(const vec4f& a, const vec4f& b) { const float& c = b.dot(); return c < mathf::epsilon ? vec4f::zero() : dot(a, b) * b / c; }
		inline static constexpr vec4f unclamped_lerp(const vec4f& x, const vec4f& y, const float& a) { return x * (1 - a) + y * a; }
		inline static constexpr vec4f lerp(const vec4f& x, const vec4f& y, const float& a) { return unclamped_lerp(x, y, mathf::clamp01(a)); }
		inline static constexpr vec4f move_towards(const vec4f& x, const vec4f& y, const float& a) { const vec4f z = y - x; const float& d = z.dot(); return a > mathf::epsilon && d > a * a ? x + z * mathf::rsqrt(d) * a : y; }
	};
	union quatf {
		struct {
			float x, y, z, w;
		};
		vec3f xyz;
		inline constexpr float dot() const { return x * x + y * y + z * z + w * w; }
		inline constexpr quatf& normalize() {
			const float& d = dot();
			if (d < mathf::epsilon) { // TODO or d < mathf::epsilon * mathf::epsilon
				x = y = z = 0; w = 1;
			}
			else {
				const float& rmag = mathf::rsqrt(d);
				x *= rmag; y *= rmag; z *= rmag; w *= rmag;
			}
			return *this;
		}
		inline constexpr quatf normalized() const {
			const float& d = dot();
			if (d < mathf::epsilon) return { 0, 0, 0, 1 };
			const float& rmag = mathf::rsqrt(d);
			return { x * rmag, y * rmag, z * rmag, w * rmag };
		}
		inline quatf& conjugate() { xyz *= -1; return *this; }
		inline constexpr quatf conjugated() const { return { -x, -y, -z, w }; }
		inline quatf& euler(const vec3f& e) { return *this = from_euler(e); }
		inline vec3f euler() const { return to_euler(*this); }

		inline constexpr quatf operator+() const { return *this; }
		inline constexpr quatf operator-() const { return (*this) * -1; }
		inline quatf& operator++() { ++x; ++y; ++z; ++w; return *this; }
		inline quatf& operator--() { --x; --y; --z; --w; return *this; }
		inline constexpr quatf operator++(int) { return { x++, y++, z++, w++ }; }
		inline constexpr quatf operator--(int) { return { x--, y--, z--, w-- }; }
		inline constexpr quatf operator+(const float& o) const { return { x + o, y + o, z + o, w + o }; }
		inline constexpr quatf operator-(const float& o) const { return { x - o, y - o, z - o, w - o }; }
		inline constexpr quatf operator*(const float& o) const { return { x * o, y * o, z * o, w * o }; }
		inline constexpr quatf operator/(const float& o) const { return { x / o, y / o, z / o, w / o }; }
		inline friend constexpr quatf operator+(const float& l, const quatf& r) { return { l + r.x, l + r.y, l + r.z, l + r.w }; }
		inline friend constexpr quatf operator-(const float& l, const quatf& r) { return { l - r.x, l - r.y, l - r.z, l - r.w }; }
		inline friend constexpr quatf operator*(const float& l, const quatf& r) { return { l * r.x, l * r.y, l * r.z, l * r.w }; }
		inline friend constexpr quatf operator/(const float& l, const quatf& r) { return { l / r.x, l / r.y, l / r.z, l / r.w }; }
		inline constexpr quatf operator+(const quatf& o) const { return { x + o.x, y + o.y, z + o.z, w + o.w }; }
		inline constexpr quatf operator-(const quatf& o) const { return { x - o.x, y - o.y, z - o.z, w - o.w }; }
		inline constexpr quatf operator*(const quatf& o) const {
			return {
				w * o.x + x * o.w + y * o.z - z * o.y,
				w * o.y + y * o.w + z * o.x - x * o.z,
				w * o.z + z * o.w + x * o.y - y * o.x,
				w * o.w - x * o.x - y * o.y - z * o.z };
		}
		inline constexpr vec3f operator*(const vec3f& o) { const vec3f a = 2 * vec3f::cross(xyz, o); return o + w * a + vec3f::cross(xyz, a); } // TODO optimize
		inline quatf& operator=(const float& o) { x = y = z = w = o; return *this; }
		inline quatf& operator+=(const float& o) { x += o; y += o; z += o; w += o; return *this; }
		inline quatf& operator-=(const float& o) { x -= o; y -= o; z -= o; w -= o; return *this; }
		inline quatf& operator*=(const float& o) { x *= o; y *= o; z *= o; w *= o; return *this; }
		inline quatf& operator/=(const float& o) { x /= o; y /= o; z /= o; w /= o; return *this; }
		inline quatf& operator+=(const quatf& o) { x += o.x; y += o.y; z += o.z; w += o.w; return *this; }
		inline quatf& operator-=(const quatf& o) { x -= o.x; y -= o.y; z -= o.z; w -= o.w; return *this; }
		inline quatf& operator*=(const quatf& o) {
			const float x = this->x, y = this->y, z = this->z, w = this->w;
			this->x = w * o.x + x * o.w + y * o.z - z * o.y;
			this->y = w * o.y + y * o.w + z * o.x - x * o.z;
			this->z = w * o.z + z * o.w + x * o.y - y * o.x;
			this->w = w * o.w - x * o.x - y * o.y - z * o.z;
			return *this;
		}
		inline constexpr bool operator==(const quatf& o) const { return x == o.x && y == o.y && z == o.z && w == o.w; }
		inline constexpr bool operator!=(const quatf& o) const { return x != o.x || y != o.y || z != o.z || w != o.w; }
		inline float& operator[](const size_t& i) const { return ((float*)this)[i]; }
		// static constants
		inline static constexpr quatf identity() { return { 0, 0, 0, 1 }; }
		// static functions
		inline static constexpr quatf rotate(vec3f x, vec3f y, const vec3f& a = vec3f::up()) {
			const float& d = vec3f::dot(x.normalize(), y.normalize());
			if (d >= 0.999999f) return { 0, 0, 0, 1 };
			if (d <= -0.999999f) return { a.x, a.y, a.z, 0 };
			const vec3f& c = vec3f::cross(x, y);
			return quatf{ c.x, c.y, c.z, 1 }.normalize();
		}
		inline static vec3f to_euler(const quatf& q) {
			const float x2 = q.x * q.x, y2 = q.y * q.y, z2 = q.z * q.z, w2 = q.w * q.w,
				u = x2 + y2 + z2 + w2, t = q.x * q.y + q.z * q.w;
			if (t > 0.499f * u)
				return { 0, mathf::halfpi, 2 * atan2f(q.x, q.w) };
			if (t < -0.499f * u)
				return { 0, -mathf::halfpi, -2 * atan2f(q.x, q.w) };
			return { atan2f(2 * q.x * q.w - 2 * q.y * q.z, -x2 + y2 - z2 + w2) PXLMATH_RAD2DEG,
					atan2f(2 * q.y * q.w - 2 * q.x * q.z, x2 - y2 - z2 + w2) PXLMATH_RAD2DEG,
					asinf(2 * t / u) PXLMATH_RAD2DEG };
		}
		inline static quatf from_euler(const vec3f& r) {
			const vec3f hr{ r * 0.5f PXLMATH_DEG2RAD };
			const float x0 = cosf(hr.x), x1 = sinf(hr.x), y0 = cosf(hr.y),
				y1 = sinf(hr.y), z0 = cosf(hr.z), z1 = sinf(hr.z),
				y0z0 = y0 * z0, y0z1 = y0 * z1, y1z0 = y1 * z0, y1z1 = y1 * z1;
			return { y0z0 * x1 + y1z1 * x0, y1z0 * x0 + y0z1 * x1, y0z1 * x0 - y1z0 * x1,
					y0z0 * x0 - y1z1 * x1 };
		}
		inline static float to_angle_axis(const quatf& a, vec3f& b) {
			const float d = a.xyz.dot();
			b = d == 0 ? vec3f::up() : a.xyz.normalized();
			return 2 * atan2f(sqrtf(d), a.w) PXLMATH_RAD2DEG;
		}
		inline static quatf from_angle_axis(const float& a, const vec3f& b) { const float ha = a * 0.5f PXLMATH_DEG2RAD, c = sinf(ha); return { b.x * c, b.y * c, b.z * c, cosf(ha) }; }
		inline static constexpr float dot(const quatf& a, const quatf& b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w + b.w; }
		inline static constexpr quatf unclamped_lerp(const quatf& x, const quatf& y, const float& a) { return x * (1 - a) + y * a; }
		inline static constexpr quatf lerp(const quatf& x, const quatf& y, const float& a) { return unclamped_lerp(x, y, mathf::clamp01(a)); }
		inline static constexpr quatf unclamped_slerp(const quatf& x, const quatf& y, const float& a) { return x.w + y.w + vec3f::dot(x.xyz, y.xyz) > 0.999999f ? lerp(x, y, a) : x * (x.conjugated() * y) * a; }
		inline static constexpr quatf slerp(const quatf& x, const quatf& y, const float& a) { return unclamped_slerp(x, y, mathf::clamp01(a)); }
	};
	struct mat4f {
		float
			m00, m01, m02, m03,
			m10, m11, m12, m13,
			m20, m21, m22, m23,
			m30, m31, m32, m33;
		inline constexpr float det() const {
			const float m2323 = m22 * m33 - m23 * m32;
			const float m1323 = m21 * m33 - m23 * m31;
			const float m1223 = m21 * m32 - m22 * m31;
			const float m0323 = m20 * m33 - m23 * m30;
			const float m0223 = m20 * m32 - m22 * m30;
			const float m0123 = m20 * m31 - m21 * m30;
			const float m2313 = m12 * m33 - m13 * m32;
			const float m1313 = m11 * m33 - m13 * m31;
			const float m1213 = m11 * m32 - m12 * m31;
			const float m2312 = m12 * m23 - m13 * m22;
			const float m1312 = m11 * m23 - m13 * m21;
			const float m1212 = m11 * m22 - m12 * m21;
			const float m0313 = m10 * m33 - m13 * m30;
			const float m0213 = m10 * m32 - m12 * m30;
			const float m0312 = m10 * m23 - m13 * m20;
			const float m0212 = m10 * m22 - m12 * m20;
			const float m0113 = m10 * m31 - m11 * m30;
			const float m0112 = m10 * m21 - m11 * m20;
			float det = m00 * (m11 * m2323 - m12 * m1323 + m13 * m1223) -
				m01 * (m10 * m2323 - m12 * m0323 + m13 * m0223) +
				m02 * (m10 * m1323 - m11 * m0323 + m13 * m0123) -
				m03 * (m10 * m1223 - m11 * m0223 + m12 * m0123);
			return det;
		}
		inline float inverse(mat4f& out) const {
			const float m2323 = m22 * m33 - m23 * m32;
			const float m1323 = m21 * m33 - m23 * m31;
			const float m1223 = m21 * m32 - m22 * m31;
			const float m0323 = m20 * m33 - m23 * m30;
			const float m0223 = m20 * m32 - m22 * m30;
			const float m0123 = m20 * m31 - m21 * m30;
			const float m2313 = m12 * m33 - m13 * m32;
			const float m1313 = m11 * m33 - m13 * m31;
			const float m1213 = m11 * m32 - m12 * m31;
			const float m2312 = m12 * m23 - m13 * m22;
			const float m1312 = m11 * m23 - m13 * m21;
			const float m1212 = m11 * m22 - m12 * m21;
			const float m0313 = m10 * m33 - m13 * m30;
			const float m0213 = m10 * m32 - m12 * m30;
			const float m0312 = m10 * m23 - m13 * m20;
			const float m0212 = m10 * m22 - m12 * m20;
			const float m0113 = m10 * m31 - m11 * m30;
			const float m0112 = m10 * m21 - m11 * m20;
			const float det = m00 * (m11 * m2323 - m12 * m1323 + m13 * m1223) -
				m01 * (m10 * m2323 - m12 * m0323 + m13 * m0223) +
				m02 * (m10 * m1323 - m11 * m0323 + m13 * m0123) -
				m03 * (m10 * m1223 - m11 * m0223 + m12 * m0123);
			if (det) {
				const float rdet = 1 / det;
				out.m00 = rdet * (m11 * m2323 - m12 * m1323 + m13 * m1223);
				out.m01 = rdet * -(m01 * m2323 - m02 * m1323 + m03 * m1223);
				out.m02 = rdet * (m01 * m2313 - m02 * m1313 + m03 * m1213);
				out.m03 = rdet * -(m01 * m2312 - m02 * m1312 + m03 * m1212);
				out.m10 = rdet * -(m10 * m2323 - m12 * m0323 + m13 * m0223);
				out.m11 = rdet * (m00 * m2323 - m02 * m0323 + m03 * m0223);
				out.m12 = rdet * -(m00 * m2313 - m02 * m0313 + m03 * m0213);
				out.m13 = rdet * (m00 * m2312 - m02 * m0312 + m03 * m0212);
				out.m20 = rdet * (m10 * m1323 - m11 * m0323 + m13 * m0123);
				out.m21 = rdet * -(m00 * m1323 - m01 * m0323 + m03 * m0123);
				out.m22 = rdet * (m00 * m1313 - m01 * m0313 + m03 * m0113);
				out.m23 = rdet * -(m00 * m1312 - m01 * m0312 + m03 * m0112);
				out.m30 = rdet * -(m10 * m1223 - m11 * m0223 + m12 * m0123);
				out.m31 = rdet * (m00 * m1223 - m01 * m0223 + m02 * m0123);
				out.m32 = rdet * -(m00 * m1213 - m01 * m0213 + m02 * m0113);
				out.m33 = rdet * (m00 * m1212 - m01 * m0212 + m02 * m0112);
			}
			return det;
		}
		inline mat4f& transpose() {
			const mat4f t = *this;
			m01 = t.m10; m02 = t.m20; m03 = t.m30;
			m10 = t.m01; m12 = t.m21; m13 = t.m31;
			m20 = t.m02; m21 = t.m12; m23 = t.m32;
			m30 = t.m03; m31 = t.m13; m32 = t.m23;
			return *this;
		}
		inline constexpr mat4f transposed() const {
			return {
				m00, m10, m20, m30,
				m01, m11, m21, m31,
				m02, m12, m22, m32,
				m03, m13, m23, m33
			};
		}
		inline constexpr mat4f operator*(const float& o) const {
			return {
				m00 * o, m01 * o, m02 * o, m03 * o,
				m10 * o, m11 * o, m12 * o, m13 * o,
				m20 * o, m21 * o, m22 * o, m23 * o,
				m30 * o, m31 * o, m32 * o, m33 * o };
		}
		inline mat4f& trs(const vec3f& t, const quatf& r = quatf::identity(), const vec3f& s = vec3f::one()) {
			const float x = r.x, y = r.y, z = r.z, w = r.w, x2 = x * x, y2 = y * y,
				z2 = z * z, xy = x * y, xz = x * z, yz = y * z, wx = w * x,
				wy = w * y, wz = w * z;
			m00 = (1.0f - 2.0f * (y2 + z2)) * s.x; m01 = (2.0f * (xy - wz)) * s.y; m02 = (2.0f * (xz + wy)) * s.z; m03 = t.x;
			m10 = (2.0f * (xy + wz)) * s.x; m11 = (1.0f - 2.0f * (x2 + z2)) * s.y; m12 = (2.0f * (yz - wx)) * s.z; m13 = t.y;
			m20 = (2.0f * (xz - wy)) * s.x; m21 = (2.0f * (yz + wx)) * s.y; m22 = (1.0f - 2.0f * (x2 + y2)) * s.z; m23 = t.z;
			m30 = 0; m31 = 0; m32 = 0; m33 = 1;
			return *this;
		}
		inline constexpr vec2f operator*(const vec2f& o) const {
			return {
				m00 * o.x + m01 * o.y + m02 + m03,
				m10 * o.x + m11 * o.y + m12 + m13 };
		}
		inline constexpr vec3f operator*(const vec3f& o) const {
			return {
				m00 * o.x + m01 * o.y + m02 * o.z + m03,
				m10 * o.x + m11 * o.y + m12 * o.z + m13,
				m20 * o.x + m21 * o.y + m22 * o.z + m23 };
		}
		inline constexpr vec4f operator*(const vec4f& o) const {
			return {
				m00 * o.x + m01 * o.y + m02 * o.z + m03 * o.w,
				m10 * o.x + m11 * o.y + m12 * o.z + m13 * o.w,
				m20 * o.x + m21 * o.y + m22 * o.z + m23 * o.w,
				m30 * o.x + m31 * o.y + m32 * o.z + m33 * o.w };
		}
		inline constexpr mat4f operator*(const mat4f& o) const {
			return {
				m00 * o.m00 + m01 * o.m10 + m02 * o.m20 + m03 * o.m30,
				m00 * o.m01 + m01 * o.m11 + m02 * o.m21 + m03 * o.m31,
				m00 * o.m02 + m01 * o.m12 + m02 * o.m22 + m03 * o.m32,
				m00 * o.m03 + m01 * o.m13 + m02 * o.m23 + m03 * o.m33,

				m10 * o.m00 + m11 * o.m10 + m12 * o.m20 + m13 * o.m30,
				m10 * o.m01 + m11 * o.m11 + m12 * o.m21 + m13 * o.m31,
				m10 * o.m02 + m11 * o.m12 + m12 * o.m22 + m13 * o.m32,
				m10 * o.m03 + m11 * o.m13 + m12 * o.m23 + m13 * o.m33,

				m20 * o.m00 + m21 * o.m10 + m22 * o.m20 + m23 * o.m30,
				m20 * o.m01 + m21 * o.m11 + m22 * o.m21 + m23 * o.m31,
				m20 * o.m02 + m21 * o.m12 + m22 * o.m22 + m23 * o.m32,
				m20 * o.m03 + m21 * o.m13 + m22 * o.m23 + m23 * o.m33,

				m30 * o.m00 + m31 * o.m10 + m32 * o.m20 + m33 * o.m30,
				m30 * o.m01 + m31 * o.m11 + m32 * o.m21 + m33 * o.m31,
				m30 * o.m02 + m31 * o.m12 + m32 * o.m22 + m33 * o.m32,
				m30 * o.m03 + m31 * o.m13 + m32 * o.m23 + m33 * o.m33 };
		}
		inline mat4f& operator*=(const float& o) {
			m00 *= o; m01 *= o; m02 *= o; m03 *= o;
			m10 *= o; m11 *= o; m12 *= o; m13 *= o;
			m20 *= o; m21 *= o; m22 *= o; m23 *= o;
			m30 *= o; m31 *= o; m32 *= o; m33 *= o;
			return *this;
		}
		inline mat4f& operator*=(const mat4f& o) { return *this = *this * o; }
		inline float& operator[](const size_t& i) const { return ((float*)this)[i]; }
		// static constants
		inline static constexpr mat4f zero() { return { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; }
		inline static constexpr mat4f identity() { return { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 }; }
		// static functions
		inline static constexpr vec3f to_translation(const mat4f& m) { return { m.m03, m.m13, m.m23 }; }
		inline static constexpr quatf to_rotation(const mat4f& m) {
			float e = m.m00 + m.m11 + m.m22;
			if (e > 0) {
				const float a = mathf::rsqrt(e + 1.0f) * 0.5f; // S=4*qw
				return { (m.m21 - m.m12) * a, (m.m02 - m.m20) * a, (m.m10 - m.m01) * a,
						0.25f / a };
			}
			else if ((m.m00 > m.m11) && (m.m00 > m.m22)) {
				float a = mathf::rsqrt(1.0f + m.m00 - m.m11 - m.m22) * 0.5f; // S=4*qx
				return { 0.25f / a, (m.m01 + m.m10) * a, (m.m02 + m.m20) * a,
						(m.m21 - m.m12) * a };

			}
			else if (m.m11 > m.m22) {
				float a = mathf::rsqrt(1.0f + m.m11 - m.m00 - m.m22) * 0.5f; // S=4*qy
				return { (m.m01 + m.m10) * a, 0.25f / a, (m.m12 + m.m21) * a,
						(m.m02 - m.m20) * a };
			}
			else {
				float a = mathf::rsqrt(1.0f + m.m22 - m.m00 - m.m11) * 0.5f; // S=4*qz
				return { (m.m02 + m.m20) * a, (m.m12 + m.m21) * a, 0.25f / a,
						(m.m10 - m.m01) * a };
			}
		}
		inline static constexpr vec3f to_scale(const mat4f& m) {
			return {
				sqrtf(m.m00 * m.m00 + m.m10 * m.m10 + m.m20 * m.m20),
				sqrtf(m.m01 * m.m01 + m.m11 * m.m11 + m.m21 * m.m21),
				sqrtf(m.m02 * m.m02 + m.m12 * m.m12 + m.m22 * m.m22) };
		}
		inline static constexpr mat4f from_translation(const vec3f& t) {
			return {
				1, 0, 0, t.x,
				0, 1, 0, t.y,
				0, 0, 1, t.z,
				0, 0, 0, 1 };
		}
		inline static constexpr mat4f from_rotation(const quatf& r) {
			const float x = r.x, y = r.y, z = r.z, w = r.w, x2 = x * x, y2 = y * y,
				z2 = z * z, xy = x * y, xz = x * z, yz = y * z, wx = w * x,
				wy = w * y, wz = w * z;
			return {
				(1.0f - 2.0f * (y2 + z2)), (2.0f * (xy - wz)), (2.0f * (xz + wy)), 0,
				(2.0f * (xy + wz)), (1.0f - 2.0f * (x2 + z2)), (2.0f * (yz - wx)), 0,
				(2.0f * (xz - wy)), (2.0f * (yz + wx)), (1.0f - 2.0f * (x2 + y2)), 0,
				0, 0, 0, 1 };
		}
		inline static constexpr mat4f from_scale(const vec3f& s) {
			return {
				s.x, 0, 0, 0,
				0, s.y, 0, 0,
				0, 0, s.z, 0,
				0, 0, 0, 1,
			};
		}
		inline static constexpr mat4f from_trs(const vec3f& t, const quatf& r = quatf::identity(), const vec3f& s = vec3f::one()) {
			const float x = r.x, y = r.y, z = r.z, w = r.w, x2 = x * x, y2 = y * y,
				z2 = z * z, xy = x * y, xz = x * z, yz = y * z, wx = w * x,
				wy = w * y, wz = w * z;
			return {
				(1.0f - 2.0f * (y2 + z2)) * s.x, (2.0f * (xy - wz)) * s.y, (2.0f * (xz + wy)) * s.z, t.x,
				(2.0f * (xy + wz)) * s.x, (1.0f - 2.0f * (x2 + z2)) * s.y, (2.0f * (yz - wx)) * s.z, t.y,
				(2.0f * (xz - wy)) * s.x, (2.0f * (yz + wx)) * s.y, (1.0f - 2.0f * (x2 + y2)) * s.z, t.z,
				0, 0, 0, 1 };
		}
		inline static constexpr mat4f view(const vec3f& t, const quatf& r = quatf::identity(), const vec3f& s = vec3f::one()) {
			const float x = r.x, y = r.y, z = r.z, w = r.w, x2 = x * x, y2 = y * y,
				z2 = z * z, xy = x * y, xz = x * z, yz = y * z, wx = w * x,
				wy = w * y, wz = w * z;
			return {
				(1.0f - 2.0f * (y2 + z2)) * s.x, (2.0f * (xy - wz)) * s.y, (2.0f * (xz + wy)) * s.z, -t.x,
				(2.0f * (xy + wz)) * s.x, (1.0f - 2.0f * (x2 + z2)) * s.y, (2.0f * (yz - wx)) * s.z, -t.y,
				(2.0f * (xz - wy)) * s.x, (2.0f * (yz + wx)) * s.y, (1.0f - 2.0f * (x2 + y2)) * s.z, -t.z,
				0, 0, 0, 1 };
		}
		inline static constexpr mat4f orthographic(const float& n, const float& f, const float& l, const float& r, const float& t, const float& b) {
			const float rl = r - l, tb = t - b, fn = f - n;
			// TODO bottom - top???
			/*return {
				2 / (rl), 0, 0, (r + l) / rl,
				0, 2 / (tb), 0, (t + b) / tb,
				0, 0, -2 / (fn), (f + n) / fn,
				0, 0, 0, 1 };*/
			/*return {
				2 / (rl), 0, 0, -(r + l) / rl,
				0, 2 / (tb), 0, -(t + b) / tb,
				0, 0, -2 / (fn), (f + n) / fn,
				0, 0, -2 / (fn), (f + n) / fn,
				0, 0, 0, 1 };*/
			return {
				2 / (rl), 0, 0, -(r + l) / rl,
				0, 2 / (tb), 0, -(t + b) / tb,
				0, 0, 2 / (fn), -(f + n) / fn,
				0, 0, 0, 1 };
		}
		inline static constexpr mat4f perspective(const float& n, const float& f, const float& l, const float& r, const float& t, const float& b) {
			const float tn = 2 * n, rl = r - l, tb = t - b, fn = f - n;
			/*return {
				(2 * n) / (r - l), 0, (r + l) / (r - l), 0,
				0, (2 * n) / (t - b), (t + b) / (t - b), 0,
				0, 0, (f + n) / (n - f), (2 * f * n) / (n - f),
				0, 0, -1, 0 };*/
				/*return {
					(2 * n) / (r - l), 0, (r + l) / (r - l), 0,
					0, (2 * n) / (t - b), (t + b) / (t - b), 0,
					0, 0, -(f + n) / (n - f), -(2 * f * n) / (n - f),
					0, 0, -1, 0 };*/
			return {
				tn / rl, 0, -(r + l) / rl, 0,
				0, tn / tb, -(t + b) / tb, 0,
				0, 0, (f + n) / fn, -(tn * f) / fn,
				0, 0, -1, 0 };
		}
	};
}

// IMPLEMENTATION
namespace pxl {
	inline constexpr vec2f::operator vec3f() const { return { x, y, 0 }; }
	inline constexpr vec2f::operator vec4f() const { return { x, y, 0, 0 }; }
	inline constexpr vec3f::operator vec2f() const { return { x, y }; }
	inline constexpr vec3f::operator vec4f() const { return { x, y, z, 0 }; }
	inline constexpr vec4f::operator vec2f() const { return { x, y }; }
	inline constexpr vec4f::operator vec3f() const { return { x, y, z }; }
}
namespace std {
#ifdef _IOSTREAM_
	ostream& operator<<(ostream& os, const pxl::vec2f& o) { return os << "{" << o.x << ", " << o.y << "}"; }
	ostream& operator<<(ostream& os, const pxl::vec3f& o) { return os << "{" << o.x << ", " << o.y << ", " << o.z << "}"; }
	ostream& operator<<(ostream& os, const pxl::vec4f& o) { return os << "{" << o.x << ", " << o.y << ", " << o.z << ", " << o.w << "}"; }
	ostream& operator<<(ostream& os, const pxl::quatf& o) { return os << "{" << o.x << ", " << o.y << ", " << o.z << ", " << o.w << "}"; }
	ostream& operator<<(ostream& os, const pxl::mat4f& o) {
		return os << "[{" << o.m00 << ", " << o.m01 << ", " << o.m02 << ", " << o.m03 << "}"
			<< "{" << o.m10 << ", " << o.m11 << ", " << o.m12 << ", " << o.m13 << "}"
			<< "{" << o.m20 << ", " << o.m21 << ", " << o.m22 << ", " << o.m23 << "}"
			<< "{" << o.m30 << ", " << o.m31 << ", " << o.m32 << ", " << o.m33 << "}]";
	}
#endif
}
#endif
