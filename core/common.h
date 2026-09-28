#pragma once

#include <stddef.h>
#include <stdint.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;
typedef float f32;
typedef double f64;

// A 3D vector: x, y, z, with + - * operators. core/math.h has rotations and transforms.
struct Vec3 {
    f32 x;
    f32 y;
    f32 z;
};
typedef Vec3 v3;

inline v3 operator+(v3 a, v3 b) { return v3{a.x + b.x, a.y + b.y, a.z + b.z}; }
inline v3 operator-(v3 a, v3 b) { return v3{a.x - b.x, a.y - b.y, a.z - b.z}; }
inline v3 operator-(v3 a) { return v3{-a.x, -a.y, -a.z}; }
inline v3 operator*(f32 s, v3 a) { return v3{s * a.x, s * a.y, s * a.z}; }
inline v3 operator*(v3 a, f32 s) { return v3{s * a.x, s * a.y, s * a.z}; }
inline v3 operator*(v3 a, v3 b) { return v3{a.x * b.x, a.y * b.y, a.z * b.z}; }
inline v3 &operator+=(v3 &a, v3 b)
{
    a = a + b;
    return a;
}
inline v3 &operator-=(v3 &a, v3 b)
{
    a = a - b;
    return a;
}
inline v3 &operator*=(v3 &a, f32 s)
{
    a = s * a;
    return a;
}

#define ARRAY_COUNT(array) (sizeof(array) / sizeof((array)[0]))
#define NS_PER_S 1000000000ull
#define KILOBYTE 1024ull
#define MEGABYTE (1024ull * KILOBYTE)
#define GIGABYTE (1024ull * MEGABYTE)
#define PI_F32 3.14159265358979f

// min, max and absolute are overloaded rather than macros, so arguments are evaluated once.
inline f32 min(f32 a, f32 b) { return a < b ? a : b; }
inline f32 max(f32 a, f32 b) { return a > b ? a : b; }
inline f64 min(f64 a, f64 b) { return a < b ? a : b; }
inline f64 max(f64 a, f64 b) { return a > b ? a : b; }
inline u32 min(u32 a, u32 b) { return a < b ? a : b; }
inline u32 max(u32 a, u32 b) { return a > b ? a : b; }
inline u64 min(u64 a, u64 b) { return a < b ? a : b; }
inline u64 max(u64 a, u64 b) { return a > b ? a : b; }
inline i32 min(i32 a, i32 b) { return a < b ? a : b; }
inline i32 max(i32 a, i32 b) { return a > b ? a : b; }
inline f32 absolute(f32 x) { return x < 0.0f ? -x : x; }
inline f64 absolute(f64 x) { return x < 0.0 ? -x : x; }

// CLOCK_MONOTONIC in nanoseconds. Wall time for pacing only; the simulation never reads it.
u64 get_time_ns(void);
// CLOCK_REALTIME in nanoseconds since the Unix epoch: ROS's system time.
u64 get_system_time_ns(void);
void sleep_ns(u64 ns);

// printf-style logging to stderr, prefixed with the level.
void log_info(const char *format, ...) __attribute__((format(printf, 1, 2)));
void log_warning(const char *format, ...) __attribute__((format(printf, 1, 2)));
void log_error(const char *format, ...) __attribute__((format(printf, 1, 2)));

// The whole file, NUL-terminated, from malloc (release with free). NULL if unreadable.
char *read_file(const char *path, u64 *size);

// The directory part of a path, without the trailing slash; "" for a bare file name.
void get_directory(const char *path, char *directory, u32 directory_size);

// 64-bit FNV-1a, for state hashes in determinism checks.
u64 hash_bytes(u64 hash, const void *data, u64 size);
#define HASH_SEED 0xcbf29ce484222325ull

// PCG32 (O'Neill), for sensor noise: small, fast, and the same sequence everywhere.
struct Random {
    u64 state;
    u64 increment; // odd; picks one of 2^63 independent streams
};

Random create_random(u64 seed, u64 stream);
u32 get_random_u32(Random *random);
f32 get_random_f32(Random *random); // uniform in [0, 1)
f32 get_random_gaussian(Random *random); // mean 0, standard deviation 1
