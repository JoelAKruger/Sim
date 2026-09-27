#include "core/common.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

u64 get_time_ns(void)
{
    timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (u64)now.tv_sec * NS_PER_S + (u64)now.tv_nsec;
}

u64 get_system_time_ns(void)
{
    timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    return (u64)now.tv_sec * NS_PER_S + (u64)now.tv_nsec;
}

void sleep_ns(u64 ns)
{
    timespec duration = {.tv_sec = (time_t)(ns / NS_PER_S), .tv_nsec = (long)(ns % NS_PER_S)};
    nanosleep(&duration, NULL);
}

static void write_log(const char *level, const char *format, va_list args)
{
    char line[1024];
    vsnprintf(line, sizeof(line), format, args);
    fprintf(stderr, "[regolith] %s %s\n", level, line);
}

void log_info(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    write_log("info ", format, args);
    va_end(args);
}

void log_warning(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    write_log("warn ", format, args);
    va_end(args);
}

void log_error(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    write_log("error", format, args);
    va_end(args);
}

u64 hash_bytes(u64 hash, const void *data, u64 size)
{
    const u8 *bytes = (const u8 *)data;
    for (u64 i = 0; i < size; i++) {
        hash ^= bytes[i];
        hash *= 0x100000001b3ull;
    }
    return hash;
}

char *read_file(const char *path, u64 *size)
{
    FILE *file = fopen(path, "rb");
    if (!file) {
        return NULL;
    }
    fseek(file, 0, SEEK_END);
    long length = ftell(file);
    fseek(file, 0, SEEK_SET);
    char *data = length >= 0 ? (char *)malloc((u64)length + 1) : NULL;
    if (data) {
        *size = fread(data, 1, (u64)length, file);
        data[*size] = 0;
    }
    fclose(file);
    return data;
}

void get_directory(const char *path, char *directory, u32 directory_size)
{
    const char *slash = strrchr(path, '/');
    u32 length = slash ? (u32)(slash - path) : 0;
    if (slash == path) {
        length = 1; // the root directory keeps its slash
    }
    length = min(length, directory_size - 1);
    memcpy(directory, path, length);
    directory[length] = 0;
}

Random create_random(u64 seed, u64 stream)
{
    Random random = {.state = 0, .increment = stream << 1u | 1u};
    get_random_u32(&random);
    random.state += seed;
    get_random_u32(&random);
    return random;
}

u32 get_random_u32(Random *random)
{
    u64 old = random->state;
    random->state = old * 6364136223846793005ull + random->increment;
    u32 shifted = (u32)(((old >> 18u) ^ old) >> 27u);
    u32 rotation = (u32)(old >> 59u);
    return shifted >> rotation | shifted << ((32u - rotation) & 31u);
}

f32 get_random_f32(Random *random) { return (f32)(get_random_u32(random) >> 8) * 0x1.0p-24f; }

// Box-Muller with libm, so it is identical wherever the same build runs.
f32 get_random_gaussian(Random *random)
{
    f32 u = 1.0f - get_random_f32(random); // (0, 1], so the log is finite
    f32 v = get_random_f32(random);
    return sqrtf(-2.0f * logf(u)) * cosf(2.0f * PI_F32 * v);
}
