#pragma once

#include "core/common.h"

// Bump allocator. Memory with the same lifetime comes from one allocator and is released
// all at once; nothing is freed individually, and nothing allocates inside the step loop.
struct Linear_Allocator {
    u8 *base;
    u64 capacity;
    u64 used;
};

bool create_allocator(Linear_Allocator *allocator, u64 capacity);
void destroy_allocator(Linear_Allocator *allocator);
void reset_allocator(Linear_Allocator *allocator);

// Zeroed memory, or NULL (with an error logged) when the allocator is full.
void *allocate_memory(Linear_Allocator *allocator, u64 size, u64 align);

#define ALLOCATE_ARRAY(allocator, type, count) \
    ((type *)allocate_memory((allocator), sizeof(type) * (u64)(count), alignof(type)))
