#include "core/allocator.h"

#include <stdlib.h>
#include <string.h>

bool create_allocator(Linear_Allocator *allocator, u64 capacity)
{
    allocator->base = (u8 *)malloc(capacity);
    allocator->capacity = allocator->base ? capacity : 0;
    allocator->used = 0;
    if (!allocator->base) {
        log_error("allocator: failed to reserve %llu bytes", (unsigned long long)capacity);
        return false;
    }
    return true;
}

void destroy_allocator(Linear_Allocator *allocator)
{
    free(allocator->base);
    *allocator = {};
}

void reset_allocator(Linear_Allocator *allocator) { allocator->used = 0; }

void *allocate_memory(Linear_Allocator *allocator, u64 size, u64 align)
{
    u64 start = (allocator->used + align - 1) & ~(align - 1);
    if (start + size > allocator->capacity) {
        log_error("allocator: out of memory (%llu of %llu bytes used, %llu requested)",
                  (unsigned long long)allocator->used, (unsigned long long)allocator->capacity,
                  (unsigned long long)size);
        return NULL;
    }
    allocator->used = start + size;
    memset(allocator->base + start, 0, size);
    return allocator->base + start;
}
