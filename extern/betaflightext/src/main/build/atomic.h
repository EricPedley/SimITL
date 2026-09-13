/*
 * SimITL simulator variant of the ARM BASEPRI based atomic helpers.
 *
 * This header shadows betaflight's build/atomic.h on the include path. The
 * real one contains ARM-only inline assembly (MSR basepri) which does not
 * compile for the host x86 target. The SimITL scheduler is single threaded
 * and never preempts itself, so interrupt masking is a no-op here. The block
 * macros still expand to a single-iteration loop so that break/continue
 * behave the same as in the firmware build.
 */

#pragma once

#include <stdint.h>

#define ATOMIC_BLOCK(prio) \
    for (uint8_t __simitl_once = 1; __simitl_once; __simitl_once = 0)

#define ATOMIC_BLOCK_NB(prio) \
    for (uint8_t __simitl_once = 1; __simitl_once; __simitl_once = 0)

#define ATOMIC_BARRIER(data)

#define ATOMIC_OR(ptr, val) __sync_fetch_and_or(ptr, val)
#define ATOMIC_AND(ptr, val) __sync_fetch_and_and(ptr, val)
