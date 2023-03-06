/* SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2021, Silicon Labs
 * Main authors:
 *     - Jérôme Pouiller <jerome.pouiller@silabs.com>
 */
#ifndef UTILS_H
#define UTILS_H

#define min(x, y) ({ \
    typeof(x) _x = (x); \
    typeof(y) _y = (y); \
    _x < _y ? _x : _y;  \
})

#define max(x, y) ({ \
    typeof(x) _x = (x); \
    typeof(y) _y = (y); \
    _x > _y ? _x : _y;  \
})

#define container_of(ptr, type, member)  (type *)((uintptr_t)(ptr) - ((uintptr_t)(&((type *)0)->member)))
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define __CLZ(value) __builtin_clz(value)
#define FIELD_GET(mask, reg) (((reg) & (mask)) >> __CLZ(mask))
#define FIELD_PREP(mask, val) (((val) << __CLZ(mask)) & (mask))

#endif
