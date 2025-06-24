#ifndef ATOMIC_H
#define ATOMIC_H

#include <stdint.h>
#include <stm32f765xx.h>
// Macro để vào critical section
#define ENTER_CRITICAL() do { \
    extern volatile uint32_t primask_state; \
    extern volatile uint32_t critical_nest_count; \
    primask_state = __get_PRIMASK(); \
    __disable_irq(); \
    critical_nest_count++; \
} while (0)

// Macro để thoát critical section
#define EXIT_CRITICAL() do { \
    extern volatile uint32_t primask_state; \
    extern volatile uint32_t critical_nest_count; \
    if (critical_nest_count > 0) { \
        critical_nest_count--; \
        if (critical_nest_count == 0) { \
            __set_PRIMASK(primask_state); \
        } \
    } \
} while (0)

// Macro để thực hiện thao tác atomic đơn giản (đọc/ghi)
#define ATOMIC_READ(var) ({ \
    uint32_t val; \
    ENTER_CRITICAL(); \
    val = (var); \
    EXIT_CRITICAL(); \
    val; \
})

#define ATOMIC_WRITE(var, val) do { \
    ENTER_CRITICAL(); \
    (var) = (val); \
    EXIT_CRITICAL(); \
} while (0)

// Hàm atomic để tăng/giảm giá trị
uint32_t atomic_inc(volatile uint32_t *var);
uint32_t atomic_dec(volatile uint32_t *var);

// Hàm atomic để thiết lập và lấy giá trị cũ
uint32_t atomic_set_and_get_old(volatile uint32_t *var, uint32_t new_val);

// Hàm atomic để so sánh và hoán đổi (compare-and-swap)
uint32_t atomic_compare_and_swap(volatile uint32_t *var, uint32_t old_val, uint32_t new_val);

#endif
