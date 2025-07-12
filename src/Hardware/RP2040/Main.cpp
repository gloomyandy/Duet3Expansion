#include <CoreIO.h>

#if RP2040
#include "pico/runtime_init.h"
extern "C" void runtime_init_usb_power_down(void);
bool dummy = false;
void runtime_init(void) {
#ifndef NDEBUG
    if (__get_current_exception()) {
        // crap; started in exception handler
        __breakpoint();
    }
#endif

#if !PICO_RUNTIME_SKIP_INIT_PER_CORE_INSTALL_STACK_GUARD
    // install core0 stack guard
    extern char __StackBottom;
    runtime_init_per_core_install_stack_guard(&__StackBottom);
#endif

    // todo maybe we want to do this in the future, but it does stuff like register_tm_clones
    //      which we didn't do in previous SDKs
    //extern void __libc_init_array(void);
    //__libc_init_array();

    // ... so instead just do the __preinit_array
    runtime_run_initializers();
    // ... and the __init_array
    extern void (*__init_array_start)(void);
    extern void (*__init_array_end)(void);
    for (void (**p)(void) = &__init_array_start; p < &__init_array_end; ++p) {
        (*p)();
    }
}
// SystemCoreClock is needed by FreeRTOS
uint32_t SystemCoreClock = 125000000;
#include "pico/stdlib.h"
#define LED_PIN 25
extern "C" int main()
{
#if 0
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (1) {
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
        gpio_put(LED_PIN, 0);
        sleep_ms(1000);
    }
#endif
    if (dummy)
        runtime_init_usb_power_down();
    AppMain();
}

#endif

// End
