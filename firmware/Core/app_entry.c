/**
 * @file    app_entry.c
 * @brief   Weak default for `tp_app_start()`: a bench heartbeat.
 *
 * @details
 * The real application lives in `firmware/Application/app_tasks.c` and owns
 * `tp_app_start()`.  That file is written separately, and the port has to link
 * and be flashable *today*, so this translation unit provides a weak definition
 * with a lower link priority.
 *
 * The default is not a stub that returns.  `main()` on both targets calls
 * `tp_app_start()` and never expects it back, so the default runs a
 * **bench heartbeat**: it reads every sensor the platform seam exposes once a
 * second and logs the result.  On a freshly assembled board with no application
 * image yet, that output is exactly what a bring-up engineer needs — it proves
 * the clock is right, that the ADC scan is running, that both quadrature
 * counters move when the axis is turned by hand, and which direction the
 * discrete inputs read in.  It touches no actuator and drives no motion.
 *
 * It is `weak`, so the moment `firmware/Application/app_tasks.c` is linked in,
 * the linker prefers that definition and this code is dropped from the image.
 *
 * @copyright MIT License, see LICENSE.
 */

/*
 * Detect the real application header *without pulling it in*.  This translation
 * unit only needs to know one thing - whether it still owns `tp_app_start()` -
 * and dragging in the whole application header tree to find that out would make
 * an error in the state machine or the protocol layer break a file that compiles
 * to nothing.  When `app_tasks.h` is present this file is an empty translation
 * unit, full stop.
 */
#if defined(__has_include)
#if __has_include("app_tasks.h")
#define TP_HAVE_APP_TASKS_HEADER 1
#endif
#endif

#if !defined(TP_HAVE_APP_TASKS_HEADER)

#include "app_entry.h"

#include "compiler.h"
#include "config.h"
#include "logger.h"
#include "platform_api.h"

/** @brief Period of the bench heartbeat, ms. */
#define TP_BENCH_PERIOD_MS 1000u

#if defined(TP_PLATFORM_HOST)
#include "platform_host.h"

/**
 * @brief Heartbeats the host build emits before returning.
 *
 * The target never returns from `tp_app_start()`, but the host binary is useful
 * as a CI smoke test: it must exercise the platform seam, print something and
 * exit non-zero-free.  Five seconds of simulated time is enough to prove the
 * physics model, the ADC path and the logger all work together.
 */
#define TP_BENCH_TICKS_HOST 5u
#endif

/**
 * @brief Weak default application entry point: a 1 Hz hardware heartbeat.
 *
 * Marked `TP_WEAK`, so a strong `tp_app_start()` from the application layer
 * silently replaces it.
 *
 * @note Never returns, on either target.
 */
TP_WEAK void tp_app_start(void)
{
    uint32_t tick = 0u;

    LOG_WARN(TP_MOD_MAIN,
             "app_tasks.c is not linked: running the bench heartbeat only "
             "(no control, no motion)");

    for (;;) {
        uint16_t force = 0u;
        uint16_t i_joint = 0u;
        uint16_t i_grip = 0u;
        uint16_t vbus = 0u;
        int32_t  joint = 0;
        int32_t  grip = 0;
        uint8_t  io = 0u;

        (void)tp_platform_adc_read(TP_ADC_FORCE, &force);
        (void)tp_platform_adc_read(TP_ADC_CURRENT_JOINT, &i_joint);
        (void)tp_platform_adc_read(TP_ADC_CURRENT_GRIPPER, &i_grip);
        (void)tp_platform_adc_read(TP_ADC_VBUS, &vbus);
        (void)tp_platform_encoder_read(TP_ENCODER_JOINT, &joint);
        (void)tp_platform_encoder_read(TP_ENCODER_GRIPPER, &grip);
        io = tp_platform_io_read_all();

#if TP_SIMULATION_MODE
        /* On the host the plant only advances when the tick hook is called, so
         * the heartbeat has to drive it or the log would never change. */
        tp_host_platform_advance(TP_BENCH_PERIOD_MS * 1000u);
#else
        tp_platform_delay_ms(TP_BENCH_PERIOD_MS);
#endif

        LOG_INFO(TP_MOD_TEST,
                 "t=%lu ms force=%u joint_i=%u grip_i=%u vbus=%u "
                 "enc=%ld/%ld io=0x%02X killed=%d",
                 (unsigned long)tick, (unsigned)force, (unsigned)i_joint,
                 (unsigned)i_grip, (unsigned)vbus,
                 (long)joint, (long)grip, (unsigned)io,
                 tp_platform_is_killed());
        tick += TP_BENCH_PERIOD_MS;

#if TP_SIMULATION_MODE
        /* Bounded on the host so the resulting binary is runnable as a smoke
         * test; the target loops forever, which is what an embedded image does. */
        if (tick >= (TP_BENCH_TICKS_HOST * TP_BENCH_PERIOD_MS)) {
            LOG_WARN(TP_MOD_TEST, "host bench heartbeat finished after %lu ms",
                     (unsigned long)tick);
            return;
        }
#endif
    }
}

#endif /* !TP_HAVE_APP_TASKS_HEADER */
