/**
 * @file    app_entry.h
 * @brief   The one symbol `main()` needs from the application layer.
 *
 * @details
 * `tp_app_start()` is the shared application entry point.  Its canonical
 * declaration lives in `firmware/Core/app_tasks.h`, together with the task
 * entry points and the queues the application creates.
 *
 * This header exists so the port can be built **before** `app_tasks.h` lands,
 * and keeps working **after** it does, without either file having to be edited:
 *
 *   - If `app_tasks.h` is present, `__has_include` pulls it in and its
 *     declaration is used.  Nothing else changes.
 *   - If it is not, the prototype is declared here.
 *
 * Either way the definition comes from `firmware/Core/app_entry.c`, which
 * supplies a **weak** default.  When the real `firmware/Application/app_tasks.c`
 * is linked in, its strong definition overrides the weak one automatically and
 * the bench heartbeat in `app_entry.c` disappears from the image — no
 * conditional compilation, no edit, no duplicate-symbol error.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_APP_ENTRY_H
#define TP_APP_ENTRY_H

#if defined(__has_include)
#if __has_include("app_tasks.h")
#include "app_tasks.h"
#define TP_HAVE_APP_TASKS_HEADER 1
#endif
#endif

#ifndef TP_HAVE_APP_TASKS_HEADER
/**
 * @brief Start the application: create the tasks and never return.
 *
 * Called once from `main()` after the platform seam and the logger are up.
 * On the bare-metal OSAL backend it runs the cooperative scheduler loop; on the
 * FreeRTOS backend it creates the tasks and starts the scheduler.  It contains
 * no hardware access of its own — everything below it is the platform seam.
 *
 * @note Declared here only until `firmware/Core/app_tasks.h` exists; see the
 *       file header.
 */
void tp_app_start(void);
#endif /* !TP_HAVE_APP_TASKS_HEADER */

#endif /* TP_APP_ENTRY_H */
