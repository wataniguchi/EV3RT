/*
    app.h

    Copyright © 2023 MS&AD Mode2P. All rights reserved.
*/
#ifndef app_h
#define app_h

#ifdef __cplusplus
extern "C" {
#endif

/* common header files */
#include "ev3api.h"

/* task priorities (smaller number has higher priority) */
#define PRIORITY_UPD_TSK    TMIN_APP_TPRI
#define PRIORITY_MAIN_TASK  (TMIN_APP_TPRI + 1)

/* task periods in micro seconds */
#define PERIOD_UPD_TSK      (60 * 1000)

/* default task stack size in bytes */
#ifndef STACK_SIZE
#define STACK_SIZE          (4096)
#endif /* STACK_SIZE */
#define MAIN_STACK_SIZE     (16384)
#define UPD_STACK_SIZE      (4096)
    
/* prototypes for configuration */
#ifndef TOPPERS_MACRO_ONLY

extern void main_task(intptr_t unused);
extern void update_task(intptr_t unused);
extern void task_activator(intptr_t tskid);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif
#endif /* app_h */
