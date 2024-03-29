#ifndef app_h
#define app_h

#ifdef __cplusplus
extern "C" {
#endif

/* common header files */
#include "ev3api.h"
#include "target_test.h"

/* task priorities (smaller number has higher priority) */
#define PRIORITY_UPD_TSK    TMIN_APP_TPRI
#define PRIORITY_MAIN_TASK  TMIN_APP_TPRI

/* task periods in micro seconds */
#define PERIOD_UPD_TSK      (60 * 1000)

/* default task stack size in bytes */
#ifndef STACK_SIZE
#define STACK_SIZE      4096
#endif /* STACK_SIZE */
    
/* prototypes for configuration */
#ifndef TOPPERS_MACRO_ONLY

extern void main_task(intptr_t unused);
extern void update_task(intptr_t unused);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif
#endif /* app_h */
