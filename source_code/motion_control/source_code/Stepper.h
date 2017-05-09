#ifndef stepper_h
#define stepper_h

#include "planner.h"

#define WRITE_E_STEP(v) WRITE(E0_STEP_PIN, v)
#define NORM_E_DIR() WRITE(E0_DIR_PIN, !INVERT_E0_DIR)
#define REV_E_DIR() WRITE(E0_DIR_PIN, INVERT_E0_DIR)

void st_init(void);
void digipot_init(void);
void microstep_init(void);
void microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2);
void microstep_mode(uint8_t driver, uint8_t stepping_mode);
void enable_endstops(uint8_t check);
void endstops_hit_on_purpose(void);
void finishAndDisableSteppers(void);

void st_synchronize(void);
void st_wake_up(void);
void st_set_e_position(const long e);
void digipot_current(uint8_t driver, int current);
long st_get_position(uint8_t axis);

#endif
