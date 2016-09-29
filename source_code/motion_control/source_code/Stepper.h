#ifndef stepper_h
#define stepper_h

void st_init(void);
void digipot_init(void);
void microstep_init(void);
void enable_endstops(uint8_t check);
void endstops_hit_on_purpose(void);
void finishAndDisableSteppers(void);

void st_synchronize(void);
void st_wake_up(void);
void st_set_e_position(const long e);
void digipot_current(uint8_t driver, int current);

#endif
