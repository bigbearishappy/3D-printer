#ifndef stepper_h
#define stepper_h

void st_init(void);
void digipot_init(void);
void microstep_init(void);
void enable_endstops(uint8_t check);
void endstops_hit_on_purpose(void);

void st_synchronize(void);

#endif
