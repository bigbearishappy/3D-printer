#ifndef MARLIN_BASE_H
#define MARLIN_BASE_H

//#include "Configuration.h"
#include"Marlin.h"
#include<stdio.h>

void system_init(void);								//initialize the system		
void setup_photpin(void);							//initialize the photo pin(if the photo is supported)

int fputc(int ch, FILE *f);							//it's related with the usart data transmission
int GetKey (void);

#endif
