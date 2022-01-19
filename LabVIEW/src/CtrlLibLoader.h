// Philippe Schuchert
// SCI-STI-AK
// philippe.schuchert@epfl.ch
// July 2021
/* --------------------------- */

#ifndef libLoader_h
#define libLoader_h


#include <stdio.h>

void calc(double [], double []);
int init(char pathLib[], char pathInit[]);
void closeLib();
long waitUntilNext(long);
#endif /* libLoader_h */
