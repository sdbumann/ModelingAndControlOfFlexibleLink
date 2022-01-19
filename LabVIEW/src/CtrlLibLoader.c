// Philippe Schuchert
// SCI-STI-AK
// philippe.schuchert@epfl.ch
// July 2021
/* --------------------------- */

/*
  
 LabVIEW caches shared objects that are loaded. To avoid restarting LabVIEW everytime we want to reload the shared object, make a shared object that loads other shared objects. This (cached) shared object, also implements a more accurate wait until next function.
 
 */

#include <dlfcn.h>
#include <errno.h>
#include <sys/time.h>
#include <unistd.h>

#include "CtrlLibLoader.h"

struct timeval ts;



bool isLoaded = false;



void *ctrlLibHandle;
int (*initFunction)(char[]);
double (*calcFunction)(double [], double []);


int init(char pathLib[], char pathInit[])
{
    
    if (isLoaded) {closeLib();} // close previous lib 
    
    ctrlLibHandle = dlopen(pathLib, RTLD_NOW); // open new lib
    
    if (ctrlLibHandle) {
        // success
        *(void**)(&initFunction) = dlsym(ctrlLibHandle, "initialize");
        *(void**)(&calcFunction) = dlsym(ctrlLibHandle, "calc");
        
        initFunction(pathInit);
        isLoaded = true;
        }
    else
    {
        perror("Library could not be opened");
    }
    
    return *(int *) ctrlLibHandle;
}

void calc(double in[], double out[])
{
    calcFunction(in, out);
}


void closeLib()
{
    dlclose(ctrlLibHandle);
    isLoaded = false;
}


long waitUntilNext(long Ts)
{
    // Ts the wait time in ms
    if (Ts)
    {
   
    unsigned long TsMS = Ts*1000;
    
	unsigned long time_ms = (1000000*ts.tv_sec + ts.tv_usec) % (2*TsMS);
	
	unsigned long DT;
	if (time_ms > TsMS)
	{
           DT = TsMS;
        }
        else{
            DT = 0;
        }
	
    
    while (1) {
        //   spin lock
        gettimeofday(&ts,NULL);
	time_ms = (1000000*ts.tv_sec + ts.tv_usec) % (2*TsMS);
	
	
        if ( (time_ms - DT)  >  TsMS)
        {
            break;
        }
        else{
            usleep(5);
        }
    }
    }
    return (Ts) ? (1000000*ts.tv_sec + ts.tv_usec): -1;
}
