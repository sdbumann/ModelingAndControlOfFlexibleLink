// Philippe Schuchert
// SCI-STI-AK
// philippe.schuchert@epfl.ch
// July 2021
/* --------------------------- */

#include <stdio.h>
#include <math.h>
#include "PID.h"

#define MIN(a,b) \
  ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b; })

#define MAX(a,b) \
  ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b; })


// Values we need to "remember" between function calls
double Kp, Ki, Kd, Tf, Ts; // PID parameters
double errorPrev, INTEGRAL, DERIVATIVE; // Memory for PID
double u_max, intPrev; // Memory for anti-windup.


int initialize(char *pathCTRL)
{
    /*
     The function is used to initialize the PID controller.pathCTRL is the path of the file created by
     
    FormatPID(Kp,Ki,Kd,Tf,Ts,u_max);
     
    function FormatPID(varargin)
    % Save the parameters to a PID.bin file, which will be read later by either
    % MATLAB, or LabVIEW

    fileID = fopen('dataPID.bin', 'w');
    fwrite(fileID,[varargin{:}] , 'double','l');
    fclose(fileID);

     end
     */
    
    Kp = 0; Ki = 0; Kd = 0; Tf = 0; Ts = 1; u_max= 0; // Reset all values to default
    
    FILE *fp;
    // Read controller binary file
    fp = fopen(pathCTRL,"rb");
    if (fp == NULL)
    {
        return -1;
    }
    
    fseek(fp, 0L, SEEK_END); // go to end of file
    int N = (int) ftell(fp)/(sizeof(double)); // number of parameters = length of file / sizeof(double)
    fseek(fp, 0L, SEEK_SET); // go back to start
    if (N<=0)
    {
        return -1;
    }
    
    if (N>=1) {fread(&Kp,1,sizeof(double),fp);} // Read Propotional gain
    if (N>=2) {fread(&Ki,1,sizeof(double),fp);} // Read Integral gain
    if (N>=3) {fread(&Kd,1,sizeof(double),fp);} // Read Derivative gain
    if (N>=4) {fread(&Tf,1,sizeof(double),fp);} // Read derivative filtering constant
    if (N>=5) {fread(&Ts,1,sizeof(double),fp);} // Read Sampling time of controller
    if (N>=6) {fread(&u_max,1,sizeof(double),fp);}// Read Antiwindup value
    
    fclose(fp);
    
    if (Ts <=0) {Ts = 1;} // Only digital controllers: assume Ts = 1 when not specified (or wrong value)
    
    // Memory
    INTEGRAL   = 0; // reset integral
    DERIVATIVE = 0; // reset derivative
    errorPrev  = 0; // reset previous error error
    
    // memory for anti-windup
    intPrev    = 0;
    return 0; // Init OK
}

void calc(double in[], double out[])
{
    /*
     The function is used to initialize the PID controller
     PID =
                Ts               1
     Kp + Ki * ------ + Kd * -----------
                z-1         Tf+Ts/(z-1)
     Kp the propotional gain
     Ki the integral gain
     Kd the derivative gain
     Tf the derivative filtering constant
     Ts the sampling time
     
     pathCTRL is the path of the file created by
     
    function FormatPID(varargin)
    % Save the parameters to a PID.bin file, which will be read later by either
    % MATLAB, or LabVIEW

    fileID = fopen('dataPID.bin', 'w');
    fwrite(fileID,[varargin{:}] , 'double','l');
    fclose(fileID);

     end
     */
    
    double error = in[1] - in[0];
    
    // Compute new integral and derivative part from PID controller
    INTEGRAL += errorPrev*Ts; // Integral part
    DERIVATIVE = (Tf>0)? (error-errorPrev)/Tf - (Ts-Tf)/Tf*DERIVATIVE : (error-errorPrev)/Ts; // Derivative part, with filtering

    double u = Kp*error + Ki*INTEGRAL + Kd*DERIVATIVE; // PID with no anti-windup
    
    errorPrev = error; // store for next sample
    
    if (u_max)
    {
        /*
         Anti-windup has been set (u_max different from 0). Conditional integration
         
         */
        if (fabs(u) > u_max)
        {
            INTEGRAL = intPrev;
            u = Kp*error + Ki*INTEGRAL + Kd*DERIVATIVE; // PID with no anti-windup
        }
        intPrev = INTEGRAL;
        u = MIN(MAX(u,-u_max),u_max);
    }

    out[0] = u;
}


