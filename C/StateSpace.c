// Philippe Schuchert
// SCI-STI-AK
// philippe.schuchert@epfl.ch
// July 2021
/* --------------------------- */

#include <Accelerate/Accelerate.h>
#include <stdio.h>
#include "StateSpace.h"

// Values we need to "remember" between function calls
#define maxOrder 50
int N = maxOrder;

double A[maxOrder*maxOrder];
double B[maxOrder];
double C[maxOrder];
double D;

double X[maxOrder]; // Internal states



void initialize(char *pathCTRL)
{
    /*
     pathCTRL is the path to the State-space data file. This file should be created with the following function in MATLAB:
     function FormatSS(A,B,C,D)
     % Inputs: A,B,D,C state-space matrices, or a LTI model
     if nargin == 1
         % If only one argument, assume "A" is a LTI model
         [A,B,C,D] = ssdata(A);
     end
     fileID = fopen('dataSS.bin', 'w');
     fwrite(fileID, [numel(B);A(:);B(:);C(:);D], 'double','l');
     fclose(fileID);
     end
     */
    FILE *fp;
    fp = fopen(pathCTRL,"rb"); // Open file in mode Read Binary
    
    double N_;
    fread(&N_,1,sizeof(double),fp);  // Number of internal states
    N = (int) N_; // Cast double to int. N could be computed from the size of the file, but it's easier this way.
    
    fread(A,1,N*N*sizeof(double),fp);  // Copy N^2 values to A
    fread(B,1,N*sizeof(double),fp);  // Copy N values to B
    fread(C,1,N*sizeof(double),fp);  // Copy N values to C
    fread(&D,1,sizeof(double),fp);  // Copy 1 value to D
    fclose(fp);
    
    catlas_dset(N,0,X,1); // Set initial conditions to zero.
}


void calc(double in[], double out[])
{
    /*
     Inputs to calc
     in[0] = y[k] the measurment
     in[1] = r[k] the reference
     
     Outputs to calc
     out[0] = the control input
     
     This function computes the control values for SISO State Space controllers
     
     N      number of states
     
     A[N]   State-transition matrix (as vector). A[i][j] is equivalent to A[i+N*j]
     B[N]   Input matrix
     C[N]   Output matrix matrix
     D      Feedthrough "matrix"
     
     X[N]   The internal states
     
     Compute
        x[k+1] = A*x[k] + B*u[k]
        y[k]   = C*x[c] + D*u[k]
     Here, this state-space system described a controller. Do not to confuse u[k] with the control input (the voltage of the DC motor), it is the input to the state-space controller. y[k] is the control input and u[k] corresponds to the tracking error.
     
     Some usefull functions from the Accelerate framework:
     cblas_dgemv    (matrix vector multiplications) https://developer.apple.com/documentation/accelerate/1513338-cblas_dgemv
     cblas_ddot     (vector vector dot product)     https://developer.apple.com/documentation/accelerate/1513214-cblas_dcopy
     catlas_daxpby  (sum two vector with scaling)   https://developer.apple.com/documentation/accelerate/1513170-catlas_daxpby
     cblas_dcopy    (copy a vector into another)    https://developer.apple.com/documentation/accelerate/1513214-cblas_dcopy
     */
    
    ???
}


