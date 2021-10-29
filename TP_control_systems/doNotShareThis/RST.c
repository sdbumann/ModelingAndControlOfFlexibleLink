//
//  controller.c
//
//
//  Created by EPFL on 25.05.21.
//
#include <stdio.h>
#include <string.h>

#include "RST.h"
#include <Accelerate/Accelerate.h>

#define maxOrder 51 // order of TF + 1
double R[maxOrder];
double input[maxOrder];

double S[maxOrder];
double output[maxOrder];

double T[maxOrder];
double ref[maxOrder];

int N = maxOrder;

void initialize(char *pathCTRL)
{
    /*
     pathCTRL is the path to the RST data file. This file should be created with the following function in MATLAB:
     
     function FormatRST(R,S,T)
     % K: controller to test on the active suspenssion
     % will create a dataRST.bin

     % Send the .bin file to acs@epfl.ch
     if numel(T) < numel(R)
         T(numel(R)) = 0;
     end

     name = 'dataRST';

     fileID = fopen(strcat([name,'.bin']), 'w');
     fwrite(fileID, [R;S;T]', 'double','l');
     fclose(fileID);

     end
     */
    FILE *fp;
    fp = fopen(pathCTRL,"rb"); // Open file in mode Read Binary
    
    double N_;
    fread(&N_,1,sizeof(double),fp);  // Number of internal states
    N = (int) N_; // Cast double to int. N could be computed from the size of the file, but it's easier this way.
    
    fread(R,1,N*sizeof(double),fp);  // Copy half the values to NUM
    fread(S,1,N*sizeof(double),fp);  // Copy half the values to NUM
    fread(T,1,N*sizeof(double),fp);  // Copy half the values to NUM
    
    fclose(fp);
    
    memset(input, 0, N*sizeof(double));
    memset(output,0, N*sizeof(double)); // Initialize (past) outputs
    memset(ref,   0, N*sizeof(double)); // Initialize (past) ref
    
    
}

void shift(double array[], double new)
{
    memmove(&array[1], &array[0], (N-1)*sizeof(double));
    array[0] = new;
}


void calc(double in[], double out[])
{
    shift(input, in[1]);
    shift(ref, in[0]);
    
    out[0] = cblas_ddot(N,T,1,ref,1) - cblas_ddot(N,R,1,input,1) - cblas_ddot(N-1,S+	1,1,output,1);
    
    shift(output, out[0]);

}


