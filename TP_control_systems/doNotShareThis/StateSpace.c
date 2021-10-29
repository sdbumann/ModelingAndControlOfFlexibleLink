
//
//  controller.c
//
//
//  Created by EPFL on 25.05.21.
//
#include <stdio.h>

#include "StateSpace.h"
#include <Accelerate/Accelerate.h>

#define maxOrder 50
double A[maxOrder*maxOrder];
double B[maxOrder];

double C[maxOrder];
double D;

double B[maxOrder];

double X[maxOrder];
double X_[maxOrder];

int N = maxOrder;

void initialize(char *pathCTRL)
{
    /*
     pathCTRL is the path to the State-space data file. This file should be created with the following function in MATLAB:
     
     function FormatSS(G)

         [A,B,C,D] = ssdata(G);

          name = 'dataSS';

          fileID = fopen(strcat([name,'.bin']), 'w');
          fwrite(fileID, [numel(B);A(:);B(:);C(:);D], 'double','l');
          fclose(fileID);

     end
     */
    FILE *fp;
    fp = fopen(pathCTRL,"rb"); // Open file in mode Read Binary
    
    double N_;
    fread(&N_,1,sizeof(double),fp);  // Copy half the values to NUM
    N = (int) N_; // Cast double to int. N could be computed from the size of the file, but it's easier this way.
    
    fread(A,1,N*N*sizeof(double),fp);  // Copy half the values to NUM
    fread(B,1,N*sizeof(double),fp);  // Copy half the values to NUM
    fread(C,1,N*sizeof(double),fp);  // Copy half the values to NUM
    fread(&D,1,sizeof(double),fp);  // Copy half the values to NUM
    fclose(fp);
    
    catlas_dset(N,0,X,1);

    
}


void calc(double in[], double out[])
{
    
    double error = in[1] - in[0];
    out[0] = cblas_ddot(N,C,1,X,1) + D*error; // y = C*x + D*u
    
    cblas_dcopy(N,X,1,X_,1); // Make a copy of X
    
    cblas_dgemv(CblasColMajor, CblasNoTrans, N, N, 1, A, N, X_, 1, 0, X, 1); // X <- A*X_, where X_ is a copy of X (otherwise we can overwrite values before when we shouldn't)
    
    catlas_daxpby(N, error, B, 1, 1, X, 1); // X <- b*U + 1*X
}


