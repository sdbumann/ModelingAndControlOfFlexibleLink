// Philippe Schuchert
// SCI-STI-AK
// philippe.schuchert@epfl.ch
// July 2021
/* --------------------------- */

#include <stdio.h>
#include "PID.h" // Change the include here when troubleshooting the other controller. Don't forget to remove the PID.c and PID.h from the project files (otherwise error as two function have the same name: calc in PID, calc in the new controller).


int main(int argc, const char * argv[]) {

    int hasInitialized = initialize("dataPID.bin");
    // initialize("dataSS.bin");  // For testing StateSpace implementation
    // initialize("dataRST.bin"); // For testing RST implementation
    
    
    if (hasInitialized < 0)
    {
        perror("Initialization failed");
        return -1 ;
    }

    
    double e[2] = {0,1};
    double u[1] = {0};
    
    for (int ii =0; ii < 10; ii++)
    {
        calc(e,u);
        printf("%f \n",u[0]);
    }
}

/*
 PID: the output using the PID controller should be:
 1.750000
 1.762500
 1.821875
 1.916406
 2.037305
 2.177979
 2.333484
 2.500113
 2.675085
 2.856314
 
 */
/*
 SS: the output using the State Space controller should be:
 0.000000
 -1.374569
 0.795480
 0.108485
 -0.893152
 -0.993247
 -0.887900
 -0.988361
 -0.887695
 -0.994989
 */

/*
 RST: the output using the RST controller should be:
0.000000
 -0.336044
 -1.013935
 -0.973569
 -1.440130
 -1.396436
 -1.709241
 -1.652159
 -1.846613
 -1.781406
 */
