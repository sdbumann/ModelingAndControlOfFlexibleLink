To compile the c code to a shared executable, please use the following commands in the Terminal:

First cd to this folder

Then: 
PID : $ 
clang -shared -undefined dynamic_lookup -o pid.so PID.c
RST : $ clang -shared -undefined dynamic_lookup -o rst.so RST.c -framework Accelerate
SS  : $ clang -shared -undefined dynamic_lookup -o statespace.so StateSpace.c -framework Accelerate

If you are not using the framework accelerate to compute the vector/matrix multiplications, you can omit "-framework Accelerate"