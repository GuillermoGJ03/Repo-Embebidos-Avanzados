#include "fft_prueba.h"

typedef float complex cplx;

int reverse_bit(int n, int length){
    int reverse = 0;
    for(int i = 0; i < length; i++){
        reverse = (reverse << 1) | (n & 1);
        n = n >> 1;
    }
    return reverse;
}

void fft(float *arr){
    int num_bit = log2(size);
    for(int i = 0; i < size; i++){
        xFFT[reverse_bit(i, num_bit)] =  arr[i];
    }

    int q = round(log(size)/log(2));
    for(int s = 1; s < q+1; s++){
        int m = pow(2,s);
        int m2 = m/2;
        cplx w = 1;
        cplx wm = cexp(I*(M_PI/m2));
        for(int j = 1; j < m2+1; j++){
            for(int k = j; k < size+1; k+=m){
                cplx t = w * xFFT[k+m2-1];
                cplx u = xFFT[k-1];
                xFFT[k-1] = u + t;
                xFFT[k+m2-1] = u - t;
            }
            w = w * wm;
        }

    }
}

void print_arr(cplx *arr){
    for(int i = 0; i < size; i++){
        printf("%f + %fj\n", creal(arr[i]), cimag(arr[i]));
    }
}