#ifndef SIGNALS_32SAMPLES_H_
#define SIGNALS_32SAMPLES_H_

float s40x[32] = {-0.88, -0.85, -0.93, -0.98, -0.98, -1.01, -0.82, -1.15, -1.05, -0.80, -0.80, -0.96, -1.00, -0.95, -1.08, -0.88, -1.06, -0.97, -1.07, -1.08, -1.12, -0.95, -0.95, -0.90, -0.99, -0.92, -1.05, -1.04, -1.04, -1.04, -0.99, -0.97};
float s40y[32] = {1.67, 1.66, 1.71, 1.66, 1.76, 1.77, 1.62, 1.60, 1.63, 1.53, 1.88, 1.63, 1.66, 1.58, 1.78, 1.49, 1.82, 1.74, 1.41, 1.52, 1.56, 1.64, 1.72, 1.85, 1.76, 1.73, 1.65, 1.70, 1.57, 1.66, 1.63, 1.78};
float s40z[32] = {-2.31, -2.31, -2.29, -2.37, -2.49, -2.35, -2.39, -2.37, -2.30, -2.30, -2.38, -2.36, -2.44, -2.30, -2.18, -2.19, -2.44, -2.34, -2.26, -2.40, -2.23, -2.57, -2.47, -2.37, -2.35, -2.37, -2.39, -2.63, -2.32, -2.42, -2.47, -2.31};
float s60x[32] = {2.78, -0.16, -0.41, -1.53, 0.64, -0.14, -1.94, -2.01, 0.09, 0.16, -1.76, -2.26, -0.11, 0.43, -2.10, 0.11, -0.05, -1.79, -2.05, 0.16, -0.11, -2.07, -1.94, 0.24, -0.23, -0.08, 0.13, -1.36, -2.19, 0.04, -0.15, -1.79};
float s60y[32] = {25.54, 6.59, 4.18, 1.39, 9.76, 9.00, -0.16, -5.22, 3.80, 7.96, 0.96, -5.82, 2.73, 9.42, -7.11, 7.66, 9.46, -3.28, -7.10, 8.10, 8.85, -4.18, -6.15, 9.44, 7.93, 2.62, 8.79, 1.75, -6.53, 4.21, 8.47, 1.14};
float s60z[32] = {-8.64, -6.28, -4.25, -5.58, -4.93, -7.36, -4.46, 1.56, -0.51, -4.82, -3.99, -1.27, 0.36, -4.55, 1.27, -1.71, -7.57, -1.98, 1.16, -1.87, -7.23, -1.76, 1.51, -2.92, -6.05, 0.05, -5.07, -4.27, 0.35, -1.12, -4.00, -5.26};
float s80x[32] = {-4.20, -7.83, -9.15, -10.24, -5.92, 0.53, 6.31, 8.57, 6.51, 1.24, -5.70, -9.82, -10.76, -8.43, -2.76, 4.09, 7.85, 8.18, 4.04, -2.45, -2.45, -10.85, -10.12, -5.49, 1.17, 6.73, 8.70, 6.10, 0.55, -6.34, -10.09, -10.07};
float s80y[32] = {3.92, -12.65, -9.89, -10.74, -1.95, 10.22, 16.48, 17.70, 8.20, 0.73, -10.38, -13.24, -15.93, -6.58, 4.20, 15.50, 17.29, 14.07, 4.43, -5.57, -5.57, -15.83, -11.67, -1.23, 11.43, 16.42, 17.35, 8.07, -0.41, -11.15, -14.49, -11.33};
float s80z[32] = {-9.06, -2.18, -3.32, -2.15, -2.28, -2.84, -1.72, -2.56, -1.25, -3.60, -2.19, -3.30, -1.47, -3.21, -1.83, -2.81, -1.40, -1.45, -2.27, -2.56, -2.56, -1.69, -2.15, -2.31, -2.97, -1.20, -2.37, -1.33, -3.77, -2.27, -2.69, -2.11};

#endif

/* 40% de duty cycle
Eje X: -0.88, -0.85, -0.93, -0.98, -0.98, -1.01, -0.82, -1.15, -1.05, -0.80, -0.80, -0.96, -1.00, -0.95, -1.08, -0.88, -1.06, -0.97, -1.07, -1.08, -1.12, -0.95, -0.95, -0.90, -0.99, -0.92, -1.05, -1.04, -1.04, -1.04, -0.99, -0.97, 
Eje Y: 1.67, 1.66, 1.71, 1.66, 1.76, 1.77, 1.62, 1.60, 1.63, 1.53, 1.88, 1.63, 1.66, 1.58, 1.78, 1.49, 1.82, 1.74, 1.41, 1.52, 1.56, 1.64, 1.72, 1.85, 1.76, 1.73, 1.65, 1.70, 1.57, 1.66, 1.63, 1.78, 
Eje Z: -2.31, -2.31, -2.29, -2.37, -2.49, -2.35, -2.39, -2.37, -2.30, -2.30, -2.38, -2.36, -2.44, -2.30, -2.18, -2.19, -2.44, -2.34, -2.26, -2.40, -2.23, -2.57, -2.47, -2.37, -2.35, -2.37, -2.39, -2.63, -2.32, -2.42, -2.47, -2.31,
*/

/* 60% de duty cycle
Eje X: 2.78, -0.16, -0.41, -1.53, 0.64, -0.14, -1.94, -2.01, 0.09, 0.16, -1.76, -2.26, -0.11, 0.43, -2.10, 0.11, -0.05, -1.79, -2.05, 0.16, -0.11, -2.07, -1.94, 0.24, -0.23, -0.08, 0.13, -1.36, -2.19, 0.04, -0.15, -1.79, 
Eje Y: 25.54, 6.59, 4.18, 1.39, 9.76, 9.00, -0.16, -5.22, 3.80, 7.96, 0.96, -5.82, 2.73, 9.42, -7.11, 7.66, 9.46, -3.28, -7.10, 8.10, 8.85, -4.18, -6.15, 9.44, 7.93, 2.62, 8.79, 1.75, -6.53, 4.21, 8.47, 1.14, 
Eje Z: -8.64, -6.28, -4.25, -5.58, -4.93, -7.36, -4.46, 1.56, -0.51, -4.82, -3.99, -1.27, 0.36, -4.55, 1.27, -1.71, -7.57, -1.98, 1.16, -1.87, -7.23, -1.76, 1.51, -2.92, -6.05, 0.05, -5.07, -4.27, 0.35, -1.12, -4.00, -5.26, 
*/

/* 80% de duty cycle
Eje X: -4.20, -7.83, -9.15, -10.24, -5.92, 0.53, 6.31, 8.57, 6.51, 1.24, -5.70, -9.82, -10.76, -8.43, -2.76, 4.09, 7.85, 8.18, 4.04, -2.45, -2.45, -10.85, -10.12, -5.49, 1.17, 6.73, 8.70, 6.10, 0.55, -6.34, -10.09, -10.07, 
Eje Y: 3.92, -12.65, -9.89, -10.74, -1.95, 10.22, 16.48, 17.70, 8.20, 0.73, -10.38, -13.24, -15.93, -6.58, 4.20, 15.50, 17.29, 14.07, 4.43, -5.57, -5.57, -15.83, -11.67, -1.23, 11.43, 16.42, 17.35, 8.07, -0.41, -11.15, -14.49, -11.33, 
Eje Z: -9.06, -2.18, -3.32, -2.15, -2.28, -2.84, -1.72, -2.56, -1.25, -3.60, -2.19, -3.30, -1.47, -3.21, -1.83, -2.81, -1.40, -1.45, -2.27, -2.56, -2.56, -1.69, -2.15, -2.31, -2.97, -1.20, -2.37, -1.33, -3.77, -2.27, -2.69, -2.11, 
*/