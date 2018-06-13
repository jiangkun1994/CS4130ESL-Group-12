#ifndef FILTER_H__
#define FILTER_H__

#define cons_a0 969
#define cons_a1 969
#define cons_b0 16383
#define cons_b1 -14444
#define P2PHI	0.0081//265
#define C1 		256//63
#define C2		1000000//2

void butterworth();
void kalman();

#endif

// #ifndef FILTER_H__
// #define FILTER_H__

// // All in Q14
// #define BUTTER_A1 969 					
// #define BUTTER_A2 969
// #define BUTTER_B1 16383 			
// #define BUTTER_B2 -14444  		

// #define KALMAN_P2PHI 150 
// #define KALMAN_C1 80 
// #define KALMAN_C3 8

// //#define MOVING_AVERAGE_SIZE 10

// void butterworth();
// void kalman();
// //void moving_average();

// #endif