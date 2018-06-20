/*------------------------------------------------------------------
 *  control.c -- here you can implement your control algorithm
 *		 and any motor clipping or whatever else
 *		 remember! motor input =  0-1000 : 125-250 us (OneShot125)
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */

#include "in4073.h"

#define MIN_RPM 204800 // 200 RPM
#define MAX_RPM 614400 // 600 RPM

// Local functions
void update_motors(void);
void run_filters_and_control(void);

void update_motors(void)
{
	motor[0] = ae[0];
	motor[1] = ae[1];
	motor[2] = ae[2];
	motor[3] = ae[3];
}

void run_filters_and_control(void)
{
	// fancy stuff here
	// control loops and/or filters

	// ae[0] = xxx, ae[1] = yyy etc etc
	update_motors();
}

/* Kun Jiang */
//in this function calculate the values for the ae[] array makis
void calculate_rpm(int32_t Z, int32_t L, int32_t M, int32_t N)
{
	int ae1[4],i;
	//if there is lift force calculate ae[] array values
	if(Z>0)
	{
		//calculate the square of each motor rpm
		ae1[0] = ((Z + 2*M - N) >> 2) + MIN_RPM;
		ae1[1] = ((Z - 2*L + N) >> 2) + MIN_RPM;
		ae1[2] = ((Z - 2*M - N) >> 2) + MIN_RPM;
		ae1[3] = ((Z + 2*L + N) >> 2) + MIN_RPM;

		//minimum rpm
		for(i=0;i<4;i++)
		{
			if(ae1[i]<MIN_RPM)
			{
				ae1[i]=MIN_RPM;
			}
		}
		//maximum rpm
		for(i=0;i<4;i++)
		{
			if(ae1[i]>MAX_RPM)
			{
				ae1[i]=MAX_RPM;
			}
		}

		// scale the values into those can be used in reality
		ae[0]=ae1[0] >> 10;
		ae[1]=ae1[1] >> 10;
		ae[2]=ae1[2] >> 10;
		ae[3]=ae1[3] >> 10;
	}
	//if there is no lift force everything should be shut down
	else if(Z<=0)
	{
		ae[0]=0;
		ae[1]=0;
		ae[2]=0;
		ae[3]=0;
	}
	//update motors
	run_filters_and_control();
}
