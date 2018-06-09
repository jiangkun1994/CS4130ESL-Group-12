#include "filter.h"
#include "fixedpoint.h"
#include "in4073.h"

// initialize the 6x2 array
int32_t x[6][2] = {0};
int32_t y[6][2] = {0};

int32_t sp_32, sq_32, sr_32 = 0;
int32_t sax_32, say_32, saz_32 = 0;
int32_t phi_32, theta_32, psi_32 = 0;
int32_t p_b, q_b, r_b = 0;

// program this part according to the MATLAB given by teacher. Note: constants needed to test with data logging
void butterworth()
{
	int i = 0;
	x[0][0] = _16to32(sp);
	x[1][0] = _16to32(sq);
	x[2][0] = _16to32(sr);
	x[3][0] = _16to32(sax);
	x[4][0] = _16to32(say);
	x[5][0] = _16to32(saz);

	while(i < 6)
	{
		y[i][0] = (multiply(cons_a0, x[i][0]) + multiply(cons_a1, x[i][1]) - multiply(cons_b1, y[i][1])) >> 14;

		x[i][1] = x[i][0];
		y[i][1] = y[i][0];
		i++;
	}

	sp = _32to16(y[0][0]);
	sq = _32to16(y[1][0]);
	sr = _32to16(y[2][0]);
	sax = _32to16(y[3][0]);
	say = _32to16(y[4][0]);
	saz = _32to16(y[5][0]);
}

// program this part according to the MATLAB given by teacher. Note: constants needed to test with data logging
void kalman()
{
	sp_32 = _16to32(sp);
	sq_32 = _16to32(sq);
	sr_32 = _16to32(sr);

	sax_32 = _16to32(sax);
	say_32 = _16to32(say);
	saz_32 = _16to32(saz);

	// say is sphi, which means when roll, the drone will be prone to move by y axis
	sp_32 = sp_32 - p_b;
	phi_32 = phi_32 + multiply(sp_32, flo2fix(P2PHI)) >> 14;
	phi_32 = phi_32 - multiply((phi_32 - say_32), (1 >> 14)) >> 14;
	p_b = p_b + multiply(multiply((phi_32 - say_32), flo2fix(123.4567901)) >> 14, flo2fix(0.000001)) >> 14;

	// sax is stheta, which means when pitch, the drone will be prone to move by x axis
	sq_32 = sq_32 - q_b;
	theta_32 = theta_32 + multiply(sq_32, flo2fix(P2PHI)) >> 14;
	theta_32 = theta_32 - multiply((theta_32 - sax_32), (1 >> 14)) >> 14;
	q_b = q_b + multiply(multiply((theta_32 - sax_32), flo2fix(123.4567901)) >> 14, flo2fix(0.000001)) >> 14;

	sr_32 = sr_32 - r_b;
	psi_32 = psi_32 + multiply(sr_32, flo2fix(P2PHI)) >> 14;
	psi_32 = psi_32 - multiply((psi_32 - saz_32), (1 >> 14)) >> 14;
	r_b = r_b + multiply(multiply((psi_32 - saz_32), flo2fix(123.4567901)) >> 14, flo2fix(0.000001)) >> 14;

	sp = _32to16(sp_32);
	sq = _32to16(sq_32);
	sr = _32to16(sr_32);

	phi = _32to16(phi_32);
	theta = _32to16(theta_32);
	psi = _32to16(psi_32);
}