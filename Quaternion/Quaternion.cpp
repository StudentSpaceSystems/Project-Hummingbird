#include "Quaternion.h"
#include <math.h>

Quaternion::Quaternion(float a0, float b0, float c0, float d0)
{
	a = a0;
	b = b0;
	c = c0;
	d = d0;
	
	theta = acos(a);
	i = b/sin(theta);
	j = c/sin(theta);
	k = d/sin(theta);
}

Quaternion::Quaternion(float v[3], float theta)
{
	i = v[0];
	j = v[1];
	k = v[2];
	
	a = cos(theta);
	b = i*sin(theta);
	c = j*sin(theta);
	d = k*sin(theta);
}

Quaternion::Quaternion(){}

Quaternion::~Quaternion()
{
}

float Quaternion::getTheta()
{
	return theta;
}

float* Quaternion::getv()
{
	return v;
}

float Quaternion::geti()
{
	return i;
}

float Quaternion::getj()
{
	return j;
}

float Quaternion::getk()
{
	return k;
}

float Quaternion::geta()
{
	return a;
}

float Quaternion::getb()
{
	return b;
}

float Quaternion::getc()
{
	return c;
}

float Quaternion::getd()
{
	return d;
}
Quaternion Quaternion::multiplyBy(Quaternion qt)
{
	float q0 = a;
	float q1 = b;
	float q2 = c;
	float q3 = d;
	float r0 = qt.geta();
	float r1 = qt.getb();
	float r2 = qt.getc();
	float r3 = qt.getd();
	
	float t0 = (r0*q0)-(r1*q1)-(r2*q2)-(r3*q3);
	float t1 = (r0*q1)+(r1*q0)+(r2*q3)-(r3*q2);
	float t2 = (r0*q2)-(r1*q3)+(r2*q0)+(r3*q1);
	float t3 = (r0*q3)+(r1*q2)-(r2*q1)+(r3*q0);
	
	return Quaternion(t0, t1, t2, t3);
}