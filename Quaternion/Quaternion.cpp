#include "Quaternion.h"
#include <math.h>

Quaternion::Quaternion(float a0, float b0, float c0, float d0)
{
	a = a0;
	b = b0;
	c = c0;
	d = d0;
	
	theta = 2*acos(a);
	i = b/sin(theta/2);
	j = c/sin(theta/2);
	k = d/sin(theta/2);
}

Quaternion::Quaternion(float v[3], float theta)
{
	i = v[0];
	j = v[1];
	k = v[2];
	
	a = cos(theta/2);
	b = i*sin(theta/2);
	c = j*sin(theta/2);
	d = k*sin(theta/2);
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

Quaternion Quaternion::conj()
{
	return Quaternion(a,b * -1,c * -1, d * -1);
}

Quaternion Quaternion::multiplyScalar(float x)
{
	return Quaternion(a*x, b*x, c*x, d*x);
}

float Quaternion::norm()
{
	return(sqrt(pow(a,2)+pow(b,2)+pow(c,2)+pow(d,2)));	
}

Quaternion Quaternion::inverse()
{
	Quaternion q = *this;
	q = q.conj();
	float m = q.norm();
	m = 1/pow(m,2);
	return q.multiplyScalar(m);
}

Quaternion Quaternion::differenceBetween(Quaternion q1, Quaternion q2)
{
	float a1 = q1.geta();
	float b1 = q1.getb();
	float c1 = q1.getc();
	float d1 = q1.getd();
	float a2 = q2.geta();
	float b2 = q2.getb();
	float c2 = q2.getc();
	float d2 = q2.getd();
	
	Quaternion q = Quaternion(a1-a2,b1-b2,c1-c2,d1-d2);
	return q;
}