#ifndef Quaternion_H
#define Quaternion_H

#include<Arduino.h>

class Quaternion 
{
	public:
		Quaternion(float a0, float b0, float c0, float d0);
		Quaternion(float v[3], float theta);
		Quaternion();
		~Quaternion();
		float getTheta();
		float* getv();
		float geti();
		float getj();
		float getk();
		float geta();
		float getb();
		float getc();
		float getd();
		Quaternion multiplyBy(Quaternion qt);
		Quaternion conj();
		Quaternion multiplyScalar(float x);
		float norm();
		Quaternion inverse();
		Quaternion differenceBetween(Quaternion q1, Quaternion q2);
		
	private:
		float theta;
		float a;
		float b;
		float c;
		float d;
		float i;
		float j;
		float k;
		float v[3];
};

#endif