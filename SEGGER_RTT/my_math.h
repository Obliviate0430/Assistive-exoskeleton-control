#ifndef __MYMATH_H
#define __MYMATH_H

#define Pi 3.1415926

extern int Round(float x);
extern int Floor(float x);
extern float (float x1,float y1,float x2,float y2,float n);
extern float Linear_interpolation(float x1,float y1,float x2,float y2,float n);
extern float Linear_Velocity_interpolation(float x1,float y1,float x2,float y2);
extern int RoundToNearest(float num);
extern float DisplacementToDegree(float Displacement);
extern float ForceToDisplacement(int force);
extern float absoluteValue(float num);
extern float DegreeToDisplacement(float Degree);
#endif
