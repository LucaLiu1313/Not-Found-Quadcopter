#ifndef   _IMU_H_
#define   _IMU_H_

//void Prepare_Data(void);
float invSqrt(float x);
void quaternion_unit(float * a, float * b, float * c,float * d);
void IMU_update(float *roll,float *pitch,float *yaw);

#endif
