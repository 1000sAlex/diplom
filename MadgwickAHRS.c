#include "MadgwickAHRS.h"
#include <math.h>

#define M_PI		3.14159265358979323846

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	500.0f		// sample frequency in Hz

#define gyroMeasError 3.14159265358979f * (2.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define Test_beta 0.866 * gyroMeasError // compute beta
#define GyroMeasDrift 3.14159265358979f * (1.1f / 180.0f)    // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
#define Zeta 0.866 * GyroMeasDrift  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value


//---------------------------------------------------------------------------------------------------
// Variable definitions

// parameters for 6 DoF sensor fusion calculations
volatile float beta = Test_beta;// 2 * proportional gain (Kp)
float zeta = Zeta;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float deltat = 1.0/sampleFreq;         // integration interval for both filter schemes
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations
#define INVSQRT 0
#if INVSQRT == 1
float invSqrt(float x);
#else
#define invSqrt(x) 1.0f/sqrtf(x)
#endif
//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MadgwickAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
    {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrtf(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * deltat;
	q1 += qDot2 * deltat;
	q2 += qDot3 * deltat;
	q3 += qDot4 * deltat;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float ax, float ay, float az, float gx, float gy, float gz)
    {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * deltat;
	q1 += qDot2 * deltat;
	q2 += qDot3 * deltat;
	q3 += qDot4 * deltat;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
#if INVSQRT == 1
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
#endif

volatile float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            // vector to hold quaternion


void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz)
{
    float q1_ = q0;
    float q2_ = q1;
    float q3_ = q2;
    float q4_ = q3;         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx = 0,gbiasy = 0,gbiasz = 0;        // gyro bias error
    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1_ = 0.5f * q1_;
    float _halfq2_ = 0.5f * q2_;
    float _halfq3_ = 0.5f * q3_;
    float _halfq4_ = 0.5f * q4_;
    float _2q1_ = 2.0f * q1_;
    float _2q2_ = 2.0f * q2_;
    float _2q3_ = 2.0f * q3_;
    float _2q4_ = 2.0f * q4_;

    // Normalise accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;
    // Compute the objective function and Jacobian
    f1 = _2q2_ * q4_ - _2q1_ * q3_ - ax;
    f2 = _2q1_ * q2_ + _2q3_ * q4_ - ay;
    f3 = 1.0f - _2q2_ * q2_ - _2q3_ * q3_ - az;
    J_11or24 = _2q3_;
    J_12or23 = _2q4_;
    J_13or22 = _2q1_;
    J_14or21 = _2q2_;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;

    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;

    // Normalize the gradient
    norm = sqrtf(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;

    // Compute estimated gyroscope biases
    gerrx = _2q1_ * hatDot2 - _2q2_ * hatDot1 - _2q3_ * hatDot4 + _2q4_ * hatDot3;
    gerry = _2q1_ * hatDot3 + _2q2_ * hatDot4 - _2q3_ * hatDot1 - _2q4_ * hatDot2;
    gerrz = _2q1_ * hatDot4 - _2q2_ * hatDot3 + _2q3_ * hatDot2 - _2q4_ * hatDot1;

    // Compute and remove gyroscope biases
    gbiasx += gerrx * deltat * zeta;
    gbiasy += gerry * deltat * zeta;
    gbiasz += gerrz * deltat * zeta;
    gx -= gbiasx;
    gy -= gbiasy;
    gz -= gbiasz;

    // Compute the quaternion derivative
    qDot1 = -_halfq2_ * gx - _halfq3_ * gy - _halfq4_ * gz;
    qDot2 =  _halfq1_ * gx + _halfq3_ * gz - _halfq4_ * gy;
    qDot3 =  _halfq1_ * gy - _halfq2_ * gz + _halfq4_ * gx;
    qDot4 =  _halfq1_ * gz + _halfq2_ * gy - _halfq3_ * gx;

    // Compute then integrate estimated quaternion derivative
    q1_ += (qDot1 -(beta * hatDot1)) * deltat;
    q2_ += (qDot2 -(beta * hatDot2)) * deltat;
    q3_ += (qDot3 -(beta * hatDot3)) * deltat;
    q4_ += (qDot4 -(beta * hatDot4)) * deltat;

    // Normalize the quaternion
    norm = sqrtf(q1_ * q1_ + q2_ * q2_ + q3_ * q3_ + q4_ * q4_);    // normalise quaternion
    norm = 1.0f/norm;
    q0 = q1_ * norm;
    q1 = q2_ * norm;
    q2 = q3_ * norm;
    q3 = q4_ * norm;
}


// Global system variables
float a_x, a_y, a_z; // accelerometer measurements
float w_x, w_y, w_z; // gyroscope measurements in rad/s
float m_x, m_y, m_z; // magnetometer measurements
float b_x = 1, b_z = 0; // reference direction of flux in earth frame
float w_bx = 0, w_by = 0, w_bz = 0; // estimate gyroscope biases error
// Function to compute one filter iteration
void filterUpdate(float a_x, float a_y, float a_z, float w_x, float w_y, float w_z, float m_x, float m_y, float m_z)
{
    // local system variables
    float norm; // vector norm
    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion rate from gyroscopes elements
    float f_1, f_2, f_3, f_4, f_5, f_6; // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, // objective function Jacobian elements
    J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
    float w_err_x, w_err_y, w_err_z; // estimated direction of the gyroscope error (angular)
    float h_x, h_y, h_z; // computed flux in the earth frame
    // axulirary variables to avoid reapeated calcualtions
    float halfq0 = 0.5f * q0;
    float halfq1 = 0.5f * q1;
    float halfq2 = 0.5f * q2;
    float halfq3 = 0.5f * q3;
    float twoq0 = 2.0f * q0;
    float twoq1 = 2.0f * q1;
    float twoq2 = 2.0f * q2;
    float twoq3 = 2.0f * q3;
    float twob_x = 2.0f * b_x;
    float twob_z = 2.0f * b_z;
    float twob_xq0 = 2.0f * b_x * q0;
    float twob_xq1 = 2.0f * b_x * q1;
    float twob_xq2 = 2.0f * b_x * q2;
    float twob_xq3 = 2.0f * b_x * q3;
    float twob_zq0 = 2.0f * b_z * q0;
    float twob_zq1 = 2.0f * b_z * q1;
    float twob_zq2 = 2.0f * b_z * q2;
    float twob_zq3 = 2.0f * b_z * q3;
    float q0q1;
    float q0q2 = q0 * q2;
    float q0q3;
    float q1q2;
    float q1q3 = q1 * q3;
    float q2q3;
    float twom_x = 2.0f * m_x;
    float twom_y = 2.0f * m_y;
    float twom_z = 2.0f * m_z;
    // normalise the accelerometer measurement
    norm = sqrtf(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x /= norm;
    a_y /= norm;
    a_z /= norm;
    // normalise the magnetometer measurement
    norm = sqrtf(m_x * m_x + m_y * m_y + m_z * m_z);
    m_x /= norm;
    m_y /= norm;
    m_z /= norm;
    // compute the objective function and Jacobian
    f_1 = twoq1 * q3 - twoq0 * q2 - a_x;
    f_2 = twoq0 * q1 + twoq2 * q3 - a_y;
    f_3 = 1.0f - twoq1 * q1 - twoq2 * q2 - a_z;
    f_4 = twob_x * (0.5f - q2 * q2 - q3 * q3) + twob_z * (q1q3 - q0q2) - m_x;
    f_5 = twob_x * (q1 * q2 - q0 * q3) + twob_z * (q0 * q1 + q2 * q3) - m_y;
    f_6 = twob_x * (q0q2 + q1q3) + twob_z * (0.5f - q1 * q1 - q2 * q2) - m_z;
    J_11or24 = twoq2; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * q3;
    J_13or22 = twoq0; // J_12 negated in matrix multiplication
    J_14or21 = twoq1;
    J_32 = 2.0f * J_14or21; // negated in matrix multiplication
    J_33 = 2.0f * J_11or24; // negated in matrix multiplication
    J_41 = twob_zq2; // negated in matrix multiplication
    J_42 = twob_zq3;
    J_43 = 2.0f * twob_xq2 + twob_zq0; // negated in matrix multiplication
    J_44 = 2.0f * twob_xq3 - twob_zq1; // negated in matrix multiplication
    J_51 = twob_xq3 - twob_zq1; // negated in matrix multiplication
    J_52 = twob_xq2 + twob_zq0;
    J_53 = twob_xq1 + twob_zq3;
    J_54 = twob_xq0 - twob_zq2; // negated in matrix multiplication
    J_61 = twob_xq2;
    J_62 = twob_xq3 - 2.0f * twob_zq1;
    J_63 = twob_xq0 - 2.0f * twob_zq2;
    J_64 = twob_xq1;
    // compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;
    // normalise the gradient to estimate direction of the gyroscope error
    norm = sqrtf(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 = SEqHatDot_1 / norm;
    SEqHatDot_2 = SEqHatDot_2 / norm;
    SEqHatDot_3 = SEqHatDot_3 / norm;
    SEqHatDot_4 = SEqHatDot_4 / norm;
    // compute angular estimated direction of the gyroscope error
    w_err_x = twoq0 * SEqHatDot_2 - twoq1 * SEqHatDot_1 - twoq2 * SEqHatDot_4 + twoq3 * SEqHatDot_3;
    w_err_y = twoq0 * SEqHatDot_3 + twoq1 * SEqHatDot_4 - twoq2 * SEqHatDot_1 - twoq3 * SEqHatDot_2;
    w_err_z = twoq0 * SEqHatDot_4 - twoq1 * SEqHatDot_3 + twoq2 * SEqHatDot_2 - twoq3 * SEqHatDot_1;
    // compute and remove the gyroscope baises
    w_bx += w_err_x * deltat * zeta;
    w_by += w_err_y * deltat * zeta;
    w_bz += w_err_z * deltat * zeta;
    w_x -= w_bx;
    w_y -= w_by;
    w_z -= w_bz;
    // compute the quaternion rate measured by gyroscopes
    SEqDot_omega_1 = -halfq1 * w_x - halfq2 * w_y - halfq3 * w_z;
    SEqDot_omega_2 = halfq0 * w_x + halfq2 * w_z - halfq3 * w_y;
    SEqDot_omega_3 = halfq0 * w_y - halfq1 * w_z + halfq3 * w_x;
    SEqDot_omega_4 = halfq0 * w_z + halfq1 * w_y - halfq2 * w_x;
    // compute then integrate the estimated quaternion rate
    q0 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
    q1 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
    q2 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
    q3 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
    // normalise quaternion
    norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
    // compute flux in the earth frame
    q0q1 = q0 * q1; // recompute axulirary variables
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q2q3 = q2 * q3;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    h_x = twom_x * (0.5f - q2 * q2 - q3 * q3) + twom_y * (q1q2 - q0q3) + twom_z * (q1q3 + q0q2);
    h_y = twom_x * (q1q2 + q0q3) + twom_y * (0.5f - q1 * q1 - q3 * q3) + twom_z * (q2q3 - q0q1);
    h_z = twom_x * (q1q3 - q0q2) + twom_y * (q2q3 + q0q1) + twom_z * (0.5f - q1 * q1 - q2 * q2);
    // normalise the flux vector to have only components in the x and z
    b_x = sqrtf((h_x * h_x) + (h_y * h_y));
    b_z = h_z;
}


