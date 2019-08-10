#ifndef GEOMETRYPARAMETER_H
#define GEOMETRYPARAMETER_H

class GeometryParameter
{
	public:

		double Distortion() { return distortion; }
		void set_Distortion( double a ) { distortion = a; }

		double FocalLenght() { return focalLenght; }
		void set_FocalLenght( double a ) { focalLenght = a; }

		double PrincipalPointX() { return principalPointX; }
		void set_PrincipalPointX( double a ) { principalPointX = a; }

		double PrincipalPointY() { return principalPointY; }
		void set_PrincipalPointY( double a ) { principalPointY = a; }

		double Q0() { return q0; }
		void set_Q0( double a ) { q0 = a; }

		double Q1() { return q1; }
		void set_Q1( double a ) { q1 = a; }

		double Q2() { return q2; }
		void set_Q2( double a ) { q2 = a; }

		double Q3() { return q3; }
		void set_Q3( double a ) { q3 = a; }

		double Tx() { return tx; }
		void set_Tx( double a ) { tx = a; }

		double Ty() { return ty; }
		void set_Ty( double a ) { ty = a; }

		double Tz() { return tz; }
		void set_Tz( double a ) { tz = a; }

	private:

		double distortion, focalLenght,
				principalPointX, principalPointY,
				q0, q1, q2, q3,
				tx, ty, tz;
};

#endif