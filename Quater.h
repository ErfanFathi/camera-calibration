#ifndef QUATERNION_H
#define QUATERNION_H

#include <opencv2/opencv.hpp>
#include "opencv/cv.h"
#include <math.h>

const double Pi = 3.14159265358979323846;
const double Epsilon = 1e-5;

using namespace cv;
using namespace std;

class Quater
{
	public:
		Quater( double _x, double _y, double _z, double _w );
		Quater();

		double X() { return x; }
		void set_X( double a ) { x = a; }

		double Y() { return y; }
		void set_Y( double a ) { y = a; }

		double Z() { return z; }
		void set_Z( double a ) { z = a; }

		double W() { return w; }
		void set_W( double a ) { w = a; }

		void Normalize();

		void Invert();

		Point3d RotateVectorByQuaternion( Point3d v );

		Point3d Quaternions( Mat rotationMat, Point3d point );

		void Quaternions( Mat rotationMat );

	private:
	
		double x, y, z, w;

		void Conjugate();
};

#endif