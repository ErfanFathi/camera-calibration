#include "Quater.h"

Quater::Quater( double _x, double _y, double _z, double _w )
{
	x = _x;
	y = _y;
	z = _z;
	w = _w;
}

Quater::Quater()
{

}

void Quater::Normalize()
{
	double mag2 = ( w * w ) + ( x * x ) + ( y * y ) + ( z * z );
	if ( abs( mag2 - 1 ) > Epsilon )
	{
		double mag = sqrt( mag2 );
		if ( mag != 0 )
		{
			w /= mag;
			x /= mag;
			y /= mag;
			z /= mag;
		}
	}
}

void Quater::Invert()
{
	Conjugate();
	double norm = x * x + y * y + z * z + w * w;
	if ( norm != 0 )
	{
		double inv_norm = 1 / norm;
		x *= inv_norm;
		y *= inv_norm;
		z *= inv_norm;
		w *= inv_norm;
	}
}

Point3d Quater::RotateVectorByQuaternion( Point3d v )
{
	double x2 = x * x;
	double y2 = y * y;
	double z2 = z * z;
	double w2 = w * w;
	double xy = x * y;
	double xz = x * z;
	double yz = y * z;
	double wx = w * x;
	double wy = w * y;
	double wz = w * z;

	Point3d result = Point3d( 0, 0, 0 );

	result.x = ( 1.0 - 2.0 * ( y2 + z2 ) ) * v.x + ( 2.0 * ( xy - wz ) ) * v.y + ( 2.0 * ( xz + wy ) ) * v.z;
	result.y = ( 2.0 * ( xy + wz ) ) * v.x + ( 1.0 - 2.0 * ( x2 + z2 ) ) * v.y + ( 2.0 * ( yz - wx ) ) * v.z;
	result.z = ( 2.0 * ( xz - wy ) ) * v.x + ( 2.0 * ( yz + wx ) ) * v.y + ( 1.0 - 2.0 * ( x2 + y2 ) ) * v.z;

	return result;
}

Point3d Quater::Quaternions( Mat rotationMat, Point3d point )
{
	Point3d result;
	double qr[4];

	qr[0] = sqrt( MAX( 0, 1 + rotationMat.at<double>( 0, 0 ) + rotationMat.at<double>( 1, 1 ) + rotationMat.at<double>( 2, 2 ) ) ) / 2;
	qr[1] = sqrt( MAX( 0, 1 + rotationMat.at<double>( 0, 0 ) - rotationMat.at<double>( 1, 1 ) - rotationMat.at<double>( 2, 2 ) ) ) / 2;
	qr[2] = sqrt( MAX( 0, 1 - rotationMat.at<double>( 0, 0 ) + rotationMat.at<double>( 1, 1 ) - rotationMat.at<double>( 2, 2 ) ) ) / 2;
	qr[3] = sqrt( MAX( 0, 1 - rotationMat.at<double>( 0, 0 ) - rotationMat.at<double>( 1, 1 ) + rotationMat.at<double>( 2, 2 ) ) ) / 2;

	qr[1] = copysign( qr[1], rotationMat.at<double>( 2, 1 ) - rotationMat.at<double>( 1, 2 ) );
	qr[2] = copysign( qr[2], rotationMat.at<double>( 0, 2 ) - rotationMat.at<double>( 2, 0 ) );
	qr[3] = copysign( qr[3], rotationMat.at<double>( 1, 0 ) - rotationMat.at<double>( 0, 1 ) );

	result.x = point.x * ( pow( qr[0], 2 ) + pow( qr[1], 2 ) - pow( qr[2], 2 ) - pow( qr[3], 2 ) ) + 
				2 * point.y * ( qr[1] * qr[2] - qr[0] * qr[3] ) + 2 * point.z * ( qr[0] * qr[2] + qr[1] * qr[3] );

	result.y = 2 * point.x * ( qr[0] * qr[3] + qr[1] * qr[2] ) + point.y * ( pow( qr[0], 2 ) - 
				pow( qr[1], 2 ) + pow( qr[2], 2 ) - pow( qr[3], 2 ) ) + 2 * point.z * ( qr[2] * qr[3] - qr[0] * qr[1] );

	result.z = 2 * point.x * ( qr[1] * qr[3] - qr[0] * qr[2] ) + 2 * point.y * ( qr[0] * qr[1] - 
				qr[2] * qr[3] ) + point.z * ( pow( qr[0], 2 ) - pow( qr[1], 2 ) - pow( qr[2], 2 ) + pow( qr[3], 2 ) );

	return result;
}

void Quater::Quaternions( Mat rotationMat )
{

	w = sqrt( MAX( 0, 1 + rotationMat.at<double>( 0, 0 ) + rotationMat.at<double>( 1, 1 ) + rotationMat.at<double>( 2, 2 ) ) ) / 2;
	x = sqrt( MAX( 0, 1 + rotationMat.at<double>( 0, 0 ) - rotationMat.at<double>( 1, 1 ) - rotationMat.at<double>( 2, 2 ) ) ) / 2;
	y = sqrt( MAX( 0, 1 - rotationMat.at<double>( 0, 0 ) + rotationMat.at<double>( 1, 1 ) - rotationMat.at<double>( 2, 2 ) ) ) / 2;
	z = sqrt( MAX( 0, 1 - rotationMat.at<double>( 0, 0 ) - rotationMat.at<double>( 1, 1 ) + rotationMat.at<double>( 2, 2 ) ) ) / 2;

	x = copysign( x, rotationMat.at<double>( 2, 1 ) - rotationMat.at<double>( 1, 2 ) );
	y = copysign( y, rotationMat.at<double>( 0, 2 ) - rotationMat.at<double>( 2, 0 ) );
	z = copysign( z, rotationMat.at<double>( 1, 0 ) - rotationMat.at<double>( 0, 1 ) );

}

void Quater::Conjugate()
{
	x = -x;
	y = -y;
	z = -z;
}
