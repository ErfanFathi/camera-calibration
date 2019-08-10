#include "GeometryParameter.h"
#include "Quater.h"

enum x
{
	aroundX,
	aroundY,
	aroundZ
};

bool CurrentSign( double n )
{
	return ( n > 0 ) ? true : false;
}

double cosine( double agl )
{
	double result = cos( agl * Pi / 180 );
	bool sign = CurrentSign( result );
	if ( CurrentSign( result + 0.000000001 ) != sign || CurrentSign( result - 0.000000001 ) != sign )
		return 0;
	return result;
}

double sine( double agl )
{
	double result = sin( agl * Pi / 180 );
	bool sign = CurrentSign( result );
	if ( CurrentSign( result + 0.000000001 ) != sign || CurrentSign( result - 0.000000001 ) != sign )
		return 0;
	return result;
}

Mat get_rotation_Matrix( double angle, int around )
{
	Mat rotation_Mat = Mat::zeros( 3, 3, CV_64F );

	switch ( around )
	{
		case aroundZ:
			rotation_Mat.at<double>( 2, 2 ) = 1;
			rotation_Mat.at<double>( 0, 0 ) = cosine( angle );
			rotation_Mat.at<double>( 0, 1 ) = -sine( angle );
			rotation_Mat.at<double>( 1, 0 ) = sine( angle );
			rotation_Mat.at<double>( 1, 1 ) = cosine( angle );
			break;
		case aroundX:
			rotation_Mat.at<double>( 0, 0 ) = 1;
			rotation_Mat.at<double>( 1, 1 ) = cosine( angle );
			rotation_Mat.at<double>( 1, 2 ) = -sine( angle );
			rotation_Mat.at<double>( 2, 1 ) = sine( angle );
			rotation_Mat.at<double>( 2, 2 ) = cosine( angle );
			break;
		case aroundY:
			rotation_Mat.at<double>( 1, 1 ) = 1;
			rotation_Mat.at<double>( 0, 0 ) = cosine( angle );
			rotation_Mat.at<double>( 0, 2 ) = sine( angle );
			rotation_Mat.at<double>( 2, 0 ) = -sine( angle );
			rotation_Mat.at<double>( 2, 2 ) = cosine( angle );
			break;
		default:
			break;
	}

	return rotation_Mat;
}

GeometryParameter euler2quaternion( double anglex, double angley, double anglez )
{
	Mat r = get_rotation_Matrix( anglez, aroundZ ) * 
            ( get_rotation_Matrix( angley, aroundY ) * 
            get_rotation_Matrix( anglex, aroundX ) );
	Quater Q;
	Q.Quaternions( r );

	GeometryParameter temp;
	temp.set_Q0( Q.X() );
	temp.set_Q1( Q.Y() );
	temp.set_Q2( Q.Z() );
	temp.set_Q3( Q.W() );
	temp.set_Tx( 0 );
	temp.set_Ty( 0 );
	temp.set_Tz( 0 );
	temp.set_Distortion( 0 );
	temp.set_FocalLenght( 0 );
	temp.set_PrincipalPointX( 0 );
	temp.set_PrincipalPointY( 0 );
	return temp;
}