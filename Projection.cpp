#include "Projection.h"

double Projection::dot( Point3d a, Point3d b )
{
	return ( a.x* b.x + a.y * b.y + a.z * b.z );
}

double Projection::ray_plane_intersect( Point3d pOrigin, Point3d pNormal, Point3d rOrigin, Point3d rVector )
{
	return ( dot( -pNormal, ( rOrigin - pOrigin ) ) / ( dot( pNormal, rVector ) ) );
}

double Projection::radialDistortion( double ru, GeometryParameter geo )
{
	if( geo.Distortion() <= DBL_MIN )
		return ru;
	double rd = 0;
	double a = geo.Distortion();
	double b = -0.9 * a * a * ru + a * sqrt( a * ( 12.0 + 81.0 * a * ru * ru ) );
	b = ( b < 0.0 ) ? ( -pow( b, 1.0 / 3.0 ) ) : pow( b, 1.0 / 3.0 );
	rd = pow( 2.0 / 3.0, 1.0 / 3.0 ) / b -
		b / ( pow(2.0 * 3.0 * 3.0, 1.0 / 3.0 ) * a );
	return rd;
}

Point2d Projection::radialDistortion( Point2d pu, GeometryParameter geo )
{
	double len = sqrt( pow( pu.x, 2 ) + pow( pu.y, 2 ) );
	double rd = radialDistortion( len, geo );
	Point2d pd = pu;
	double size = sqrt( pd.x * pd.x + pd.y * pd.y );
	if ( size < Epsilon )
		pd.x = pd.y = 0;
	else
	{
		pd.x *= rd / size;
		pd.y *= rd / size;
	}
	return pd;
}

double Projection::radialDistortionInv( double rd, GeometryParameter geo )
{
	double ru = rd * ( 1.0 + rd * rd * geo.Distortion() );
	return ru;
}

Point2d Projection::radialDistortionInv( Point2d pd, GeometryParameter geo )
{
	double len = sqrt( pow( pd.x, 2 ) + pow( pd.y, 2 ) );
	double ru = radialDistortionInv( len, geo );
	Point2d pu = pd;

	double size = sqrt( pu.x * pu.x + pu.y * pu.y );
	if ( size < Epsilon )
		pu.x = pu.y = 0;
	else
	{
		pu.x *= ru / size;
		pu.y *= ru / size;
	}

	return pu;
}

Point3d Projection::Image2Field( Point3d pI, GeometryParameter geo, double z )
{
	Point2d p_d = Point2d( ( pI.x - geo.PrincipalPointX() ) / geo.FocalLenght(),
		( pI.y - geo.PrincipalPointY() ) / geo.FocalLenght() );

	Point2d p_un = radialDistortionInv( p_d, geo );
	Point3d v = Point3d( p_un.x, p_un.y, 1 );

	Quater q_field2cam = Quater( geo.Q0(), geo.Q1(), geo.Q2(), geo.Q3() );
	q_field2cam.Normalize();
	Point3d translation = Point3d( geo.Tx(), geo.Ty(), geo.Tz() );

	q_field2cam.Invert();
	Point3d v_in_w = q_field2cam.RotateVectorByQuaternion( v );

	Point3d zero = Point3d( 0, 0, 0 );
	Point3d zero_in_w = q_field2cam.RotateVectorByQuaternion( zero - translation );

	Point3d temp0 = Point3d( 0, 0, z );
	Point3d temp1 = Point3d( 0, 0, 1 );

	double size = sqrt( temp1.x * temp1.x + temp1.y * temp1.y + temp1.z * temp1.z );
	if ( size < Epsilon )
		temp1.x = temp1.y = temp1.z = 0;
	else
	{
		temp1.x /= size;
		temp1.y /= size;
		temp1.z /= size;
	}

	double size2 = sqrt( v_in_w.x * v_in_w.x + v_in_w.y * v_in_w.y + v_in_w.z * v_in_w.z );
	if ( size2 < Epsilon )
		v_in_w.x = v_in_w.y = v_in_w.z = 0;
	else
	{
		v_in_w.x /= size2;
		v_in_w.y /= size2;
		v_in_w.z /= size2;
	}

	double t = ray_plane_intersect( temp0, temp1, zero_in_w, v_in_w );

	double size3 = sqrt( v_in_w.x * v_in_w.x + v_in_w.y * v_in_w.y + v_in_w.z * v_in_w.z );
	if ( size3 < Epsilon )
		v_in_w.x = v_in_w.y = v_in_w.z = 0;
	else
	{
		v_in_w.x /= size3;
		v_in_w.y /= size3;
		v_in_w.z /= size3;
	}
	
	Point3d p_f = zero_in_w + v_in_w * t;
	return Point3d( p_f.x, p_f.y, p_f.z );
}

Point2d Projection::Field2Image( Point2d pF, GeometryParameter geo )
{
	Quater q_field2cam = Quater( geo.Q0(), geo.Q1(), geo.Q2(), geo.Q3() );
	q_field2cam.Normalize();
	Point3d translation = Point3d( geo.Tx(), geo.Ty(), geo.Tz() );

	Point3d pFVec = Point3d( pF.x, pF.y, 0 );
	Point3d p_c = q_field2cam.RotateVectorByQuaternion( pFVec ) + translation;

	Point2d p_un = Point2d( p_c.x / p_c.z, p_c.y / p_c.z );
	Point2d p_d = radialDistortion( p_un, geo );

	Point2d princ = Point2d( geo.PrincipalPointX(), geo.PrincipalPointY() );
	Point2d p_i = geo.FocalLenght() * p_d + princ;

	return p_i;
}
