#ifndef PROJECTION_H
#define PROJECTION_H

#include "Quater.h"
#include "GeometryParameter.h"

class Projection
{
	public:
		double dot( Point3d a, Point3d b );

		double ray_plane_intersect( Point3d pOrigin, Point3d pNormal, Point3d rOrigin, Point3d rVector );

		double radialDistortion( double ru, GeometryParameter geo );

		double radialDistortionInv( double rd, GeometryParameter geo );

		Point2d radialDistortion( Point2d pu, GeometryParameter geo );

		Point2d radialDistortionInv( Point2d pd, GeometryParameter geo );

		Point3d Image2Field( Point3d pI, GeometryParameter geo, double z );
		
		Point2d Field2Image( Point2d pF, GeometryParameter geo );
};

#endif