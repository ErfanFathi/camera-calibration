#include <iostream>
#include "ChessPatternDetector.h"
#include "RotationMatrix.h"
#include "Projection.h"
#include "PSO.h"

static vector<Point2f> corners;
Point3d realPoint[9];
GeometryParameter geometry;

/*
func : 
func is our cost function . it calculate error of particles
in pso optimization .
it give x which include in intrinsic and extrinsic parameters of camera .
then transform rotation angles to "quaternion" and rotate 
points then calculate error with euclidean distance .
*/
auto func( vector<double> x ) -> double
{
	Projection pj;

	geometry = euler2quaternion( x[0], x[1], x[2] );
	geometry.set_Tx( x[3] );
	geometry.set_Ty( x[4] );
	geometry.set_Tz( x[5] );
	geometry.set_Distortion( x[6] );
	geometry.set_FocalLenght( x[7] );
	geometry.set_PrincipalPointX( x[8] );
	geometry.set_PrincipalPointY( x[9] );

	Point3d points[9];
	for( int i = 0; i < 9; i++)
	{
		points[i] = Point3d( corners[i].x, corners[i].y, 0 );
	}


	long double error = 0;
	Point3d dayy;
	Point2d dayy2;

	for ( int n = 0; n < 9; n++ )
	{
		Point3d poi = Point3d( corners[n].x, corners[n].y, 0 );
		dayy2 = pj.Field2Image( Point2d( realPoint[n].x, realPoint[n].y ), geometry );
		dayy = pj.Image2Field( poi, geometry, 0 );
		error += sqrt( ( pow( corners[n].x - dayy2.x, 2 ) ) + ( pow( corners[n].y - dayy2.y, 2 ) ) );
		error += sqrt( ( pow( realPoint[n].x - dayy.x, 2 ) ) + ( pow( realPoint[n].y - dayy.y, 2 ) ) );
	}

	return error;
}

auto main() -> int
{
    int cameraIndex(1); // Check index of your camera in "/dev"
	double num_h(3); // Number of hight chess corners
	double num_w(3); // Number of width chess corners
	int iteration_number(20); // Number of pso iteration


	/* 
	get_pattern_point : Detect corners in image and 
	sort that the return corners (x, y) 
	and Mattrix of image 
	*/
	auto Res = get_pattern_points(cameraIndex, num_h, num_w);
	corners = Res.first;
    Mat frame = Res.second;


	/*
	Input the dstance of camera to corners respectively.
	Parameters :             X    Y   Z = 0  
	note that parameters are in MM .
	*/
	realPoint[0] = Point3d( 290, -50, 0 );
	realPoint[1] = Point3d( 290, 0, 0 );
	realPoint[2] = Point3d( 290, 50, 0 );
	realPoint[3] = Point3d( 240, -50, 0 );
	realPoint[4] = Point3d( 240, 0, 0 );
	realPoint[5] = Point3d( 240, 50, 0 );
	realPoint[6] = Point3d( 190, -50, 0 );
	realPoint[7] = Point3d( 190, 0, 0 );
	realPoint[8] = Point3d( 190, 50, 0 );

	double err(500);
	pair<vector<double>, double> result;
	
	/*
	Optimization process : 
	This part of code very important because pso algorithm 
	very parameter based so pay attention to the parameters.
	rand parameter give two number : 
	first  :: minimum of generated random.
	second :: maximum of generated random.
	*/ 
	PSO *pso;
	for ( int i = 0; i < iteration_number; i++ )
	{
		PSO *t = new PSO;

		t->set_numberOfParticles( 200 );
		t->set_numberOfDimension( 10 );
		t->set_maximumOfIteration( 750 );
		t->set_rand_rx( -90, 90 );
		t->set_rand_ry( -90, 90 );
		t->set_rand_rz( -90, 90 );
		t->set_rand_tx( -10, 10 );
		t->set_rand_ty( -10, 10 );
		t->set_rand_tz( -10, 10 );
		t->set_rand_distortion( -0.01, 0.01 );
		t->set_rand_focalLength( 500, 520 );
		t->set_rand_principalPointX( 319, 321 );
		t->set_rand_principalPointY( 239, 241 );
		t->set_errorCon( 30 );
		t->set_w( 0.5 );
		t->set_c1( 0.2 );
		t->set_c2( 1.8 );

		auto result_t = t->optimize( func );

		cout << " *********************** " << endl;
		cout << " Iteration : " << i + 1 << endl;
		cout << " Error     : " << result_t.second << endl;
		cout << " *********************** " << endl;

		if ( result_t.second < err )
		{
			err = result_t.second;
			pso = t;
			result = result_t;
		}

		if ( result_t.second < 60.0 )
			break;
	}

	// Print Results
	cout << " ********************************************** " << endl;
	cout << " Best Error        : " << result.second   << endl;
	cout << " Rotation X        : " << result.first[0] << endl;
	cout << " Rotation Y        : " << result.first[1] << endl;
	cout << " Rotation Z        : " << result.first[2] << endl;
	cout << " Translation X     : " << result.first[3] << endl;
	cout << " Translation Y     : " << result.first[4] << endl;
	cout << " Translation Z     : " << result.first[5] << endl;
	cout << " Distortion        : " << result.first[6] << endl;
	cout << " Focal Lenght      : " << result.first[7] << endl;
	cout << " Principal Point X : " << result.first[8] << endl;
	cout << " Principal Point Y : " << result.first[9] << endl;
	cout << " ********************************************** " << endl;

	//Projected chess board corners
	geometry = euler2quaternion( result.first[0],  
								 result.first[1], 
								 result.first[2] );
	geometry.set_Tx( result.first[3] );
	geometry.set_Ty( result.first[4] );
	geometry.set_Tz( result.first[5] );
	geometry.set_Distortion( result.first[6] );
	geometry.set_FocalLenght( result.first[7] );
	geometry.set_PrincipalPointX( result.first[8] );
	geometry.set_PrincipalPointY( result.first[9] );

	// Draw circles on actual corners and predicted corners
	for ( int i = 0; i < corners.size(); i++ )
	{
		Projection pp;
		Point2d ff = pp.Field2Image( Point2d( realPoint[i].x, realPoint[i].y ), geometry );
		Point3d aa = pp.Image2Field( Point3d( ff.x, ff.y, 5 ), geometry, 0 ); // A simple instance for transform image point to world point

		// Predicted corners
		circle(frame, ff, 5, Scalar(45, 0, 200), 2);
		// Actual corners
		circle( frame, corners[i], 5, Scalar( 200, 100, 45 ), 2 );
	}

    imshow("Result", frame);
	waitKey(0);

	return 0;
}