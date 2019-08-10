#ifndef PSO_H
#define PSO_H

#include <algorithm>
#include <functional>
#include <vector>
#include <random>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class PSO
{

private:

	int                          numberOfParticles;
	int                          numberOfDimensions;
	int                          maximumOfIteration; // Termination Condition By Iteration

	double                       errorCon; // Termination Condition By Error
	double                       W; // Inertial Coefficient
	double                       C1; // Acceleration Coefficient
	double                       C2; // Acceleration Coefficient

	pair<double, double> randRx;
	pair<double, double> randRy;
	pair<double, double> randRz;
	pair<double, double> randTx;
	pair<double, double> randTy;
	pair<double, double> randTz;
	pair<double, double> randDist;
	pair<double, double> randFocal;
	pair<double, double> randPrincX;
	pair<double, double> randPrincY;

	pair<vector<double>, double> result;

public:

	auto set_numberOfParticles( int    NOP ) { this->numberOfParticles = NOP; };

	auto set_numberOfDimension( int    NOD ) { this->numberOfDimensions = NOD; };

	auto set_maximumOfIteration( int    MOI ) { this->maximumOfIteration = MOI; };

	auto set_rand_rx( double min, double max ) { this->randRx = make_pair( min, max ); };
	auto set_rand_ry( double min, double max ) { this->randRy = make_pair( min, max ); };
	auto set_rand_rz( double min, double max ) { this->randRz = make_pair( min, max ); };

	auto set_rand_tx( double min, double max ) { this->randTx = make_pair( min, max ); };
	auto set_rand_ty( double min, double max ) { this->randTy = make_pair( min, max ); };
	auto set_rand_tz( double min, double max ) { this->randTz = make_pair( min, max ); };

	auto set_rand_distortion( double min, double max ) { this->randDist = make_pair( min, max ); };
	auto set_rand_focalLength( double min, double max ) { this->randFocal = make_pair( min, max ); };
	auto set_rand_principalPointX( double min, double max ) { this->randPrincX = make_pair( min, max ); };
	auto set_rand_principalPointY( double min, double max ) { this->randPrincY = make_pair( min, max ); };

	auto set_errorCon( double ERC ) { this->errorCon = ERC; };

	auto set_w( double w ) { this->W = w; };

	auto set_c1( double c1 ) { this->C1 = c1; };

	auto set_c2( double c2 ) { this->C2 = c2; };

	auto optimize( function<double( vector<double> )> fitFunc )
	{

		mt19937 mersenne( static_cast<unsigned int>( time( nullptr ) ) );

		uniform_real_distribution<> rndRx( randRx.first, randRx.second );
		uniform_real_distribution<> rndRy( randRy.first, randRy.second );
		uniform_real_distribution<> rndRz( randRz.first, randRz.second );
		uniform_real_distribution<> rndTx( randTx.first, randTx.second );
		uniform_real_distribution<> rndTy( randTy.first, randTy.second );
		uniform_real_distribution<> rndTz( randTz.first, randTz.second );
		uniform_real_distribution<> rndDist( randDist.first, randDist.second );
		uniform_real_distribution<> rndFocal( randFocal.first, randFocal.second );
		uniform_real_distribution<> rndPrincX( randPrincX.first, randPrincX.second );
		uniform_real_distribution<> rndPrincY( randPrincY.first, randPrincY.second );

		uniform_real_distribution<> rnd2( 0, 1 );

		vector<double> localBest( numberOfDimensions );
		localBest[0] = rndRx( mersenne );
		localBest[1] = rndRy( mersenne );
		localBest[2] = rndRz( mersenne );
		localBest[3] = rndTx( mersenne );
		localBest[4] = rndTy( mersenne );
		localBest[5] = rndTz( mersenne );
		localBest[6] = rndDist( mersenne );
		localBest[7] = rndFocal( mersenne );
		localBest[8] = rndPrincX( mersenne );
		localBest[9] = rndPrincY( mersenne );

		vector<double> globalBest( numberOfDimensions );
		globalBest = localBest;

		vector <vector<double> > particles;
		particles.resize( numberOfParticles, vector<double>( numberOfDimensions, 0 ) );

		vector<double> velocity;
		velocity.resize( numberOfDimensions, 0 );

		for (int i = 0; i < particles.size(); i++)
		{
			particles[i][0] = rndRx( mersenne );
			particles[i][1] = rndRy( mersenne );
			particles[i][2] = rndRz( mersenne );
			particles[i][3] = rndTx( mersenne );
			particles[i][4] = rndTy( mersenne );
			particles[i][5] = rndTz( mersenne );
			particles[i][6] = rndDist( mersenne );
			particles[i][7] = rndFocal( mersenne );
			particles[i][8] = rndPrincX( mersenne );
			particles[i][9] = rndPrincY( mersenne );

			localBest.clear();
			localBest = particles[i];

			if ( fitFunc( localBest ) < fitFunc( globalBest ) )
			{
				globalBest.clear();
				globalBest = localBest;
			}

			localBest.clear();
			localBest = globalBest;
		}

		int iterator = 0;
		while ( fitFunc( globalBest ) > errorCon and iterator < maximumOfIteration )
		{
			++iterator;
			for ( int i = 0; i < numberOfParticles; ++i )
			{
				for ( int j = 0; j < numberOfDimensions; ++j )
				{
					double r1 = rnd2( mersenne );
					double r2 = rnd2( mersenne );

					velocity[j] = W * velocity[j]
						+ ( C1 * r1 * ( localBest[j] - particles[i][j] ) )
						+ ( C2 * r2 * ( globalBest[j] - particles[i][j] ) );

					particles[i][j] = particles[i][j] + velocity[j];
				}

				if ( fitFunc( particles[i]) < fitFunc( localBest ) )
				{
					localBest.clear();
					localBest = particles[i];
				}
			}

			if ( fitFunc( localBest ) < fitFunc( globalBest ) )
			{
				globalBest.clear();
				globalBest = localBest;
			}

		}

		this->result = make_pair( globalBest, fitFunc( globalBest ) );
		return make_pair( globalBest, fitFunc( globalBest ) );
	}

};

#endif