#ifndef CHESSPATTERNDETECTOR_H
#define CHESSPATTERNDETECTOR_H

#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv/ml.h"
#include "opencv/cv.h"
#include <utility>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

pair<vector<Point2f>, Mat> get_pattern_points( int cameraIndex, int num_h, int num_w );

void sortGridPoints( vector<Point2f> &points, int width, int height );

#endif