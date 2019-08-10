#include "ChessPatternDetector.h"

pair<vector<Point2f>, Mat> get_pattern_points( int cameraIndex, int num_h, int num_w )
{
  VideoCapture cap(cameraIndex);
  
  while(true)
  {
    Mat frame;
    cap >> frame;

    static vector<Point2f> cornerPoints;
    cornerPoints.clear();
    bool patternfound = false; 
    patternfound = findChessboardCorners( frame, Size(num_h, num_w), cornerPoints );

    if( patternfound )
    {
      cout << " **Pattern Found** " << endl;
      sortGridPoints( cornerPoints, num_h, num_w );
      return make_pair( cornerPoints, frame );
    }else
    {
      cout << " **Pattern NOT Found** " << endl;
    }
    

  }
}

void sortGridPoints( vector<Point2f> &points,int width, int height )
{
  for( int i = 1; i < points.size(); ++i) 
  {
    for( int j = 0; j < ( points.size()-i ); ++j )
      if( points[j].y > points[j+1].y )
      {
        Point2f temp=points[j];
        points[j]=points[j+1];
        points[j+1]=temp;	  
      }     
  }
  
  for( int i = 0; i < height ; i++ )
  {
    for( int j = 1; j < width; j++ )
    {
      for( int k = 0; k < width-j; k++ )
      {
        int index = i * width + k;
        if( points[index].x > points[index+1].x )
        {
          Point2f temp = points[index];
          points[index] = points[index+1];
          points[index+1] = temp;	 
        }
      }
    }
  }
}