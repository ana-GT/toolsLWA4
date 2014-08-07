
/**
 * @file dt_test.cpp
 * @brief Test Pedro's code for Distance Transform for a binary image
 */
#include <tabletop_symmetry/dt/dt.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <iostream>

/**
 * @function main
 */
int main(int argc, char **argv) {
  if (argc != 2) {
    fprintf(stderr, "usage: %s input_image \n", argv[0]);
    return 1;
  }

  char *input_name = argv[1];
  char *output_name = argv[2];

  // load input
  cv::Mat input = cv::imread( input_name, CV_LOAD_IMAGE_GRAYSCALE );

  cv::imshow( "Input", input );

  int k;
  while( true ) {
    k = cv::waitKey(30);
    if( k != -1 ) { break; }
  }

  
  cv::Mat output = matDT( input );

  double minVal, maxVal; cv::Point m; cv::Point M;
  minMaxLoc( output, &minVal, &maxVal, &m, &M );

  std::cout << "Min: "<< minVal << " maxval: "<< maxVal<< std::endl;
  
  cv::Mat og;
  output.convertTo( og, CV_8U, 255.0 / maxVal );


  // Show
  cv::imshow( "DT", og );

  while( true ) {
    k = cv::waitKey(30);
    if( k != -1 ) { break; }
  }
  
  return 0;

}
