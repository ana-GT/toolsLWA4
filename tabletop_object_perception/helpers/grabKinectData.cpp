/**
 * @file grabKinectData
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <string>

#include <iostream>

/** Global variables */
char rgbWindow[50] = "RGB Image";
char depthMapWindow[50] = "Depth Map";

/** Function headers */
void printDeviceInfo( cv::VideoCapture &_capture );
void parseCommandLine( int _argc,
		       char*_argv[],
		       std::string &_filename,
		       bool &_isFileReading );
void printCommandLineParams();


/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  /** Variables to indicate video file reading  (no capturing) */
  std::string filename;
  bool isVideoReading;

  /** Parse input directives */
  parseCommandLine( argc, argv, filename, isVideoReading );

  /** Open video sources */
  std::cout << "** Device is opening **" << std::endl;
  cv::VideoCapture capture;

  /** If reading video file */
  if( isVideoReading ) {
    capture.open( filename );
  }
  /** Otherwise, get video stream from Kinect */
  else {
    capture.open( CV_CAP_OPENNI );
  }

  /** Check device opened correctly */
  if( !capture.isOpened() ) {
    std::cout << "\t [ERROR] Device / file was not opened." << std::endl;
    return -1;
  } else {
    std::cout << "\t [OK] Device / file was opened correctly." << std::endl;
  }

  /** Start capture if that is the case */
  if( !isVideoReading ) {
    capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );
  }

  /** Print device info */
  printDeviceInfo( capture );

  /** Loop */
  for(;;) {
    
    cv::Mat depthMap;
    cv::Mat bgrImage;
    cv::Mat depthMapShow;

    // Grab a frame
    if( !capture.grab() ) {
      std::cout << " [ERROR] Could not grab images. BANG" << std::endl;
      return -1;
    }

    else {
      
      /** Get depth image */
      if( capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP) ) {
	const float scaleFactor = 0.05f;
	depthMap.convertTo( depthMapShow, CV_8UC1, scaleFactor );
	cv::imshow( depthMapWindow, depthMapShow );
      }

      /** Get BGR Image */
      if( capture.retrieve( bgrImage, CV_CAP_OPENNI_BGR_IMAGE ) ) {
	cv::imshow( rgbWindow, bgrImage );
      }
      
    } // end else grab()

    if( cv::waitKey(30) >= 0 ) {
      break;
    }

  } // end for loop

  return 0;

}

/**
 * @function printDeviceInfo
 */
void printDeviceInfo( cv::VideoCapture &_capture ) {

  // Print depth info
  std::cout << "** Depth generator output mode **" << std::endl;
  std::cout << "\t * Frame height: "<< (int) _capture.get(CV_CAP_PROP_FRAME_HEIGHT) << " pixels." << std::endl;
  std::cout << "\t * Frame width: "<< (int) _capture.get(CV_CAP_PROP_FRAME_WIDTH) << " pixels." << std::endl;  
  std::cout << "\t * Frame max depth: "<< (int) _capture.get(CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH) << " mm." << std::endl;
  std::cout << "\t * Frames per second: "<< (int) _capture.get(CV_CAP_PROP_FPS) << " frames/second." << std::endl; 		
  std::cout << "\t * Registration: "<< (int) _capture.get(CV_CAP_PROP_OPENNI_REGISTRATION)  << std::endl;

  // Print image info if available
  if( _capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR_PRESENT ) ) {
    std::cout << "** Image generator output mode" << std::endl;
    std::cout << "\t * Frame width: "<< (int)_capture.get(CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_WIDTH)<<"pixels"<<std::endl;
    std::cout << "\t * Frame height: "<< (int)_capture.get(CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_HEIGHT)<<"pixels"<<std::endl;
    std::cout << "\t * Frame per second: "<< (int)_capture.get(CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FPS)<<"frames/second"<<std::endl;
  }

}

/**
 * @function parseCommandLine
 */
void parseCommandLine( int _argc,
		       char*_argv[],
		       std::string &_filename,
		       bool &_isFileReading ) {

  
  // Set default values
  
  _filename.clear();
  _isFileReading = false;
  
  if( _argc == 1 ) {
    // Default
  } else {
    for( int i = 1; i < _argc; ++i ) {
      if( !strcmp( _argv[i], "--help" ) || !strcmp( _argv[i], "-h") ) {
	printCommandLineParams();
	exit(0);
      }
      else if( !strcmp( _argv[i], "-r" ) ) {
	_filename = _argv[++i];
	_isFileReading = true;
      }
      else {
	printf( "Unsupported command line argument: %s \n", _argv[i] );
	exit(-1);
      }
      
    } // end for 
  } // end else
}


/**
 * @function printCommandLineParams
 */
void printCommandLineParams() {

  printf( "--help: Usage help \n");
  printf( " -r: Filename of video file \n" );

}

