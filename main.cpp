/*
 * main.cpp
 *
 *  Created on: May 9, 2018
 *      Author: Dana
 */
/*********** Includes ***********/
#include <iostream>
#include <string>
#include "PathCalculator.h"
#include "Defs.h"
/*
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
*/
#include <iostream>
#include <string>

//#include "opencv-3.1.0\opencv\sources\3rdparty\libtiff\tiff.h"

//using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
//	TIFF* tif;
	float** map;
    int rows;
    int cols;
    int * coordinates;
    int radius;
    int velocity;

    // Check the number of parameters
    if (argc < MAIN_NUM_OF_ARGS) {
        // Tell the user how to run the program
    	string arg_string = "tif_file man_coordinates_x man_coordinates_y initial_radius helicopter_velocity";
        std::cerr << "Usage: " << argv[0] << arg_string << std::endl;
        return RETURN_CODE_ERROR;
    }

    // Parse Arguments
    // Open the TIFF file using libtiff
//    tif = TIFFOpen(argv[0].c_str(), "r");
    coordinates = new int[2];
    coordinates[0] = atoi(argv[1]);
    coordinates[1] = atoi(argv[2]);
    radius = atoi(argv[3]);
    velocity = atoi(argv[4]);

    // Create object for the PathCalculator
    PathCalculator path(rows, cols, map, coordinates, radius, velocity);
}


