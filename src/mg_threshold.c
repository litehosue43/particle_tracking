/*
Primary accretion detection algorithm.

PGM thresholding functions.

Jack Lightholder
lightholder.jack16@gmail.com

Space and Terrestrial Robotic Exploration Laboratory (SpaceTREx)
Arizona State University
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mg.h"
#include "mg_threshold.h"
#include "mg_image.h"

/**
  *@brief Threshold a given image at a given threshold value
  *
  *INPUTS
  *@param image        : Image to be thresholded
  *@param thresholdVal : Value to threshold image at
  *
  *OUTPUTS
  *@param result : Resulting black and white image
  *
  */
void thresholdImage(PGMImage* image,PGMImage* result,int thresholdVal){

    int height=0, width=0, i=0, j=0, intPix=0;
    unsigned char tmpPix;

    if(image == NULL || result == NULL){
        printf("Error:  Null pointer exception.  Mg_threshold : thresholdImage");
        exit(0);
    }

    if(image->image == NULL || result->image == NULL){
        printf("Error:  Null pointer exception.  Mg_threshold : thresholdImage");
        exit(0);
    }

    height = image->header.height;
    width = image->header.width;

    for(i=0; i<height; i++){
      for(j=0; j<width; j++){
        tmpPix = (image->image[i][j]);
        intPix = tmpPix & 0xFF;
        //printf("\nintPix: %d\n",intPix);

        if(intPix > thresholdVal)
            result->image[i][j] = WHITEPIX;
        else
            result->image[i][j] = BLACKPIX;
        }
    }

    result->header.grayscale = 1;
}

/**
  *@brief Threshold a given image at every value between 0 and 255.  Use 2D correlation
  *          to determine correlation value between every resulting threshold image and original.
  *          Return threshold value of image with highest correlation.
  *
  *INPUTS
  *@param image : Image to be thresholded.
  *
  *OUTPUTS
  *@param Thresholding value with highest correlation to original image.
  */
int thresholdImageSequence(PGMImage* image){

  int i=0, index=0;
  double r=0.0, r_max=0.0;
  PGMImage result;

  copyPGM(image,&result);

    for(i = 0; i<=255; i++){
      thresholdImage(image,&result,i);
      r = corr2d(image,&result);
      //printf("threshold %d is %0.2f\n", i, r);

      if(r > r_max){
          r_max = r;
          index = i;
      }
    }
    //printf("Max correlation: %f\n",r_max);
    //printf("Optimal threshold value: %d\n",index);

    return index;
}
