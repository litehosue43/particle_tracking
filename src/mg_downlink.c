/*
Primary accretion detection algorithm.

Data queuing functions for prioritizing based on likelihood of primary accretion.

Jack Lightholder
lightholder.jack16@gmail.com

Space and Terrestrial Robotic Exploration Laboratory (SpaceTREx)
Arizona State University
*/

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <stdbool.h>
#include "mg_centroid.h"
#include "mg.h"
#include "mg_image.h"

extern char sourceImageDir[];
extern char downlinkDir[];

/**
  *@brief Downlink image by moving from *\data\camera_data\* to *\data\downlink\* folder.
  *
  *INPUTS
  *@param image      : Image structure to be copied.
  *@param path       : Original path of the image to be copied.
  *@param index      : Index of image in the data set.
  *@param downlinked : Array containing info on which images have been downlinked.
  *@param score      : Score of each image.  Influences downlink order.
  *@param startImg   : Number of the first image in the data set.
  *
  *OUTPUTS
  *@param downlinkCount : Number of images currently downlinked from the data set.
  */
void downlinkImage(PGMImage* image, char* path,int index,int* downlinkCount,bool downlinked[],double score[],int startImg){

    sprintf(path, "%s%03d.pgm", sourceImageDir,startImg+index);
    readPGM(path,image);
    sprintf(path, "%s%03d.pgm", downlinkDir,startImg+index);
    writePGM(path,image);
    downlinked[index] = true;
    (*downlinkCount)++;
    score[index] = 0.0;
    freePGMImage(image);
}

/**
  *@brief Select images for file transfer (representative spacecraft downlink)
  *        based on cluster distance and frame acceleration.
  *
  *INPUTS
  *@param downlinkPercentage : Percentage (0-100) of the data set to be transfered.
  *@param acceleration       : Array containing acceleration data
  *@param kDistances         : K-means cluster mean point to center distance.
  *@param startImg           : Value of the first image in the data set.
  *@param numImages          : Value containing the total number of images in the data set.
  *
  *OUTPUTS
  *none
  */
void downlinkData(int downlinkPercentage,Shift* acceleration,double* kDistances,int startImg,int numImages){

    int i=0,index=0,maxTries=0;
    int downlinkCount=0, images2Downlink=0;
    int endImg = (startImg + numImages - 1);
    double score[numImages];
    double maxScore=0.0;
    char path[MAXSTRINGLENGTH];
    bool downlinked[numImages];
    PGMImage image;

    for(i = 0; i < numImages; i++) {
      downlinked[i] = false;
    }

    //Classifiers.  Change weight based on training data.
    double c1=0.5,c2=0.5;

    images2Downlink = (numImages * (downlinkPercentage * .01));

    //Print the first image.
    sprintf(path, "%s%03d.pgm", sourceImageDir,startImg);
    readPGM(path,&image);
    sprintf(path, "%s%03d.pgm", downlinkDir,startImg);
    writePGM(path,&image);
    downlinked[0] = true;
    score[0] = 0.0;
    downlinkCount++;
    freePGMImage(&image);

    //Print the last image.
    sprintf(path, "%s%03d.pgm", sourceImageDir,endImg);
    readPGM(path,&image);
    sprintf(path, "%s%03d.pgm", downlinkDir,endImg);
    writePGM(path,&image);
    downlinked[numImages-1] = true;
    score[numImages-1] = 0.0;
    downlinkCount++;
    freePGMImage(&image);

    // Score each image pair based on trained classifiers
    // Skip the first and last indices because those represent the first and
    // last image which were already downlinked above.
    for(i=1; i<(numImages-1); i++){
        score[i] = (kDistances[i]*c1)+((acceleration[i].x + acceleration[i].y)*c2);
        printf("Score %d     : %0.5f\n", i, score[i]);
        printf("kDistances   : %0.5f\n", kDistances[i]);
        printf("acceleration : (%0.5f,%0.5f)\n", acceleration[i].x, acceleration[i].y);
    }

    while((downlinkCount < images2Downlink) && (maxTries < 1000)){

        maxScore = 0.0;
        for(i=1; i<(numImages-1); i++){
            if(score[i] > maxScore){
                maxScore = score[i];
                index = i;
            }
        }
        maxTries++;

        if(downlinked[index-1] == false){
            downlinkImage(&image,path,index-1,&downlinkCount,downlinked,score,startImg);
        }

        if(downlinked[index] == false){
            downlinkImage(&image,path,index,&downlinkCount,downlinked,score,startImg);
        }

        if(downlinked[index+1] == false){
            downlinkImage(&image,path,index+1,&downlinkCount,downlinked,score,startImg);
        }
        printf("\ndownlinkCount : %d\n",downlinkCount);
    }

    /*
    for(i = 0; i < images2Downlink; i++) {
      printf("Img %d downlinked: %d\n", i, downlinked[i]);
    }
    */

    maxTries = 0;
}

