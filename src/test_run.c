/*
 * Jack Lightholder
 * lightholder.jack16@gmail.com
 * 9/20/2015
 *
 * Primary accretion detection algorithm.
 * Space and Terrestrial Robotic Exploration Laboratory (SpaceTREx)
 * Arizona State University
 *
 * Change file path, start image, end image and the percentage of data to select before running.
 *
 * Algorithm:
 *
 * 1.) Program thresholds each image at 0-255 to determine optimal thresholding value using 2d correlation between
 *      each thresholded image and the original reference.  Once completed for each image in the set, the mean
 *      optimal thresholding value for the set is used to threshold each image in the set and write them to
 *      *\threshold\ folder.
 *
 * 2.) Connected components is run on optimally thresholded images to identify each particle.  Returns centroid values for each particle.
 *
 * 3.) Centroid values are clustered using K-Means with k=sqrt(number centroids / 2).  The density of particle groups informs the average
 *     particle field density.
 *
 * 4.) Shift between images is determined by returning the mean value of the difference in location between a given centroid in the first and
 *     second frame.  Frame shift calculated in both the x and y plane to return an (x,y) shift pair. Roughly equivalent to particle velocities.
 *
 * 5.) Difference between shifts returned in step 5 are calculated and stored.  Roughly equivalent to particle accelerations.
 *
 * 6.) Data is sorted based on data collected in step 5 (rough acceleration data) and K-Means grouping density collected in step 3.
 *     Sorted based on value = (acceleration * weighted classifier 1)*(k-Means density * weighted classifier 2).  Data queued in value
 *     descending order. To mimic spacecraft downlink limitations the user can choose what percentage of the data to send from a given sample.
 *     Downlink function will return given amount of data, prioritizing data with higher classified values.
 *
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <math.h>
#include "mg_image.h"
#include "mg.h"
#include "mg_threshold.h"
#include "mg_conncomp.h"
#include "mg_kmeans.h"
#include "mg_centroid.h"
#include "mg_downlink.h"

char sourceImageDir[] = "C:\\work\\AOSAT\\data\\camera_data\\";
char destImageDir[]   = "C:\\work\\AOSAT\\data\\threshold\\";
char downlinkDir[]    = "C:\\work\\AOSAT\\data\\downlink\\";

/**
  *@brief Data processing sequence.  Reads image, thresholds, conducts connected component
  *           analysis and determines centroids.
  *
  *INPUTS
  *@param original     : Grayscale image direct from camera
  *@param result       : Original image after thresholding
  *@param thresholdVal : Value to threshold all images in the data set at
  *@param imageIndex   : Image index in the data set
  *@param numImages    : Number of total images in the data set
  *@param distance     : Mean value of each centroid and it's cluster center
  *
  *OUTPUTS
  *@param centroids : List of image centroid coordinates
  *@param ccCount   : Number of connected components
  *
  */
Centroid* ProcessImage(PGMImage* original,
                       PGMImage* result,
                       Centroid* centroids,
                       int thresholdVal,
                       int* ccCount,
                       int imageIndex,
                       int numImages,
                       double* distance)
{
    char readPath[MAXSTRINGLENGTH];
    char writePath[MAXSTRINGLENGTH];

    int k = 0;

    sprintf(readPath, "%s%03d.pgm", sourceImageDir,imageIndex);
    readPGM(readPath,original);
    copyPGM(original,result);
    thresholdImage(original,result,thresholdVal);

    centroids = ConnectedComponentLabeling(result,ccCount, &k);

    if(centroids == NULL)
    {
        printf("Error: Connected Components Labeling did not return centroids.  Quitting program.");
        exit(0);
    }

    kmeans(result,k,centroids,*ccCount);
    *distance = calcClusterDensity(*ccCount, centroids);
    sprintf(writePath, "%s%03d.pgm", destImageDir,imageIndex);
    writePGM(writePath,result);

    return centroids;
}

/**
  * @brief Process to free an allocated centroid array.
  *
  * INPUTS
  * @param c    : The Centroid* array that is to be freed
  * @param cLen : The length of the Centroid* array
  *
  * OUTPUTS
  * none
  *
  * @post The Centroid* array has had all of its distance elements and the main array itself
  *         freed and set to NULL
  *
  */
void freeCentroidArray(Centroid* c, int cLen) {

  int i = 0;
  if(c != NULL) {
    for(i = 0; i < cLen; i++) {
      free(c[i].distances);
      c[i].distances = NULL;
    }
    free(c);
    c = NULL;
  }
}

/**
  *@brief Main science sequence.  Processes each image in the data set, determines acceleration and cluster density.
  *          Calls spacecraft to downlink requested percentage of queued data.
  *
  *INPUTS
  *@param startImg           : First image in the data set
  *@param endImg             : Last image in the data set
  *@param downlinkPercentage : Percentage of the data to be downlinked from the spacecraft
  *
  *OUTPUTS
  *none
  */
int SciAnalysis(int startImg, int endImg, int downlinkPercentage)
{
    char pathImage[MAXSTRINGLENGTH];
    PGMImage workingImage1;
    PGMImage result1;
    PGMImage workingImage2;
    PGMImage result2;

    int i=0, sum=0, numImages=0;
    int thresholdVal=0, index=0;
    int shiftIndex=0, distIndex=0;
    int accIndex=0;
    int centList1Len=0,centList2Len=0;
    int corrMatrix[endImg-startImg+1];
    double mean=0.0, distance=0.0;
    Centroid *centList1=NULL;
    Centroid *centList2=NULL;
    Shift *shiftList = NULL;
    Shift *accList = NULL;
    Shift *shift;
    Shift *shiftPrev = NULL;

    numImages = endImg - startImg + 1;
    double kDistances[numImages];

    // initialize memory to zero
    memset(kDistances, 0, sizeof(double)*numImages);

    printf("Number of images to be processed: %d\n",numImages);

    index = startImg;
    for(i = 0; i < numImages; i++)
    {
        sprintf(pathImage, "%s%03d.pgm", sourceImageDir,index);
        puts(pathImage);
        readPGM(pathImage,&workingImage1);
        corrMatrix[i] = thresholdImageSequence(&workingImage1);
        index++;

        // free memory created from the readPGM call
        freePGMImage(&workingImage1);
    }

    for(i = 0; i < numImages ; i++)
    {
        printf("%d: %d\n",i,corrMatrix[i]);
        sum += corrMatrix[i];
    }

    mean = sum/(double)numImages;
    thresholdVal = (int)mean;
    printf("Mean thresholding value for the given dataset: %d\n",thresholdVal);

    shiftList = malloc((numImages-1)*(sizeof(Shift)));
    accList = malloc((numImages-2)*(sizeof(Shift)));
    shiftPrev = malloc(sizeof(Shift));

    // Modulus logic to reduce number of necessary image reads. Fills opposite image structure on each incremental call
    //  and reverses comparison order to retain cohesion.  Allows for since image read on every iteration.
    if(startImg % 2 == 0)
    {
        centList1 = ProcessImage(&workingImage1,&result1,centList1,thresholdVal,&centList1Len,startImg,numImages,&distance);
    }
    else
    {
        centList2 = ProcessImage(&workingImage2,&result2,centList2,thresholdVal,&centList2Len,startImg,numImages,&distance);
    }
    kDistances[distIndex] = distance;
    distIndex++;


    for(i = startImg+1; i <= endImg; i++)
    {
        //Modulus logic to reduce number of necessary image reads. Fills opposite image structure on each incremental call
        //  and reverses comparison order to retain cohesion.  Allows for since image read on every iteration.
        if(i % 2 == 0)
        {
            centList1 = ProcessImage(&workingImage1,&result1,centList1,thresholdVal,&centList1Len,i,numImages,&distance);
        }
        else
        {
            centList2 = ProcessImage(&workingImage2,&result2,centList2,thresholdVal,&centList2Len,i,numImages,&distance);
        }

        kDistances[distIndex] = distance;
        distIndex++;

        shift = detectShift(centList1,centList1Len,centList2,centList2Len);
        shiftList[shiftIndex].x = shift->x;
        shiftList[shiftIndex].y = shift->y;
        shiftIndex++;

        if(i > startImg+1 && (accIndex < (numImages-2))){
            accList[accIndex].x = shiftPrev->x - shift->x;
            accList[accIndex].y = shiftPrev->y - shift->y;
            accIndex++;
        }
        shiftPrev->x = shift->x;
        shiftPrev->y = shift->y;

        free(shift);
        shift = NULL;

        // After performing the shift detection, free the memory on the image and centroid array we are about to overwrite in
        // the next iteration through this loop.
        if((i+1) % 2 == 0)
        {
          freeCentroidArray(centList1, centList1Len);
          freePGMImage(&workingImage1);
          freePGMImage(&result1);
          centList1 = NULL;
        }
        else
        {
          freeCentroidArray(centList2, centList2Len);
          freePGMImage(&workingImage2);
          freePGMImage(&result2);
          centList2 = NULL;
        }
    }

    //Determine which images to queue for downlink from the spacecraft based on acceleration & K-means distance data.
    downlinkData(downlinkPercentage,accList,kDistances,startImg,numImages);

    // Free the final memory for centroid arrays
    if(centList1 != NULL) {
      freeCentroidArray(centList1, centList1Len);
      centList1 = NULL;
    }
    if(centList2 != NULL) {
      freeCentroidArray(centList2, centList2Len);
      centList2 = NULL;
    }

    // Free the final memory for working and resulting images
    if(workingImage1.image != NULL) {
        freePGMImage(&workingImage1);
        workingImage1.image = NULL;
    }
    if(workingImage2.image != NULL) {
        freePGMImage(&workingImage2);
        workingImage2.image = NULL;
    }
    if(result1.image != NULL) {
        freePGMImage(&result1);
        result1.image = NULL;
    }
    if(result2.image != NULL) {
        freePGMImage(&result2);
        result2.image = NULL;
    }

    // Free the final memory for the shift array
    if(shiftList != NULL) {
        free(shiftList);
        shiftList = NULL;
    }

    // Free the final memory for the acceleration array
    if(accList != NULL) {
      free(accList);
      accList = NULL;
    }

    // Free the final memory for the previous shift pointer
    if(shiftPrev != NULL) {
      free(shiftPrev);
      shiftPrev = NULL;
    }

    return 0;
}

/**
  *@brief Program main().  Currently tests science analysis and downlink queue creation algorithms.
  *
  *INPUTS
  *none
  *
  *OUTPUTS
  *none
  *
  *@post Downlink queue established.  Currently located in \data\downlink\.  Represents all data
  *         which needs to be downlinked from spacecraft from a given science routine.
  */
int main()
{

    printf("\nBeginning the ASP accretion RFS test...\n\n");

    int startImg = 1;
    int endImg = 135;
    int downlinkPercentage = 25;

    SciAnalysis(startImg,endImg,downlinkPercentage);

    return 0;

}
