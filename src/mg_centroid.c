/*
Primary accretion detection algorithm.

Centroid and image pair shift detection functions.

Jack Lightholder
lightholder.jack16@gmail.com

Space and Terrestrial Robotic Exploration Laboratory (SpaceTREx)
Arizona State University
*/

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include "mg_centroid.h"
#include "mg.h"

/**
  *@brief Create memory to store centroid coordinates.
  *
  *INPUTS
  *@param numCents : Number of centroids to create
  *@param k        : Length of the distances array of each centroid
  */
Centroid* createCents(int numCents,int k)
{
    int i=0;
    Centroid *cents = NULL;

    // malloc_createCents() cents, free in test_run.c
    cents = malloc(numCents*(sizeof(Centroid)));

    for(i = 0; i<numCents; i++)
    {
        // malloc_createCents() cents[].distances, free in test_run.c
        cents[i].distances = malloc(sizeof(double)*k);
    }

    return cents;
}

/**
  *@brief Detect shift between two lists of centroid coordinates
  *
  *INPUTS
  *@param centList1    : Centroid coordinate list from first image
  *@param centList1Len : Number of centroid coordinates in the first list
  *@param centList2    : Centroid coordinate list from the second image
  *@param centList2Len : Number of centroids in the second list
  *
  *OUTPUTS
  *@param Shift list containing x and y shift
  */
Shift* detectShift(Centroid *centList1,int centList1Len,Centroid *centList2,int centList2Len)
{
    int i=0, smallCent=0;
    double diffX=0.0, diffY=0.0;
    Shift *shift;

    if(centList1Len <= centList2Len)
        smallCent = centList1Len;
    else
        smallCent = centList2Len;

    //printf("\nSmallest amount of centroids between two images: %d\n",smallCent);

    // Malloc_detectShift Shift* free in test_run.c
    shift = malloc(sizeof(Shift));

    for(i=0; i<smallCent; i++)
    {
        diffX = centList2[i].x - centList1[i].x;
        diffY = centList2[i].y - centList1[i].y;
    }

    shift->x = diffX / smallCent;
    shift->y = diffY / smallCent;

    //printf("\nShift X: %f \nShift Y: %f\n",shift[1].x,shift[1].y);

    return shift;

}

