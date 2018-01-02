/*
Primary accretion detection algorithm.

K-Means data sorting algorithms. Used for grouping centroid locations.

Jack Lightholder
lightholder.jack16@gmail.com

Space and Terrestrial Robotic Exploration Laboratory (SpaceTREx)
Arizona State University
*/

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <time.h>
#include <math.h>
#include <stdbool.h>
#include "mg.h"
#include "mg_kmeans.h"
#include "mg_centroid.h"

/**
  *@brief Check to determine if given value exists in a given array.
  *
  *INPUTS
  *@param val     : Value to check.
  *@param arr     : Array which needs to be searched.
  *@param arrSize : Size of the array to be searched.
  *
  *OUTPUTS
  *@param True if val is in arr, False otherwise
  */
bool ValueInArray(int val, int *arr, int arrSize){

    int i=0;

    for (i=0; i < arrSize; i++) {
        if (arr[i] == val)
            return true;
    }
    return false;
}


/**
  *@brief Determine distance between two (x,y) coordinate pairs.
  *
  *INPUTS
  *@param x1 : x value for the first coordinate pair.
  *@param y1 : y value for the first coordinate pair.
  *@param x2 : x value for the second coordinate pair.
  *@param y2 : y value for the second coordinate pair.
  *
  *OUTPUTS
  *@param Return distance between two points.
  */
double calcDist(int x1,int y1,int x2,int y2){

    float distance;
    int xDist,yDist,sum;
    //printf("Values: %d %d %d %d\n",x1,y1,x2,y2);
    xDist = x2 - x1;
    yDist = y2 - y1;
    sum = abs((xDist*xDist)+(yDist*yDist));
    distance = sqrt((float)sum);

    return distance;
}

/**
  *@brief Determine if two structures containing cluster center coordinates are equivalent.
  *       Returning true indicates K-Means updating has converged.
  *
  *@param rCents1 : RandomCentroid structure containing cluster center coordinates.
  *@param rCents2 : RandomCentroid structure containing cluster center coordinates.
  *@param k       : Number of clusters to be compared.
  *
  */
bool compareCents(RandomCentroid *rCents1,RandomCentroid *rCents2, int k){

    bool equivilent = false;
    int i=0, j=0, count = 0;

    for(i = 0; i<k; i++){
        for(j=0; j<k; j++){
            if(rCents1[i].x == rCents2[j].x  && rCents1[i].y == rCents2[j].y)
                count += 1;
        }
    }

    if(count == k)
        equivilent = true;
    else
        equivilent = false;

    //fputs(equivilent ? "true\n" : "false\n", stdout);
    return equivilent;

}

/**
  *@brief Sort centroids into clusters based on distances from
  *         cluster centers.  Sorted to closest cluster.
  *
  *INPUTS
  *@param numCent : Number of centroids.
  *@param k       : Number of clusters.
  *@param cents   : Array containing centroid data.
  *
  *OUTPUTS
  *none
  */
void sortCentroids(int numCent,int k,Centroid *cents){

    int minIndex=0, a=0, b=0;

    for(a=0; a<numCent; a++){
        for(b=0; b<k; b++){
            if(cents[a].distances[b] < cents[a].distances[minIndex])
                minIndex = b;
        }
        cents[a].kGroup = minIndex;
    }
}

/**
  *@brief Recenter centroid cluster based on coordinates of each centroid in the cluster
  *
  *@param numCents  : Number of centroids.
  *@param k         : Number of clusters.
  *@param cents     : Centroid structure containing the centroid coordinates.
  *
  *OUTPUTS
  *@param rCent : Centroid structure containing the centroid cluster coordinates.
  *
  */
void adjustRandomCentroid(int numCents,int k,Centroid *cents,RandomCentroid *rCent){

    int i=0, xSum=0, ySum=0, curK=0;
    int xMean=0, yMean=0;

    for(curK = 0; curK<k; curK++){
        for(i = 0; i<numCents; i++){
            if(cents[i].kGroup == curK)
                xSum += cents[i].x;
                ySum += cents[i].y;
        }
            xMean = xSum / numCents;
            yMean = ySum / numCents;
            //printf("New X,Y Coordinate: %d %d\n",xMean,yMean);
            rCent[curK].x = xMean;
            rCent[curK].y = yMean;
    }
}

/**
  *@brief K-Means sorting based on k = sqrt(number of centroids / 2)
  *         Sorts centroids into k clusters based on their distance from the cluster center.
  *
  *INPUTS
  *@param image    : PGMIMage structure image to be analyzed.
  *@param k        : Number of clusters.
  *@param numCents : Number of centroids.
  *
  *OUTPUTS
  *none
  */
void kmeans(PGMImage* image,int k,Centroid *cents,int numCents){

    int i=0, j=0, random=0, loopCount=0;
    srand ( time(NULL) );
    int randCentInd[k];
    bool answer;
    bool equivilent = false;
    RandomCentroid *rCent;
    RandomCentroid *tmpCent;

    rCent = malloc(k*sizeof(RandomCentroid));
    tmpCent = malloc(k*sizeof(RandomCentroid));

    for(i=0; i<k; i++){
        answer = true;
        while(answer == true){
            random = rand() % (k + 1);
            answer = ValueInArray(random,randCentInd,k);
        }
        randCentInd[i] = random;
        //printf("random number: %d\n",randCentInd[i]);
    }

    for(i=0; i<k; i++){
        rCent[i].x = cents[randCentInd[i]].x;
        rCent[i].y = cents[randCentInd[i]].y;
        //printf("Random Centroid Values %d: %d %d\n",j,rCent[i].x,rCent[i].y);
    }

    for(i=0; i<k; i++){
        for(j=0; j<numCents; j++){
            cents[j].distances[i] = calcDist(rCent[i].x,rCent[i].y,cents[j].x,cents[j].y);
            //printf("Current Cent Distance: %f\n",cents[j].distances[i]);
        }
    }

    sortCentroids(numCents,k,cents);

    for(i=0; i<k; i++){
        tmpCent[i].x = rCent[i].x;
        tmpCent[i].y = rCent[i].y;
    }

    adjustRandomCentroid(numCents,k,cents,rCent);

    equivilent = compareCents(tmpCent,rCent,k);

    while(equivilent == false){
        sortCentroids(numCents,k,cents);
        for(i=0; i<k; i++){
            tmpCent[i].x = rCent[i].x;
            tmpCent[i].y = rCent[i].y;
        }
        adjustRandomCentroid(numCents,k,cents,rCent);
        equivilent = compareCents(tmpCent,rCent,k);
        loopCount += 1;

        // If cluster centroid has not converged in 500 attempts,
        // use current cluster centroid
        if(loopCount >= 500)
            equivilent = true;
    }

    printf("Number of clusters: %d\n",k);
    for(i=0; i<k; i++){
        printf("Cluster %d (X,Y) center: %d %d\n",i,rCent[i].x,rCent[i].y);
    }

    free(tmpCent);
    tmpCent = NULL;
    free(rCent);
    rCent = NULL;
}
