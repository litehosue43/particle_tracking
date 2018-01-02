/*
Primary accretion detection algorithm.

K-Means data sorting algorithms. Used for grouping centroid locations.

Jack Lightholder
lightholder.jack16@gmail.com

Space and Terrestrial Robotic Exploration Laboratory (SpaceTREx)
Arizona State University
*/

#ifndef MG_KMEANS_H_INCLUDED
#define MG_KMEANS_H_INCLUDED

#include <stdbool.h>
#include "mg.h"
#include "mg_centroid.h"

void createRandomCent(int numCents);
bool ValueInArray(int val, int *arr, int arrSize);
double calcDist(int x1,int y1,int x2,int y2);
bool compareCents(RandomCentroid *rCents1,RandomCentroid *rCents2, int k);
void adjustRandomCentroid(int numCent,int k,Centroid *cents,RandomCentroid *rCent);
void sortCentroids(int numCent,int k,Centroid *cents);
void kmeans(PGMImage* image,int k,Centroid *cents,int numCents);

#endif // MG_KMEANS_H_INCLUDED
