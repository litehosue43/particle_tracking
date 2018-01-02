/*
Primary accretion detection algorithm.

Centroid and image pair shift detection functions.

Jack Lightholder
lightholder.jack16@gmail.com

Space and Terrestrial Robotic Exploration Laboratory (SpaceTREx)
Arizona State University
*/

#ifndef CENTROID_H_INCLUDED
#define CENTROID_H_INCLUDED

typedef struct Centroid {
    int x;
    int y;
    int kGroup;
    double *distances;
}Centroid;

typedef struct RandomCentroid{
    int x;
    int y;
}RandomCentroid;

typedef struct Shift{
    double x;
    double y;
}Shift;

Centroid* createCents(int numCents,int k);
Shift* detectShift(Centroid *cents1,int numCent1,Centroid *cents2,int numCent2);

#endif // CENTROID_H_INCLUDED
