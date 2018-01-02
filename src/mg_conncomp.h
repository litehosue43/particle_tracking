/*
Primary accretion detection algorithm.

Connected component analysis functions.

Jack Lightholder
lightholder.jack16@gmail.com

Space and Terrestrial Robotic Exploration Laboratory (SpaceTREx)
Arizona State University
*/

#ifndef MG_CONNCOMP_H_INCLUDED
#define MG_CONNCOMP_H_INCLUDED

#include "mg_centroid.h"

int validatePGM(PGMImage* image, int *pwidth, int *pheight);
void Tracer(int *cy, int *cx, int *tracingdirection);
void ContourTracing(int cy, int cx, int labelindex, int tracingdirection);
Centroid* ConnectedComponentLabeling(PGMImage* image,int* ccCount, int* k);
double calcClusterDensity(int ccCount, const Centroid* centList);

#endif // MG_CONNCOMP_H_INCLUDED
