/*
Primary accretion detection algorithm.

PGM thresholding functions.

Jack Lightholder
lightholder.jack16@gmail.com

Space and Terrestrial Robotic Exploration Laboratory (SpaceTREx)
Arizona State University
*/

#ifndef MG_THRESHOLD_H_INCLUDED
#define MG_THRESHOLD_H_INCLUDED

void thresholdImage(PGMImage* image,PGMImage* result, int threshold_val);
int thresholdImageSequence(PGMImage* image);

#endif // MG_THRESHOLD_H_INCLUDED
