/*
Primary accretion detection algorithm.

PGM read/write functionality.

Jack Lightholder
lightholder.jack16@gmail.com

Space and Terrestrial Robotic Exploration Laboratory (SpaceTREx)
Arizona State University
*/

#ifndef MG_IMAGE_H_INCLUDED
#define MG_IMAGE_H_INCLUDED

#include "mg.h"

#define BLACKPIX 1
#define WHITEPIX 0

int bytesToNextSpace(FILE* file);
void parsePGMHeader(PGMHeader* header, FILE* file);
double corr2d(PGMImage* image1,PGMImage* image2);
void readPGM(char* filename,PGMImage* image);
void writePGM(char* filename,PGMImage* image);
void copyPGM(PGMImage* imageSource, PGMImage* imageDest);
void allocatePGMImageArray(PGMImage* pgm);
void deallocatePGMImageArray(PGMImage* pgm);
void freePGMImage(PGMImage* img);

#endif // MG_IMAGE_H_INCLUDED
