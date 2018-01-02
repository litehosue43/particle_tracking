/*
Primary accretion detection algorithm.

Data structures for storing PGM images.

Jack Lightholder
lightholder.jack16@gmail.com

Space and Terrestrial Robotic Exploration Laboratory (SpaceTREx)
Arizona State University
*/

#ifndef MG_H_INCLUDED
#define MG_H_INCLUDED

#define MAXSTRINGLENGTH 1024

// NULL not standard on all systems, define is necessary
#ifndef NULL
#define NULL 0
#endif // NULL

typedef struct PGMHeader {
  unsigned char type[2];
  int width;
  int numWidthDigits;
  int height;
  int numHeightDigits;
  int grayscale;
  int numGrayscaleDigits;
} PGMHeader;

typedef struct PGMImage {
  PGMHeader header;
  unsigned char** image;
} PGMImage;

typedef enum PGMHeaderPhase {
  READ_TYPE,
  READ_WIDTH,
  READ_HEIGHT,
  READ_GRAYSCALE,
  READ_DONE
} PGMHeaderPhase;

#endif // MG_H_INCLUDED
