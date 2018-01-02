/*
Primary accretion detection algorithm.

PGM read/write functionality.

Jack Lightholder
lightholder.jack16@gmail.com

Space and Terrestrial Robotic Exploration Laboratory (SpaceTREx)
Arizona State University
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mg_image.h"
#include "mg_threshold.h"
#include "mg.h"

/**
  *@brief Dynamically determine number of bytes to read while parsing PGM file.
  *       LIMITATION: Currently assume all whitespace in header is a single space
  *                   this is actually not per the PGM spec and needs to be improved.
  *
  *INPUTS
  *@param file : File to be parsed
  *
  *OUTPUTS
  *@param Number of bytes to read
  *
  */
int bytesToNextSpace(FILE* file) {

  unsigned char oneByte;
  int startPos = ftell(file);
  int bytes = 0;

  fread(&oneByte, sizeof(unsigned char), 1, file);
  while(oneByte != ' ' && oneByte != '\n') {
    //printf("%x ", oneByte);
    fread(&oneByte, sizeof(unsigned char), 1, file);
    bytes++;
  }
  // return the file pointer to its original place
  fseek(file, startPos, SEEK_SET);
  return bytes;
}

/**
  *@brief Parse PGM header functionality.  Utilized for populating PGMImage structure header.
  *
  *INPUTS
  *@param header : Header structure to be populated
  *@param file   : File to be read
  *
  *OUTPUTS
  *none
  */
void parsePGMHeader(PGMHeader* header, FILE* file) {

  unsigned char oneByte;
  unsigned char* tempBuffer;
  int bytesToRead=0;

  PGMHeaderPhase phase = READ_TYPE;
  do {
    switch(phase) {
    case READ_TYPE:
      // To read the type, read two bytes
      fread(header->type, sizeof(unsigned char), 2, file);
      // Read the space to move to the next section
      fread(&oneByte, 1, 1, file);
      //printf("%x ", oneByte);
      phase = READ_WIDTH;
      break;
    case READ_WIDTH:
      // Find the next space in the header and then read from the current file pointer
      // to that space
      bytesToRead = bytesToNextSpace(file);
      // malloc_tempBuffer free in mg_image.c
      tempBuffer = malloc(sizeof(unsigned char)*bytesToRead+1);
      fread(tempBuffer, sizeof(unsigned char), bytesToRead, file);
      header->width = atoi((char*)tempBuffer);
      header->numWidthDigits = bytesToRead;
      printf("Width is %d\n", header->width);
      free(tempBuffer);
      tempBuffer = NULL;
      // Read the space to move to the next section
      fread(&oneByte, 1, 1, file);
      phase = READ_HEIGHT;
      break;
    case READ_HEIGHT:
      bytesToRead = bytesToNextSpace(file);
      // malloc_tempBuffer free in mg_image.c
      tempBuffer = malloc(sizeof(unsigned char)*bytesToRead+1);
      fread(tempBuffer, sizeof(unsigned char), bytesToRead, file);
      header->height = atoi((char*)tempBuffer);
      header->numHeightDigits = bytesToRead;
      // Read the space to move to the next section
      fread(&oneByte, 1, 1, file);
      printf("Height is %d\n", header->height);
      free(tempBuffer);
      tempBuffer = NULL;
      phase = READ_GRAYSCALE;
      break;
    case READ_GRAYSCALE:
      bytesToRead = bytesToNextSpace(file);
      // malloc_tempBuffer free in mg_image.c
      tempBuffer = malloc(sizeof(unsigned char)*bytesToRead+1);
      fread(tempBuffer, sizeof(unsigned char), bytesToRead, file);
      header->grayscale = atoi((char*)tempBuffer);
      header->numGrayscaleDigits = bytesToRead;
      printf("Grayscale is %d\n\n", header->grayscale);
      free(tempBuffer);
      tempBuffer = NULL;
      phase = READ_DONE;
      break;
    case READ_DONE:
      fread(&oneByte, sizeof(unsigned char), 1, file);
      break;
    }

  } while(oneByte != '\n');
}

 /**
   *@brief PGM read functionality.
   *
   *INPUTS
   *@param filename :  Read path for the file.
   *@param image    :  Structure to store read data.
   *
   *OUTPUTS
   *none
   */
void readPGM(char* filename,PGMImage* image){
  int i=0;
  FILE* file = NULL;
  file = fopen(filename, "rb");

  if(file != NULL) {
    printf("Opened file %s\n", filename);
    parsePGMHeader(&(image->header), file);
    // After the header is parsed memory can be allocated for the image

    // malloc_readPGM image->image free in test_run.c
    image->image = malloc(sizeof(unsigned char*)*image->header.height);
    for(i = 0; i < image->header.height; i++) {
      // malloc_readPGM image->image[] free in test_run.c
      image->image[i] = malloc(sizeof(unsigned char)*image->header.width);
    }

    for(i = 0; i < image->header.height; i++) {
        fread(image->image[i], sizeof(unsigned char), image->header.width, file);
    }

    fclose(file);
  }
  else {
    printf("Error opening file for read: %s\n",filename);
    exit(0);
  }
}

/**
  *@brief PGM write functionality.
  *
  *INPUTS
  *@param filename : Write path for the file.
  *@param image    : PGMImage containing the image to be written.
  *
  *OUTPUTS
  *none
  */
void writePGM(char* filename,PGMImage* image){

  int i=0;
  FILE* file = NULL;
  unsigned char* tempBuffer = NULL;

  file = fopen(filename, "wb");
  if(file != NULL) {

    if(image == NULL){
        printf("Error: Null pointer exception mg_image : writePGM");
    }
    //printf("Printing image\n");

    // Write the type and a space
    fwrite(image->header.type,sizeof(unsigned char),2,file);
    fwrite(" ",sizeof(char),1,file);

    // Write the width and a space
    tempBuffer = malloc(sizeof(unsigned char)*(image->header.numWidthDigits+1));
    itoa(image->header.width, (char*)tempBuffer, 10);
    fwrite(tempBuffer, sizeof(unsigned char), image->header.numWidthDigits, file);
    fwrite(" ",sizeof(char),1,file);
    free(tempBuffer);
    tempBuffer = NULL;

    // Write the height and a space
    tempBuffer = malloc(sizeof(unsigned char)*(image->header.numHeightDigits+1));
    itoa(image->header.height, (char*)tempBuffer, 10);
    fwrite(tempBuffer, sizeof(unsigned char), image->header.numHeightDigits, file);
    fwrite(" ",sizeof(char),1,file);
    free(tempBuffer);
    tempBuffer = NULL;

    // Write grayscale image data
    tempBuffer = malloc(sizeof(unsigned char)*(image->header.numGrayscaleDigits+1));
    itoa(image->header.grayscale, (char*)tempBuffer, 10);
    fwrite(tempBuffer, sizeof(unsigned char), image->header.numGrayscaleDigits, file);
    fwrite("\n",sizeof(char),1,file);
    free(tempBuffer);
    tempBuffer = NULL;

    for(i = 0; i < image->header.height; i++) {
        fwrite(image->image[i], sizeof(char), image->header.width, file);
    }

    fclose(file);
  }
  else {
    printf("Error opening file for write: %s\n",filename);
    exit(0);
  }
}

/**
  *@brief 2D Correlation coefficient functionality.
  *
  *INPUTS
  *@param image1 :  PGMIMage structure containing the first image to be compared.
  *@param image2 :  PGMIMage structure containing the second image to be compared.
  *
  *OUTPUTS
  *@param Correlation value between image1 & image2 (0-1).
  */
double corr2d(PGMImage* image1, PGMImage* image2){

    int image1_height=0, image2_height=0, image1_width=0;
    int image2_width=0,image1_numPix=0, image2_numPix=0,i=0,j=0;
    unsigned char tmpPix1,tmpPix2;
    double intPix1=0.0,intPix2=0.0, result=0.0;
    double numerator=0.0,denominator=0.0,sum1=0.0,sum2=0.0;
    double image1Mean=0.0, image2Mean=0.0;

    image1_width = image1->header.width;
    image2_width = image2->header.width;
    image1_height = image1->header.height;
    image2_height = image2->header.height;
    image1_numPix = image1_width * image1_height;
    image2_numPix = image2_width * image2_height;

    //printf("Image one width x height: %d x %d\n",image1_width, image1_height);
    //printf("Image two width x height: %d x %d\n",image2_width, image2_height);

    if(image1_width != image2_width || image1_height != image2_height){
      printf("Error: Cannot correlate images, dimensions do not match\n");
      exit(0);
    }

    for(i = 0; i<image1->header.height; i++){
      for(j = 0; j<image1->header.width; j++){
        tmpPix1 = image1->image[i][j];
        intPix1 = tmpPix1 & 0xFF;
        tmpPix2 = image2->image[i][j];
        intPix2 = tmpPix2 & 0xFF;
        image1Mean += intPix1;
        image2Mean += intPix2;
      }
    }

    image1Mean = round(image1Mean / image1_numPix);
    image2Mean = round(image2Mean / image2_numPix);

    //printf("Image 1 mean: %f\n", image1Mean);
    //printf("Image 2 mean: %f\n", image2Mean);

    for(i = 0; i < image1->header.height; i++){
      for(j = 0; j < image1->header.width; j++){
        tmpPix1 = image1->image[i][j];
        intPix1 = tmpPix1 & 0xFF;

        tmpPix2 = image2->image[i][j];
        intPix2 = tmpPix2 & 0xFF;

        numerator += ((intPix1 - image1Mean) * (intPix2 - image2Mean));
        sum1 += ((intPix1 - image1Mean)*(intPix1 - image1Mean));
        sum2 += ((intPix2 - image2Mean)*(intPix2 - image2Mean));
      }
    }

    denominator = sqrt(sum1*sum2);

    // Protect against divide by zero for the correlation value
    if(denominator == 0) {
      result = 0;
    }
    else {
      result = numerator / denominator;
    }

    // Make sure correlation value is always positive
    if(result < 0) {
      result *= -1;
    }

    return result;
}

/**
  *@brief Copy data from one PGMImage structure to another.
  *
  *INPUTS
  *@param imageSource : Structure with PGMImage data to be transfered
  *@param imageDest   : Structure receiving PGMImage data from imageSource
  *
  *OUTPUTS
  *none
  */
void copyPGM(PGMImage* imageSource, PGMImage* imageDest){
    int i;
    imageDest->header.width = imageSource->header.width;
    imageDest->header.height = imageSource->header.height;
    imageDest->header.grayscale = imageSource->header.grayscale;
    imageDest->header.type[0] = imageSource->header.type[0];
    imageDest->header.type[1] = imageSource->header.type[1];
    imageDest->header.numHeightDigits = imageSource->header.numHeightDigits;
    imageDest->header.numWidthDigits = imageSource->header.numWidthDigits;
    imageDest->header.numGrayscaleDigits = imageSource->header.numGrayscaleDigits;

    allocatePGMImageArray(imageDest);
    for(i=0; i<imageDest->header.height; i++){
        memcpy(imageDest->image[i],imageSource->image[i],imageDest->header.width*sizeof(char));
    }
}


/**
  *@brief Allocate heap memory for PGMImage structure image.
  *
  *INPUTS
  *@param pgm : PGMIMage structure to allocate image memory in
  *
  *OUTPUTS
  *none
  */
void allocatePGMImageArray(PGMImage* pgm){
    int i;
    if(pgm->header.height != 0 && pgm->header.width != 0){
        pgm->image = malloc(sizeof(unsigned char*)*pgm->header.height);
        for(i = 0; i < pgm->header.height; i++) {
          pgm->image[i] = malloc(sizeof(unsigned char)*pgm->header.width);
        }
    }
    else{
        printf("Error: Header was not previously defined.");
        exit(0);
    }
}

/**
  *@brief Function for deallocating heap memory allocated for the PGM image array.
  *
  *INPUTS
  *@param pgm: PGMImage structure containing image to have image array heap memory deallocated.
  *
  *OUTPUTS
  *none
  */
void deallocatePGMImageArray(PGMImage* pgm){
    int i;
    for(i = 0; i < pgm->header.height; i++) {
        free(pgm->image[i]);
        pgm->image[i] = NULL;
    }
}

/**
  *@brief Free allocated heap memory in PGMImage structure.
  *
  *INPUTS
  *@param img : Image structure to have free memory freed
  *
  *OUTPUTS
  *none
  */
void freePGMImage(PGMImage* img) {
  int i = 0;
  int height = 0;
  if(img != NULL) {
    height = img->header.height;
    for(i = 0; i < height; i++) {
      free(img->image[i]);
      img->image[i] = NULL;
    }
    free(img->image);
    img->image = NULL;
  }
}


