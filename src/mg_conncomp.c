/*
Primary accretion detection algorithm.

Connected component analysis functions.

Jack Lightholder
lightholder.jack16@gmail.com

Space and Terrestrial Robotic Exploration Laboratory (SpaceTREx)
Arizona State University
*/

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <math.h>
#include "mg.h"
#include "mg_conncomp.h"
#include "mg_centroid.h"
#include "mg_image.h"

static int SearchDirection[8][2] = {{0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1}};
static unsigned char **bitmap;
static int **labelmap;

/**
  *@brief Validate data in a PGMImage is ready for connected components analysis and
  *          allocate necessary memory.
  *
  *INPUTS
  *@param image   : PGMImage structure to be analyzed.
  *@param pwidth  : Width of the image.
  *@param pheight : Height of the image.
  *
  *OUTPUTS
  *none
  */
int validatePGM(PGMImage* image, int *pwidth, int *pheight)
{
	int i=0, x=0, y=0;

	*pwidth = image->header.width;
	*pheight = image->header.height;
	// malloc_validatePGM bitmap free in mg_conncompo.c
	bitmap   = malloc(*pheight * sizeof(unsigned char*));
	// malloc_validatePGM labelmap free in mg_conncompo.c
	labelmap = malloc(*pheight * sizeof(int*));

  if(bitmap == NULL|| labelmap == NULL)
	{
		return -2;
	}

	for(y = 0; y < *pheight; y++)
	{
	    // malloc_validatePGM bitmap[] free in mg_conncompo.c
		bitmap[y]   = malloc(*pwidth * sizeof(unsigned char));
		// malloc_validatePGM labelmap[] free in mg_conncompo.c
		labelmap[y] = malloc(*pwidth * sizeof(int));

        if(bitmap[y] == NULL || labelmap[y] == NULL)
        {
            return -2;
        }

		memset(bitmap[y], 0, *pwidth);
		memset(labelmap[y], 0, *pwidth * sizeof(int));
	}

	for(y = 1; y <= *pheight - 2; y++)
	{
		for(x = 1; x <= *pwidth - 2; x++)
		{
			i = image->image[y][x];
			bitmap[y][x] = (unsigned char)i;
		}
	}

	return 1;
}

/**
  *@brief Supporting function to trace connected component flood fill.
  *
  *INPUTS
  *@param               cy : Height of the image.
  *@param               cx : Width of the image.
  *@param tracingDirection : Direction to trace fill.
  *
  *OUTPUTS
  *none
  */
void Tracer(int *cy, int *cx, int *tracingdirection)
{
	int i, y, x;

	for(i = 0; i < 7; i++)
	{
		y = *cy + SearchDirection[*tracingdirection][0];
		x = *cx + SearchDirection[*tracingdirection][1];

		if(bitmap[y][x] == 0)
		{
			labelmap[y][x] = -1;
			*tracingdirection = (*tracingdirection + 1) % 8;
		}
		else
		{
			*cy = y;
			*cx = x;
			break;
		}
	}
}

/**
  *@brief Connected component contour tracing. Aspect of connected component flood fill.
  *
  *INPUTS
  *@param cy               : Height of the image.
  *@param cx               : Width of the image.
  *@param labelindex       : Fill flag for pixel.
  *@param tracingdirection : Direction to trace fill.
  *
  *OUTPUTS
  *none
  */
void ContourTracing(int cy, int cx, int labelindex, int tracingdirection)
{
	char tracingstopflag = 0, SearchAgain = 1;
	int fx=0, fy=0, sx = cx, sy = cy;

	Tracer(&cy, &cx, &tracingdirection);

	if(cx != sx || cy != sy)
	{
		fx = cx;
		fy = cy;

		while(SearchAgain)
		{
			tracingdirection = (tracingdirection + 6) % 8;
			labelmap[cy][cx] = labelindex;
			Tracer(&cy, &cx, &tracingdirection);

			if(cx == sx && cy == sy)
			{
				tracingstopflag = 1;
			}
			else if(tracingstopflag)
			{
				if(cx == fx && cy == fy)
				{
					SearchAgain = 0;
				}
				else
				{
					tracingstopflag = 0;
				}
			}
		}
	}
}

/**
  *@brief Connected Components analysis. Determines number of discrete objects in the image.  Determines
  *         and returns centroid coordinates for each connected component.
  *
  *INPUTS
  *@param image : PGMImage structure containing the file to be analyzed.
  *@param k     : Number of clusters
  *
  *OUTPUTS
  *@param ccCount  : Number of connected components detected.
  *
  *@pre PGMImage must contain black and white image.
  *
  */
Centroid* ConnectedComponentLabeling(PGMImage* image,int* ccCount, int* k)
{
	int height=0, width=0, cx=0, cy=0;
	int tracingdirection=0, ConnectedComponentsCount=0;
	int labelindex=0, i=0, count=0;
	Centroid pointbuff[500];
    Centroid *cents;

	if(validatePGM(image, &width, &height) != 1)
	{
		printf("Error: Cannot validate PGM structure of allocate memory.  Quitting program.");
		exit(0);
	}

	for(cy = 1; cy < height - 1; cy++)
	{
		for(cx = 1, labelindex = 0; cx < width - 1; cx++)
		{
			if(bitmap[cy][cx] == BLACKPIX)
			{
				if(labelindex != 0)
				{
					labelmap[cy][cx] = labelindex;
				}
				else
				{
					labelindex = labelmap[cy][cx];

					if(labelindex == 0)
					{
						labelindex = ++ConnectedComponentsCount;
						tracingdirection = 0;
						ContourTracing(cy, cx, labelindex, tracingdirection);
						labelmap[cy][cx] = labelindex;
						pointbuff[count].x = cx;
						pointbuff[count].y = cy;
						count++;
						if(count >= 500){
                            printf("Error: Too many connected components identified.  Exiting program.");
                            exit(0);
						}
					}
				}
			}
			// White pixel & pre-pixel has been labeled
			else if(labelindex != 0)
			{
				if(labelmap[cy][cx] == 0)
				{
					tracingdirection = 1;
					// Internal contour
					ContourTracing(cy, cx - 1, labelindex, tracingdirection);
				}
				labelindex = 0;
			}
		}
	}

    *ccCount = ConnectedComponentsCount;
    *k = sqrt(*ccCount/2);

    cents = createCents(ConnectedComponentsCount,*k);
    for(i=0; i<ConnectedComponentsCount; i++){
        cents[i].x = pointbuff[i].x;
        cents[i].y = pointbuff[i].y;
    }

  for(i = 0; i < height; i++) {
    free(bitmap[i]);
    bitmap[i] = NULL;
    free(labelmap[i]);
    labelmap[i] = NULL;
  }
  free(bitmap);
  free(labelmap);
  bitmap = NULL;
  labelmap = NULL;

  return cents;
}

/**
  * @brief Calculates the cluster density for a given array of centroids which are connected
  *        components.
  *
  * INPUTS
  * @param ccCount  : Length of the centroid array
  * @param centList : Array of connected component centroids
  *
  * OUTPUT
  * @param The sum of the distances divided by the count of the connected components
  */
double calcClusterDensity(int ccCount, const Centroid* centList) {

  int i=0;
  double distance=0.0, distanceSum=0.0;

  for(i=0; i<ccCount; i++){
    distanceSum += centList[i].distances[centList[i].kGroup];
  }
  distance = distanceSum / ccCount;

  return distance;
}



