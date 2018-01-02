/*
Primary accretion detection algorithm.

Data queuing functions for prioritizing based on likelihood of primary accretion.

Jack Lightholder
lightholder.jack16@gmail.com

Space and Terrestrial Robotic Exploration Laboratory (SpaceTREx)
Arizona State University
*/

#ifndef MG_DOWNLINK_H_INCLUDED
#define MG_DOWNLINK_H_INCLUDED

void downlinkImage(PGMImage* image, char* path,int index,int* downlinkCount,bool downlinked[],double score[],int startImg);
void downlinkData(int downlinkPercentage,Shift* acceleration,double* kDistances,int startImg,int numImages);

#endif // MG_DOWNLINK_H_INCLUDED
