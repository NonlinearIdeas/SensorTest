//
//  RanNumGen.h
//  Box2DTestBed
//
//  Created by James Wucher on 8/9/13.
//
//

#ifndef __Box2DTestBed__RanNumGen__
#define __Box2DTestBed__RanNumGen__

#include "CommonSTL.h"
#include "CommonProject.h"

class RanNumGen
{
   
public:
   // Sets the random seed.
   static void SetSeed(uint32 seed);
   // Returns a random floating point number from min to max.
   static float32 RandFloat(float32 min, float32 max);
   // Returns a random integer between min and max
   static int32 RandInt(int32 min, int32 max);
};

#endif /* defined(__Box2DTestBed__RanNumGen__) */
