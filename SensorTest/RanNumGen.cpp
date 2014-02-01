//
//  RanNumGen.cpp
//  Box2DTestBed
//
//  Created by James Wucher on 8/9/13.
//
//

#include "RanNumGen.h"

void RanNumGen::SetSeed(uint32 seed)
{
   srand(seed);
}

// Returns a random floating point number from min to max.
float32 RanNumGen::RandFloat(float32 min, float32 max)
{
   if(max == min)
      return max;
   if(min > max)
   {
      float32 temp = min;
      min = max;
      max = temp;
   }
   float32 result = min + (float32)rand()/((float32)RAND_MAX/(max-min));
   return result;
}

// Returns a random integer between min and max (inclusive)
int32 RanNumGen::RandInt(int32 min, int32 max)
{
   if(max == min)
      return max;
   if(min > max)
   {
      int32 temp = min;
      min = max;
      max = temp;
   }
   float32 range = (1+max - min);
   int32 rnd = min + int((range * rand()) / (RAND_MAX + 1.0));
   return rnd;
}
