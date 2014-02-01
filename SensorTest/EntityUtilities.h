//
//  EntityUtilities.h
//  Box2DTestBed
//
//  Created by James Wucher on 8/4/13.
//
//

#ifndef __Box2DTestBed__EntityUtilities__
#define __Box2DTestBed__EntityUtilities__

#include "CommonSTL.h"
#include "CommonProject.h"
#include "Entity.h"

class EntityUtilities
{
public:
   /* This handy function is used to adjust the size of a node based
    on the entity.  A "scaleAdjust" value is provided because the
    polygon lines drawn for the physics may be slightly outside the
    image used to create them (because the alpha blending on the edges
    may show up as a edge outside the image edge).  This can be easily
    compensated by bumping the scale a bit.  The physics still looks pretty
    good.
    */
   static void AdjustNodeScale(CCNode* node, Entity* entity, float32 scaleAdjust, float32 ptmRatio)
   {
      float32 entitySizeMeters = entity->GetSizeMeters();
      CCSize nodeSize = node->getContentSize();
      float32 maxSizePixels = max(nodeSize.width,nodeSize.height);
      assert(maxSizePixels >= 1.0);
      float32 scale = scaleAdjust*(entitySizeMeters*ptmRatio/maxSizePixels);
      
      node->setScale(scale);
      
      /*
       CCLOG("Adjusting Node Scale: em:%f, msp:%f, ptm:%f, scale:%f",
       entitySizeMeters,
       maxSizePixels,
       ptm,
       scale
       );
       */
   }
   
   static void AdjustNodeScale(CCNode* node, Entity* entity, float32 maxSizePixels, float32 scaleAdjust, float32 ptmRatio)
   {
      assert(maxSizePixels >= 1.0);
      float32 entitySizeMeters = entity->GetSizeMeters();
      CCSize nodeSize = node->getContentSize();
      float32 scale = scaleAdjust*(entitySizeMeters*ptmRatio/maxSizePixels);
      
      node->setScale(scale);
      
      /*
       CCLOG("Adjusting Node Scale: em:%f, msp:%f, ptm:%f, scale:%f",
       entitySizeMeters,
       maxSizePixels,
       ptm,
       scale
       );
       */
   }

};

#endif /* defined(__Box2DTestBed__EntityUtilities__) */
