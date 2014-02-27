//
//  GraphicsUtilities.cpp
//  Box2DTestBed
//
//  Created by James Wucher on 9/2/13.
//
//

#include "GraphicsUtilities.h"


void GraphicsUtilities::AdjustSpriteSizeToMaximumPixels(CCSprite* sprite, uint32 pixels)
{
   CCSize size = sprite->getContentSize();
   uint32 maxDimension = MAX(size.width,size.height);
   float32 scale = 1.0*pixels/maxDimension;
   sprite->setScale(scale);
}
