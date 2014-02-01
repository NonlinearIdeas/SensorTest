/********************************************************************
 * File   : SpriteBatchLayer.cpp
 * Project: Multiple
 *
 ********************************************************************
 * Created on 10/20/13 By Nonlinear Ideas Inc.
 * Copyright (c) 2013 Nonlinear Ideas Inc. All rights reserved.
 ********************************************************************
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any
 * damages arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any
 * purpose, including commercial applications, and to alter it and
 * redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must
 *    not claim that you wrote the original software. If you use this
 *    software in a product, an acknowledgment in the product
 *    documentation would be appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and
 *    must not be misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source
 *    distribution.
 */

#include "SpriteBatchLayer.h"


SpriteBatchLayer::SpriteBatchLayer() :
_batchNode(NULL)
{
   
}

bool SpriteBatchLayer::init(const char* spriteImageFile, const char* spriteDataFile)
{
   CCSpriteFrameCache::sharedSpriteFrameCache()->addSpriteFramesWithFile(spriteDataFile, spriteImageFile);
   _batchNode = CCSpriteBatchNode::create(spriteImageFile);
   assert(_batchNode != NULL);
   addChild(_batchNode);
   return true;
}

SpriteBatchLayer* SpriteBatchLayer::create(const char* spriteImageFile, const char* spriteDataFile)
{
   SpriteBatchLayer *pRet = new SpriteBatchLayer();
   if (pRet && pRet->init(spriteImageFile,spriteDataFile))
   {
      pRet->autorelease();
      return pRet;
   }
   else
   {
      CC_SAFE_DELETE(pRet);
      return NULL;
   }
}

void SpriteBatchLayer::RemoveSprite(uint32 tag)
{
   removeChildByTag(tag,true);
}

void SpriteBatchLayer::AddSprite(CCSprite* sprite)
{
   assert(_batchNode != NULL);
   _batchNode->addChild(sprite);
}
