/********************************************************************
 * File   : Box2DDebugDrawLayer.cpp
 * Project: Multiple
 *
 ********************************************************************
 * Created on 9/21/13 By Nonlinear Ideas Inc.
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

#include "Box2DDebugDrawLayer.h"


Box2DDebugDrawLayer::Box2DDebugDrawLayer() :
_world(NULL),
_debugDraw(NULL)
{
   
}

Box2DDebugDrawLayer* Box2DDebugDrawLayer::create(b2World* world)
{
   Box2DDebugDrawLayer *pRet = new Box2DDebugDrawLayer();
   if (pRet && pRet->init(world))
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

void Box2DDebugDrawLayer::draw()
{
   if(_world != NULL)
   {
      CCLayer::draw();
      
      ccGLEnableVertexAttribs( kCCVertexAttribFlag_Position );
      
      kmGLPushMatrix();
      
      _world->DrawDebugData();
      
      kmGLPopMatrix();
   }
}

bool Box2DDebugDrawLayer::init(b2World* world)
{
   if(!CCLayer::init())
   {
      return false;
   }
   
   _world = world;
   _debugDraw = new Box2dDebugDraw();
   world->SetDebugDraw(_debugDraw);
   
   uint32 flags = 0;
   flags += b2Draw::e_shapeBit;
   flags += b2Draw::e_jointBit;
   //   flags += b2Draw::e_aabbBit;
   flags += b2Draw::e_pairBit;
   flags += b2Draw::e_centerOfMassBit;
   _debugDraw->SetFlags(flags);
   return true;
}
Box2DDebugDrawLayer::~Box2DDebugDrawLayer()
{
   delete _debugDraw;
}
