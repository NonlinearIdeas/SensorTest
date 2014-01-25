/********************************************************************
 * File   : Box2DDebugDrawLayer.h
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

#ifndef __Box2DDebugDrawLayer__
#define __Box2DDebugDrawLayer__

#include "cocos2d.h"
#include "Box2DDebugDraw.h"

using namespace cocos2d;

class Box2DDebugDrawLayer : public CCLayer
{
private:
   // Weak reference, do not delete here.
   b2World* _world;
   Box2dDebugDraw* _debugDraw;
   Box2DDebugDrawLayer();
protected:
   bool init(b2World* world);

public:
   static Box2DDebugDrawLayer* create(b2World* world);
   virtual void draw();
   virtual ~Box2DDebugDrawLayer();
   
   // Use this to get the drawing tool in case the size needs
   // to be adjusted.
   Box2dDebugDraw& GetDebugDraw() { return *_debugDraw; }

};

#endif /* defined(__Box2DDebugDrawLayer__) */
