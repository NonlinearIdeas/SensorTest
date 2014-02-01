/********************************************************************
 * File   : Snake.h
 * Project: SensorTest
 *
 ********************************************************************
 * Created on 1/31/14 By Nonlinear Ideas Inc.
 * Copyright (c) 2014 Nonlinear Ideas Inc. All rights reserved.
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

#ifndef __Snake__
#define __Snake__

#include "CommonProject.h"
#include "CommonSTL.h"
#include "MovingEntity.h"

class Snake : public MovingEntity
{
private:
   vector<Body*> _segments;
   
public:
	Snake()
   {
      
   }
   
	virtual ~Snake()
   {
      for(int idx = 0; idx < _segments.size(); ++idx)
      {
         b2Body* body = _segments[idx];
         body->GetWorld()->DestroyBody(body);
      }
      _segments.clear();
   }
   

public:
   virtual void StopBody();
   virtual void CreateBody(b2World& world, const b2Vec2& position, float32 angleRads);
};

#endif /* defined(__Snake__) */
