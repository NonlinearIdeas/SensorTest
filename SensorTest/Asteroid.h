/********************************************************************
 * File   : Asteroid.h
 * Project: Multiple
 *
 ********************************************************************
 * Created on 1/20/14 By Nonlinear Ideas Inc.
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

#ifndef __Asteroid__
#define __Asteroid__

#include "Entity.h"
#include "Viewport.h"
#include "RanNumGen.h"
#include "Box2DShapeCache.h"
#include "EntityUtilities.h"

class Asteroid : public Entity
{
private:
   b2Fixture* _hull;
   Vec2 _anchor;
   CCSprite* _sprite;
   float32 _targetRadius;
public:
   // Some getters to help us out.
   b2Fixture& GetHullFixture() const { return *_hull; }
   float32 GetTargetRadius() { return _targetRadius; }
   CCSprite* GetSprite() { return _sprite; }
   
   
   void UpdateDisplay()
   {
      // Update the sprite position and orientation.
      CCPoint pixel = Viewport::Instance().Convert(GetBody()->GetPosition());
      _sprite->setPosition(pixel);
      _sprite->setRotation(-CC_RADIANS_TO_DEGREES(GetBody()->GetAngle()));
   }
   
   virtual void Update()
   {
      Body* body = GetBody();
      
      Vec2 vRadius = body->GetPosition();
      Vec2 vTangent = vRadius.Skew();
      
      vTangent.Normalize();
      vRadius.Normalize();
      
      
      // If it is not moving...give it some spin.
      if(fabs(vTangent.Dot(body->GetLinearVelocity())) < 1)
      {
         body->SetLinearDamping(0.001);
         body->ApplyForceToCenter(body->GetMass()*1.5*vTangent);
         body->ApplyForce(vRadius,body->GetMass()*0.05*vRadius);
      }
      else
      {
         body->SetLinearDamping(0.05);
      }
   }
   
   ~Asteroid()
   {
      
   }
   
   Asteroid() :
   Entity(EF_CAN_MOVE,50)
   {
      
   }
   
   bool Create(b2World& world, const string& shapeName,const Vec2& position, float32 targetRadius)
   {
      _targetRadius = targetRadius;
      _anchor = position;
      
      string str = shapeName;
      str += ".png";
      _sprite = CCSprite::createWithSpriteFrameName(str.c_str());
      _sprite->setTag((int)this);
      _sprite->setAnchorPoint(ccp(0.5,0.5));
      
      //      _sprite->setVisible(false);
      
      b2BodyDef bodyDef;
      bodyDef.position = position;
      bodyDef.type = b2_dynamicBody;
      Body* body = world.CreateBody(&bodyDef);
      assert(body != NULL);
      
      // Add the polygons to the body.
      Box2DShapeCache::instance().addFixturesToBody(body, shapeName, GetSizeMeters());
      
      SetBody(body);      
      return true;
   }
   
};


#endif /* defined(__Asteroid__) */
