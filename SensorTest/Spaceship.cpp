/********************************************************************
 * File   : Spaceship.cpp
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


#include "Spaceship.h"
#include "Box2DShapeCache.h"
#include "Viewport.h"
#include "EntityUtilities.h"

void Spaceship::SetupTurnController()
{
   MovingEntity::SetupTurnController();
   
   PIDController& turnController = GetTurnController();
   
   turnController.ResetHistory();
   turnController.SetKDerivative(4.0);
   turnController.SetKProportional(2.0);
   turnController.SetKIntegral(0.05);
   turnController.SetKPlant(1.0);
}


void Spaceship::CreateBody(b2World& world, const b2Vec2& position, float32 angleRads)
{
   string root = "ship_0";
   string spriteName = root + ".png";
   
   _sprite = CCSprite::createWithSpriteFrameName(spriteName.c_str());
   _sprite->setTag((int)this);
   _sprite->setAnchorPoint(ccp(0.5,0.5));
   
   b2BodyDef bodyDef;
   bodyDef.position = position;
   bodyDef.type = b2_dynamicBody;
   Body* body = world.CreateBody(&bodyDef);
   assert(body != NULL);
   
   SetScale(10);
   // Add the polygons to the body.
   Box2DShapeCache::instance().addFixturesToBody(body, root, GetSizeMeters());
   
   SetBody(body);
   
   // Setup Parameters
   SetMaxAngularAcceleration(40*M_PI);
   SetMaxLinearAcceleration(5);
   SetMaxSpeed(5);
   SetMinSeekDistance(0.5);
   EntityUtilities::AdjustNodeScale(_sprite, this, 1.0, Viewport::Instance().GetPTMRatio());
   
   SetFlag(HF_CAN_MOVE | HF_NO_SENSOR_CONTACT);
}

void Spaceship::UpdateDisplay()
{
   // Update the sprite position and orientation.
   CCPoint pixel = Viewport::Instance().Convert(GetBody()->GetPosition());
   _sprite->setPosition(pixel);
   _sprite->setRotation(-CC_RADIANS_TO_DEGREES(GetBody()->GetAngle()));
}


