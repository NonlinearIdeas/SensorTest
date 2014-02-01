/********************************************************************
 * File   : MovingEntity.cpp
 * Project: MissileDemo
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


#include "MovingEntity.h"

void MovingEntity::StopBody()
{
   GetBody()->SetLinearDamping(4.0);
   GetBody()->SetAngularDamping(2.5);
}


void MovingEntity::SetupTurnController()
{
   GetBody()->SetAngularDamping(0);
   PIDController& turnController = GetTurnController();
   
   turnController.ResetHistory();
   _turnController.ResetHistory();
   _turnController.SetKDerivative(5.0);
   _turnController.SetKProportional(1.0);
   _turnController.SetKIntegral(0.05);
   _turnController.SetKPlant(1.0);
}

void MovingEntity::CreateBody(b2World& world, const b2Vec2& position, float32 angleRads)
{
   // Create the body.
   b2BodyDef bodyDef;
   bodyDef.position = position;
   bodyDef.type = b2_dynamicBody;
   Body* body = world.CreateBody(&bodyDef);
   assert(body != NULL);
   // Store it in the base.
   SetBody(body);
   
   // Now attach fixtures to the body.
   FixtureDef fixtureDef;
   PolygonShape polyShape;
   vector<Vec2> vertices;
   
   const float32 VERT_SCALE = GetSizeMeters();
   fixtureDef.shape = &polyShape;
   fixtureDef.density = 1.0;
   fixtureDef.friction = 1.0;
   fixtureDef.isSensor = false;
   
   // Nose
   vertices.clear();
   vertices.push_back(Vec2(4*VERT_SCALE,2*VERT_SCALE));
   vertices.push_back(Vec2(4*VERT_SCALE,-2*VERT_SCALE));
   vertices.push_back(Vec2(8*VERT_SCALE,-0.5*VERT_SCALE));
   vertices.push_back(Vec2(8*VERT_SCALE,0.5*VERT_SCALE));
   polyShape.Set(&vertices[0],vertices.size());
   body->CreateFixture(&fixtureDef);
   body->SetLinearDamping(0.25);
   body->SetAngularDamping(0.25);
   
   // Main body
   vertices.clear();
   vertices.push_back(Vec2(-4*VERT_SCALE,2*VERT_SCALE));
   vertices.push_back(Vec2(-4*VERT_SCALE,-2*VERT_SCALE));
   vertices.push_back(Vec2(4*VERT_SCALE,-2*VERT_SCALE));
   vertices.push_back(Vec2(4*VERT_SCALE,2*VERT_SCALE));
   polyShape.Set(&vertices[0],vertices.size());
   body->CreateFixture(&fixtureDef);
   
   // Setup Parameters
   SetMaxAngularAcceleration(20*M_PI);
   SetMaxLinearAcceleration(10);
   SetMaxSpeed(5);
   SetMinSeekDistance(0.5);
}
