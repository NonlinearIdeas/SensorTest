/********************************************************************
 * File   : Snake.cpp
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


#include "Snake.h"

void Snake::StopBody()
{
   MovingEntity::StopBody();
   
   b2Vec2 vel0(0,0);
   
   for(int idx = 0; idx < _segments.size();idx++)
   {
      _segments[idx]->SetLinearVelocity(vel0);
      _segments[idx]->SetAngularVelocity(0);
   }
}

void Snake::CreateBody(b2World& world, const b2Vec2& position, float32 angleRads)
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
   
   float32 VERT_SCALE = GetSizeMeters();
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
   
   // NOW, create several duplicates of the "Main Body" fixture
   // but offset them from the previous one by a fixed amount and
   // overlap them a bit.
   const uint32 SNAKE_SEGMENTS = 3;
   Vec2 offset(-4*VERT_SCALE,0*VERT_SCALE);
   b2Body* pBodyA = body;
   b2Body* pBodyB = NULL;
   b2RevoluteJointDef revJointDef;
   revJointDef.collideConnected = false;
   revJointDef.lowerAngle = -0.5f * M_PI;
   revJointDef.upperAngle = 0.5f * M_PI;
   revJointDef.enableLimit = true;
   revJointDef.maxMotorTorque = 5.0f;
   revJointDef.motorSpeed = 0.0f;
   revJointDef.enableMotor = true;
   
   // Add some "regular segments".
   for(int idx = 0; idx < SNAKE_SEGMENTS; idx++)
   {
      // Create a body for the next segment.
      bodyDef.position = pBodyA->GetPosition() + offset;
      pBodyB = world.CreateBody(&bodyDef);
      // Set the user data.
      pBodyB->SetUserData(this);
      
      // Store the body as one of the segments.
      _segments.push_back(pBodyB);
      // Add some damping so body parts don't 'flop' around.
      pBodyB->SetLinearDamping(0.25);
      pBodyB->SetAngularDamping(0.25);
      // Offset the vertices for the fixture.
      for(int vidx = 0; vidx < vertices.size(); vidx++)
      {
         vertices[vidx] += offset;
         vertices[vidx].y *= 0.9;
      }
      // and create the fixture.
      polyShape.Set(&vertices[0],vertices.size());
      pBodyB->CreateFixture(&fixtureDef);
      
      // Create a Revolute Joint at a position half way
      // between the two bodies.
      Vec2 midpoint = (pBodyA->GetPosition() + pBodyB->GetPosition());
      revJointDef.Initialize(pBodyA, pBodyB, midpoint);
      world.CreateJoint(&revJointDef);
      // Update so the next time through the loop, we are
      // connecting the next body to the one we just
      // created.
      pBodyA = pBodyB;
   }
   // Make the next bunch of segments get "smaller" each time
   // to make a tail.
   for(int idx = 0; idx < SNAKE_SEGMENTS; idx++)
   {
      // Create a body for the next segment.
      bodyDef.position = pBodyA->GetPosition() + offset;
      pBodyB = world.CreateBody(&bodyDef);
      // Set the user data to point to this entity.
      pBodyB->SetUserData(this);
      
      _segments.push_back(pBodyB);
      // Add some damping so body parts don't 'flop' around.
      pBodyB->SetLinearDamping(0.5);
      pBodyB->SetAngularDamping(0.1);
      // Offset the vertices for the fixture.
      for(int vidx = 0; vidx < vertices.size(); vidx++)
      {
         vertices[vidx] += offset;
         vertices[vidx].y *= 0.75;
      }
      // and create the fixture.
      polyShape.Set(&vertices[0],vertices.size());
      pBodyB->CreateFixture(&fixtureDef);
      
      // Create a Revolute Joint at a position half way between the two bodies.
      Vec2 midpoint = (pBodyA->GetPosition() + pBodyB->GetPosition());
      revJointDef.Initialize(pBodyA, pBodyB, midpoint);
      world.CreateJoint(&revJointDef);
      // Update so the next time through the loop, we are
      // connecting the next body to the one we just
      // created.
      pBodyA = pBodyB;
   }
   // Give the tail some real "drag" so that it pulls the
   // body straight when it can.
   pBodyB->SetLinearDamping(1.5);
   pBodyB->SetAngularDamping(1.5);
   
   // Setup Parameters
   SetMaxAngularAcceleration(8*M_PI);
   SetMaxLinearAcceleration(10);
   SetMaxSpeed(5);
   SetMinSeekDistance(1.0);
}


