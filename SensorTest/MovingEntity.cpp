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

bool MovingEntity::IsNearTarget()
{
   Vec2 toTarget = GetTargetPos() - GetBody()->GetPosition();
   
   if(toTarget.LengthSquared() < GetMinSeekDistance()*GetMinSeekDistance())
   {
      return true;
   }
   return false;
}

void MovingEntity::ApplyTurnTorque()
{
   Vec2 toTarget = GetTargetPos() - GetBody()->GetPosition();
   
   float32 angleBodyRads = MathUtilities::AdjustAngle(GetBody()->GetAngle());
   float32 angleTargetRads = MathUtilities::AdjustAngle(atan2f(toTarget.y, toTarget.x));
   float32 angleError = MathUtilities::AdjustAngle(angleBodyRads - angleTargetRads);
   _turnController.AddSample(angleError);
   
   // Negative Feedback
   float32 angAcc = -_turnController.GetLastOutput();
   
   // This is as much turn acceleration as this
   // "motor" can generate.
   if(angAcc > GetMaxAngularAcceleration())
      angAcc = GetMaxAngularAcceleration();
   if(angAcc < -GetMaxAngularAcceleration())
      angAcc = -GetMaxAngularAcceleration();
   
   float32 torque = angAcc * GetBody()->GetInertia();
   GetBody()->ApplyTorque(torque);
}

void MovingEntity::ApplyThrust()
{
   // Get the distance to the target.
   Vec2 toTarget = GetTargetPos() - GetBody()->GetWorldCenter();
   toTarget.Normalize();
   Vec2 desiredVel = GetMaxSpeed()*toTarget;
   Vec2 currentVel = GetBody()->GetLinearVelocity();
   Vec2 thrust = GetMaxLinearAcceleration()*(desiredVel - currentVel);
   GetBody()->ApplyForceToCenter(thrust);
}

void MovingEntity::PrepareForMotion()
{
   GetBody()->SetLinearDamping(0.0f);
   GetBody()->SetAngularDamping(0.0f);
}

void MovingEntity::EnterSeek()
{
   PrepareForMotion();
   GetTurnController().ResetHistory();
}

void MovingEntity::ExecuteSeek()
{
   if(IsNearTarget())
   {
      StopBody();
      ChangeState(ST_IDLE);
   }
   else
   {
      ApplyTurnTorque();
      ApplyThrust();
   }
}


void MovingEntity::EnterIdle()
{
   StopBody();
}

void MovingEntity::ExecuteIdle()
{
}

void MovingEntity::EnterTurnTowards()
{
   PrepareForMotion();
   GetTurnController().ResetHistory();
}

void MovingEntity::ExecuteTurnTowards()
{
   ApplyTurnTorque();
}


void MovingEntity::EnterFollowPath()
{
   // If there are any points to follow,
   // then pop the first as the target
   // and follow it.  Otherwise, go idle.
   list<Vec2>& path = GetPath();
   if(path.size() > 0)
   {
      PrepareForMotion();
      GetTurnController().ResetHistory();
      GetTargetPos() = *(path.begin());
      path.erase(path.begin());
   }
   else
   {
      ChangeState(ST_IDLE);
   }
}

void MovingEntity::ExecuteFollowPath()
{
   list<Vec2>& path = GetPath();
   bool isNearTarget = IsNearTarget();
   if(path.size() == 0 && isNearTarget)
   {  // Done.
      ChangeState(ST_IDLE);
   }
   else if(isNearTarget)
   {  // Still more points to go.
      GetTargetPos() = *(path.begin());
      path.erase(path.begin());
      ApplyThrust();
      ApplyTurnTorque();
   }
   else
   {  // Just keep moving along..
      ApplyThrust();
      ApplyTurnTorque();
   }
}

void MovingEntity::ExecuteState(STATE_T state)
{
   switch(state)
   {
      case ST_IDLE:
         ExecuteIdle();
         break;
      case ST_TURN_TOWARDS:
         ExecuteTurnTowards();
         break;
      case ST_SEEK:
         ExecuteSeek();
         break;
      case ST_FOLLOW_PATH:
         ExecuteFollowPath();
         break;
      default:
         assert(false);
   }
}

void MovingEntity::EnterState(STATE_T state)
{
   switch(state)
   {
      case ST_IDLE:
         EnterIdle();
         break;
      case ST_TURN_TOWARDS:
         EnterTurnTowards();
         break;
      case ST_SEEK:
         EnterSeek();
         break;
      case ST_FOLLOW_PATH:
         EnterFollowPath();
         break;
      default:
         assert(false);
   }
}

void MovingEntity::ChangeState(STATE_T state)
{
   EnterState(state);
   _state = state;
}

MovingEntity::MovingEntity() :
Entity(HF_CAN_MOVE,2),
_state(ST_IDLE)
{
}

MovingEntity::~MovingEntity()
{
   
}

bool MovingEntity::Create(b2World& world,const Vec2& position, float32 angleRads)
{
   CreateBody(world,position,angleRads);
   SetupTurnController();
   return true;
}

// Commands - Use thse to change the state
// of the missile.
void MovingEntity::CommandFollowPath(const list<Vec2>& path)
{
   GetPath() = path;
   ChangeState(ST_FOLLOW_PATH);
}


void MovingEntity::CommandTurnTowards(const Vec2& position)
{
   GetTargetPos() = position;
   ChangeState(ST_TURN_TOWARDS);
}

void MovingEntity::CommandSeek(const Vec2& position)
{
   GetTargetPos() = position;
   ChangeState(ST_SEEK);
}

void MovingEntity::CommandIdle()
{
   ChangeState(ST_IDLE);
}

void MovingEntity::Update()
{
   ExecuteState(_state);
   UpdateDisplay();
}
