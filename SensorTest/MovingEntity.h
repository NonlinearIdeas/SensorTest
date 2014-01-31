/********************************************************************
 * File   : MovingEntity.h
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

#ifndef __MovingEntity__
#define __MovingEntity__

#include "CommonSTL.h"
#include "CommonProject.h"
#include "PIDController.h"
#include "MathUtilities.h"
#include "Entity.h"


class MovingEntity : public Entity
{
private:
   typedef enum
   {
      ST_IDLE,
      ST_TURN_TOWARDS,
      ST_SEEK,
      ST_FOLLOW_PATH,
      ST_MAX
   } STATE_T;
   
   Vec2 _targetPos;
   float32 _maxAngularAcceleration;
   float32 _maxLinearAcceleration;
   float32 _minSeekDistance;
   float32 _maxSpeed;
   list<Vec2> _path;
   
   STATE_T _state;
   // Create turning acceleration
   PIDController _turnController;
   vector<Body*> _segments;
   
   
   void StopBody()
   {
      Vec2 vel0(0,0);
      
      GetBody()->SetLinearVelocity(vel0);
      GetBody()->SetAngularVelocity(0);
      for(int idx = 0; idx < _segments.size();idx++)
      {
         _segments[idx]->SetLinearVelocity(vel0);
         _segments[idx]->SetAngularVelocity(0);
      }
   }
   
   
   bool IsNearTarget()
   {
      Vec2 toTarget = GetTargetPos() - GetBody()->GetPosition();
      
      if(toTarget.LengthSquared() < GetMinSeekDistance()*GetMinSeekDistance())
      {
         return true;
      }
      return false;
   }
   
   void ApplyTurnTorque()
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
   
   void ApplyThrust()
   {
      // Get the distance to the target.
      Vec2 toTarget = GetTargetPos() - GetBody()->GetWorldCenter();
      toTarget.Normalize();
      Vec2 desiredVel = GetMaxSpeed()*toTarget;
      Vec2 currentVel = GetBody()->GetLinearVelocity();
      Vec2 thrust = desiredVel - currentVel;
      GetBody()->ApplyForceToCenter(GetMaxLinearAcceleration()*thrust);
   }
   
   void EnterSeek()
   {
      SetupTurnController();
   }
   
   void ExecuteSeek()
   {
      if(IsNearTarget())
      {
         StopBody();
      }
      else
      {
         ApplyTurnTorque();
         ApplyThrust();
      }
   }
   
   
   void EnterIdle()
   {
      StopBody();
   }
   
   void ExecuteIdle()
   {
   }
   
   void EnterTurnTowards()
   {
      SetupTurnController();
   }
   
   void ExecuteTurnTowards()
   {
      ApplyTurnTorque();
   }
   
   void UpdatePathTarget()
   {
      list<Vec2>& path = GetPath();
      Vec2& targetPos = GetTargetPos();
      
      if(path.size() > 0)
      {
         targetPos = *path.begin();
         while(path.size() > 0 && IsNearTarget())
         {
            targetPos = *path.begin();
            path.pop_front();
         }
      }
      else
      {
         targetPos = GetBody()->GetPosition();
      }
   }
   
   void EnterFollowPath()
   {
      // If there are any points to follow,
      // then pop the first as the target
      // and follow it.  Otherwise, go idle.
      UpdatePathTarget();
      if(GetPath().size() > 0)
      {
         SetupTurnController();
      }
      else
      {
         ChangeState(ST_IDLE);
      }
   }
   
   void ExecuteFollowPath()
   {
      UpdatePathTarget();
      if(GetPath().size() > 0)
      {
         ApplyThrust();
         ApplyTurnTorque();
      }
      else
      {
         ChangeState(ST_IDLE);
      }
   }
   
   void ExecuteState(STATE_T state)
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
   
   void EnterState(STATE_T state)
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
   
   void ChangeState(STATE_T state)
   {
      EnterState(state);
      _state = state;
   }
   
protected:
   Vec2& GetTargetPos() { return _targetPos; }
   list<Vec2>& GetPath() { return _path; }

   PIDController& GetTurnController()
   {
      return _turnController;
   }
   
   virtual void SetupTurnController()
   {
      GetBody()->SetAngularDamping(0);
      PIDController& turnController = GetTurnController();
      
      turnController.ResetHistory();
      turnController.SetKDerivative(5.0);
      turnController.SetKProportional(2.0);
      turnController.SetKIntegral(0.1);
      turnController.SetKPlant(1.0);
   }
   
   virtual void CreateBody(b2World& world, const b2Vec2& position, float32 angleRads)
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
      
      const float32 VERT_SCALE = .5;
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
      const uint32 SNAKE_SEGMENTS = 4;
      Vec2 offset(-4*VERT_SCALE,0*VERT_SCALE);
      b2Body* pBodyA = body;
      b2Body* pBodyB = NULL;
      b2RevoluteJointDef revJointDef;
      revJointDef.collideConnected = false;
      
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
         pBodyB->SetLinearDamping(0.25);
         pBodyB->SetAngularDamping(0.25);
         // Offset the vertices for the fixture.
         for(int vidx = 0; vidx < vertices.size(); vidx++)
         {
            vertices[vidx] += offset;
            vertices[vidx].y *= 0.75;
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
      // Give the tail some real "drag" so that it pulls the
      // body straight when it can.
      pBodyB->SetLinearDamping(1.5);
      pBodyB->SetAngularDamping(1.5);
      
      // Setup Parameters
      SetMaxAngularAcceleration(4*M_PI);
      // As long as this is high, they forces will be strong
      // enough to get the body close to the target position
      // very quickly so the entity does not "circle" the
      // point.
      SetMaxLinearAcceleration(100);
      SetMaxSpeed(10);
      SetMinSeekDistance(1.0);
   }

   
public:
   inline float32 GetMaxLinearAcceleration() { return _maxLinearAcceleration; }
   inline void SetMaxLinearAcceleration(float32 maxLinearAcceleration) { _maxLinearAcceleration = maxLinearAcceleration; }
   
   inline float32 GetMaxAngularAcceleration() { return _maxAngularAcceleration; }
   inline void SetMaxAngularAcceleration(float32 maxAngularAcceleration) { _maxAngularAcceleration = maxAngularAcceleration; }
   
   inline float32 GetMinSeekDistance() { return _minSeekDistance; }
   inline void SetMinSeekDistance(float32 minSeekDistance) { _minSeekDistance = minSeekDistance; }
   
   inline float32 GetMaxSpeed() { return _maxSpeed; }
   inline void SetMaxSpeed(float32 maxSpeed) { _maxSpeed = maxSpeed; }
   
   
   // Constructor
	MovingEntity(b2World& world,const Vec2& position, float32 angleRads) :
   Entity(EF_CAN_MOVE,10),
   _state(ST_IDLE)
   {
      CreateBody(world,position,angleRads);
   }
   
   
   // Commands - Use thse to change the state
   // of the missile.
   void CommandFollowPath(const list<Vec2> path)
   {
      GetPath() = path;
      ChangeState(ST_FOLLOW_PATH);
   }
   
   
   void CommandTurnTowards(const Vec2& position)
   {
      GetTargetPos() = position;
      ChangeState(ST_TURN_TOWARDS);
   }
   
   void CommandSeek(const Vec2& position)
   {
      GetTargetPos() = position;
      ChangeState(ST_SEEK);
   }
   
   void SetTargetPosition(const Vec2& position)
   {
      GetTargetPos() = position;
   }
   
   void CommandIdle()
   {
      ChangeState(ST_IDLE);
   }
   
   virtual void Update()
   {
      ExecuteState(_state);
   }
   
protected:
private:
};

#endif /* defined(__MovingEntity__) */
