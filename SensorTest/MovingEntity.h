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
   
   virtual void ApplyThrust()
   {
      // Get the distance to the target.
      Vec2 toTarget = GetTargetPos() - GetBody()->GetWorldCenter();
      toTarget.Normalize();
      Vec2 desiredVel = GetMaxSpeed()*toTarget;
      Vec2 currentVel = GetBody()->GetLinearVelocity();
      Vec2 thrust = GetMaxLinearAcceleration()*(desiredVel - currentVel);
      GetBody()->ApplyForceToCenter(thrust);
   }
   
   void PrepareForMotion()
   {
      GetBody()->SetLinearDamping(0.0f);
      GetBody()->SetAngularDamping(0.0f);
   }
   
   void EnterSeek()
   {
      PrepareForMotion();
      GetTurnController().ResetHistory();
   }
   
   void ExecuteSeek()
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
   
   
   void EnterIdle()
   {
      StopBody();
   }
   
   void ExecuteIdle()
   {
   }
   
   void EnterTurnTowards()
   {
      PrepareForMotion();
      GetTurnController().ResetHistory();
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
         PrepareForMotion();
         GetTurnController().ResetHistory();
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
   
   virtual void UpdateDisplay()
   {
   }
   

   
public:
   virtual void StopBody();
   virtual void SetupTurnController();
   virtual void CreateBody(b2World& world, const b2Vec2& position, float32 angleRads);
   
   
   inline float32 GetMaxLinearAcceleration() { return _maxLinearAcceleration; }
   inline void SetMaxLinearAcceleration(float32 maxLinearAcceleration) { _maxLinearAcceleration = maxLinearAcceleration; }
   
   inline float32 GetMaxAngularAcceleration() { return _maxAngularAcceleration; }
   inline void SetMaxAngularAcceleration(float32 maxAngularAcceleration) { _maxAngularAcceleration = maxAngularAcceleration; }
   
   inline float32 GetMinSeekDistance() { return _minSeekDistance; }
   inline void SetMinSeekDistance(float32 minSeekDistance) { _minSeekDistance = minSeekDistance; }
   
   inline float32 GetMaxSpeed() { return _maxSpeed; }
   inline void SetMaxSpeed(float32 maxSpeed) { _maxSpeed = maxSpeed; }
   
   
   // Constructor
	MovingEntity() :
   Entity(EF_CAN_MOVE,2),
   _state(ST_IDLE)
   {
   }
   
   
   bool Create(b2World& world,const Vec2& position, float32 angleRads)
   {
      CreateBody(world,position,angleRads);
      SetupTurnController();
      return true;
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
      UpdateDisplay();
   }
};

#endif /* defined(__MovingEntity__) */
