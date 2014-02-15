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
   
   
   bool IsNearTarget();
   void ApplyTurnTorque();
   void ApplyThrust();
   void PrepareForMotion();
   void EnterSeek();
   void ExecuteSeek();
   void EnterIdle();
   void ExecuteIdle();
   void EnterTurnTowards();
   void ExecuteTurnTowards();
   void EnterFollowPath();
   void ExecuteFollowPath();
   void ExecuteState(STATE_T state);
   void EnterState(STATE_T state);
   void ChangeState(STATE_T state);
   
protected:
   Vec2& GetTargetPos() { return _targetPos; }
   list<Vec2>& GetPath() { return _path; }

   PIDController& GetTurnController()
   {
      return _turnController;
   }
   
   inline float32 GetMaxLinearAcceleration() { return _maxLinearAcceleration; }
   inline void SetMaxLinearAcceleration(float32 maxLinearAcceleration) { _maxLinearAcceleration = maxLinearAcceleration; }
   
   inline float32 GetMaxAngularAcceleration() { return _maxAngularAcceleration; }
   inline void SetMaxAngularAcceleration(float32 maxAngularAcceleration) { _maxAngularAcceleration = maxAngularAcceleration; }
   
   inline float32 GetMinSeekDistance() { return _minSeekDistance; }
   inline void SetMinSeekDistance(float32 minSeekDistance) { _minSeekDistance = minSeekDistance; }
   
   inline float32 GetMaxSpeed() { return _maxSpeed; }
   inline void SetMaxSpeed(float32 maxSpeed) { _maxSpeed = maxSpeed; }
   
   virtual void CreateBody(b2World& world, const b2Vec2& position, float32 angleRads);
   virtual void StopBody();
   virtual void UpdateDisplay() { }
   virtual void SetupTurnController();
   
   
public:
   

   // Constructor
	MovingEntity();
   ~MovingEntity();
   
   
   bool Create(b2World& world,const Vec2& position, float32 angleRads);
   
   // Commands - Use thse to change the state
   // of the missile.
   void CommandFollowPath(const list<Vec2>& path);
   void CommandTurnTowards(const Vec2& position);
   void CommandSeek(const Vec2& position);
   void CommandIdle();
   void Update();
};

#endif /* defined(__MovingEntity__) */
