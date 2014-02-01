/********************************************************************
 * File   : StraightMovingEntity.cpp
 * Project: SensorTest
 *
 ********************************************************************
 * Created on 2/1/14 By Nonlinear Ideas Inc.
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


#include "StraightMovingEntity.h"


StraightMovingEntity::StraightMovingEntity()
{
	
}

StraightMovingEntity::~StraightMovingEntity()
{
	
}

void StraightMovingEntity::ApplyThrust()
{
   // Get the world vector (normalized) along the axis of the body.
   Vec2 direction = GetBody()->GetWorldVector(Vec2(1.0,0.0));
   Vec2 linVel = GetBody()->GetLinearVelocity();
   float32 speed = linVel.Length();
   if(speed >= GetMaxSpeed())
      speed = GetMaxSpeed();
   // Pile all the momentum in the direction the body is facing.
   // The missile "cannot" slip sideways.
   GetBody()->SetLinearVelocity(speed*direction);
   
   // Thrust Calculation
   float32 thrust = GetMaxLinearAcceleration() * GetBody()->GetMass();
   
   // Apply Thrust
   GetBody()->ApplyForceToCenter(thrust*direction);
}