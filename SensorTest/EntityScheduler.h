/********************************************************************
 * File   : EntityScheduler.h
 * Project: SensorTest
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

#ifndef __EntityScheduler__
#define __EntityScheduler__

#include "CommonSTL.h"
#include "CommonProject.h"
#include "SingletonTemplate.h"
#include "Entity.h"


/* This class executes updates of the entities based on
 * their priority.  This allows for a Level of Detail (LOD)
 * as well as for entities that don't require an update every
 * cycle to automatically do their update periodically but 
 * not tax the system.
 */
class EntityScheduler : public SingletonDynamic<EntityScheduler>
{   
public:
   
   virtual bool Init()
   {
      Reset();
      return true;
   }
   
   virtual void Reset()
   {
   }
   
   virtual void Shutdown()
   {
      Reset();
   }
   
   
};

#endif /* defined(__EntityScheduler__) */
