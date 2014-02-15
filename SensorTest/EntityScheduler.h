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
 *
 * Assumptions:
 * 1. If an Entity has Update() called each frame (HF_UPDATE_PRIO_1),
 *    then it also has UpdateDisplay() called each frame.
 * 2. UpdateDisplay() will be called on every entity that 
 *    has the HF_NEEDS_DISPLAY_UPDATE set every frame, regardless
 *    of priority.
 * 3. Execution of updates is hopefully "optimal".  Removal or
 *    addition of Entities needing update may not be so fast.
 * 4. The flags on the entity define how it is handled by the 
 *    scheduler.  Changing them after creation WILL NOT change
 *    the schedule.  Only removing, changing, a re-adding will
 *    change them.  THIS PROBABLY SHOULD NOT HAPPEN AND MAY BE
 *    A DESIGN FLAW (i.e. why do you need to change the schedule).
 */
class EntityScheduler : public SingletonDynamic<EntityScheduler>
{
private:
   typedef vector<Entity*> ENTITY_LIST_T;
   typedef vector<ENTITY_LIST_T> FRAME_LIST_T;
   typedef map<Entity*, int32> PHASE_MAP_T;
   
   ENTITY_LIST_T _needUpdateDisplay;
   ENTITY_LIST_T _prio1Updates;
   FRAME_LIST_T _prio2Updates;
   FRAME_LIST_T _prio3Updates;
   FRAME_LIST_T _prio4Updates;
   FRAME_LIST_T _prio5Updates;
   PHASE_MAP_T _phaseMap;
   
   uint32 _frame;
   
   void RegisterPrio(Entity* entity, uint32 skip, FRAME_LIST_T& frameList);

   void UpdateEntityList(ENTITY_LIST_T& entList);
   void RemoveEntity(Entity* entity, FRAME_LIST_T& frameList, uint32 skip);
   void RemoveEntity(Entity* entity, ENTITY_LIST_T& entityList);
   
public:
   
   virtual bool Init();
   virtual void Reset();
   virtual void Shutdown();

   void Register(Entity* entity);
   void DeRegister(Entity* entity);
   void Update();
   
};

#endif /* defined(__EntityScheduler__) */
