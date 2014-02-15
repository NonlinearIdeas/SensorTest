/********************************************************************
 * File   : EntityScheduler.cpp
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


#include "EntityScheduler.h"


bool EntityScheduler::Init()
{
   Reset();
   return true;
}

void EntityScheduler::Reset()
{
   // Clear all these
   _needUpdateDisplay.clear();
   _prio1Updates.clear();
   _prio2Updates.clear();
   _prio3Updates.clear();
   _prio4Updates.clear();
   _prio5Updates.clear();
   // These get resized with a slot for
   // each frame.
   _prio2Updates.resize(TICKS_PER_SECOND);
   _prio3Updates.resize(TICKS_PER_SECOND);
   _prio4Updates.resize(TICKS_PER_SECOND);
   _prio5Updates.resize(TICKS_PER_SECOND);
   // Reset Frame Counter
   _frame = 0;
}

void EntityScheduler::Shutdown()
{
   Reset();
}

void EntityScheduler::RegisterPrio(Entity* entity, uint32 skip, FRAME_LIST_T& frameList)
{
   int32 phase = 0;
   int32 phaseCount = frameList[0].size();
   int32 tempCount;
   
   /* If I have done this correctly, this should be 
    * Wright's algorithm for scheduling.  Find the
    * array with the lowest number of elements and 
    * use that for the first phase location to 
    * place the entry.
    *
    * Note that we cache off the phase for the
    * entity so that it can removed much more
    * efficiently later (if needed).
    */
   for(int idx = 1; idx < skip; ++idx)
   {
      tempCount = frameList[idx].size();
      if(tempCount < phaseCount)
      {
         phase = idx;
         phaseCount = tempCount;
      }
   }
   // Now we know the phase.  Save it for later.
   _phaseMap[entity] = phase;
   
   
   for(int idx = phase; idx < TICKS_PER_SECOND; idx += skip)
   {
      frameList[idx].push_back(entity);
   }
   if(entity->IsFlagClear(Entity::HF_NO_DISPLAY_UPDATE))
   {
      _needUpdateDisplay.push_back(entity);
   }
}

void EntityScheduler::UpdateEntityList(ENTITY_LIST_T& entList)
{
   for(int idx = 0; idx < entList.size(); idx++)
   {
      entList[idx]->Update();
   }
}

void EntityScheduler::RemoveEntity(Entity* entity, ENTITY_LIST_T& entityList)
{
   ENTITY_LIST_T::iterator location = find(entityList.begin(),entityList.end(),entity);
   assert(location != entityList.end());
   if(location != entityList.end())
   {
      entityList.erase(location);
   }
}


void EntityScheduler::RemoveEntity(Entity* entity, FRAME_LIST_T& frameList, uint32 skip)
{
   PHASE_MAP_T::iterator iter = _phaseMap.find(entity);
   assert(iter != _phaseMap.end());
   if(iter != _phaseMap.end())
   {
      // Get the value
      int32 phase = iter->second;
      // Remove it from the phase map
      _phaseMap.erase(iter);
      // Iterate through the frameList and remove them all.
      for(int idx = phase; idx < TICKS_PER_SECOND; idx += skip)
      {
         RemoveEntity(entity, frameList[idx]);
      }
      // If this item needed a display update, remove
      // that as well.
      if(entity->IsFlagClear(Entity::HF_NO_DISPLAY_UPDATE))
      {
         RemoveEntity(entity, _needUpdateDisplay);
      }
      if(entity->IsFlagClear(Entity::HF_NO_DISPLAY_UPDATE))
      {
         RemoveEntity(entity, _needUpdateDisplay);
      }
   }
}

void EntityScheduler::Register(Entity* entity)
{
   if(entity->IsFlagSet(Entity::HF_UPDATE_PRIO_1))
   {
      _prio1Updates.push_back(entity);
   }
   else if(entity->IsFlagSet(Entity::HF_UPDATE_PRIO_2))
   {
      RegisterPrio(entity, 2, _prio2Updates);
   }
   else if(entity->IsFlagSet(Entity::HF_UPDATE_PRIO_3))
   {
      RegisterPrio(entity, 3, _prio3Updates);
   }
   else if(entity->IsFlagSet(Entity::HF_UPDATE_PRIO_4))
   {
      RegisterPrio(entity, 10, _prio4Updates);
   }
   else if(entity->IsFlagSet(Entity::HF_UPDATE_PRIO_5))
   {
      RegisterPrio(entity, 30, _prio5Updates);
   }
}

void EntityScheduler::DeRegister(Entity* entity)
{
   if(entity->IsFlagSet(Entity::HF_UPDATE_PRIO_1))
   {
      RemoveEntity(entity, _prio1Updates);
   }
   else if(entity->IsFlagSet(Entity::HF_UPDATE_PRIO_2))
   {
      RemoveEntity(entity, _prio2Updates, 2);
   }
   else if(entity->IsFlagSet(Entity::HF_UPDATE_PRIO_3))
   {
      RemoveEntity(entity, _prio2Updates, 3);
   }
   else if(entity->IsFlagSet(Entity::HF_UPDATE_PRIO_4))
   {
      RemoveEntity(entity, _prio2Updates, 10);
   }
   else if(entity->IsFlagSet(Entity::HF_UPDATE_PRIO_5))
   {
      RemoveEntity(entity, _prio2Updates, 30);
   }
}



void EntityScheduler::Update()
{
   // Update the "every frame" list.
   for(int32 idx = 0; idx < _prio1Updates.size(); ++idx)
   {
      _prio1Updates[idx]->Update();
      _prio1Updates[idx]->UpdateDisplay();
   }
   
   // Update the individual lists
   UpdateEntityList(_prio2Updates[_frame]);
   UpdateEntityList(_prio3Updates[_frame]);
   UpdateEntityList(_prio4Updates[_frame]);
   UpdateEntityList(_prio5Updates[_frame]);
   
   // Update the items that need a display call.
   for(int idx = 0; idx < _needUpdateDisplay.size(); ++idx)
   {
      _needUpdateDisplay[idx]->UpdateDisplay();
   }
   
   // Update the frame counter
   ++_frame;
   if(_frame == TICKS_PER_SECOND)
      _frame = 0;
}
