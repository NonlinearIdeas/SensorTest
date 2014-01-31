/********************************************************************
 * File   : EntityManager.h
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

#ifndef __EntityManager__
#define __EntityManager__

#include "CommonSTL.h"
#include "CommonProject.h"
#include "SingletonTemplate.h"
#include "Entity.h"

/* This class is a container for Entity objects
 * Once the Entity object is created (external to
 * this class), it is registered here and THIS
 * CONTAINER OWNS THE OBJECT.  This container
 * is responsible for releasing the memory associated
 * with the object. 
 *
 * If you deregister an entity from the EntityManager,
 * you are responsible for releasing its memory as well.
 */
class EntityManager : public SingletonDynamic<EntityManager>
{

private:
   uint32 _nextEntityID;
   typedef map<uint32, Entity*> ENTITY_MAP_T;
   typedef map<uint32, Entity*>::iterator ENTITY_MAP_ITER_T;
   
   ENTITY_MAP_T _entities;
   
   /* This will remove all
    * entities and delete them.
    */
   void RemoveAllEntities()
   {
      for(ENTITY_MAP_ITER_T iter = _entities.begin(); iter != _entities.end(); ++iter)
      {
         delete iter->second;
      }
      _entities.clear();
      _nextEntityID = 100;
   }
   
public:
   
   /* Remove all entities and 
    * delete them as well.
    */
   virtual void Reset()
   {
      RemoveAllEntities();
   }
   
   virtual bool Init()
   {
      Reset();
      return true;
   }
   
   virtual void Shutdown()
   {
      Reset();
   }
   
   EntityManager()
   {
      Reset();
   }
   
   ~EntityManager()
   {
      RemoveAllEntities();
   }
   
   uint32 GetCount()
   {
      return _entities.size();
   }
   
   Entity* GetEntity(uint32 ID)
   {
      ENTITY_MAP_ITER_T iter = _entities.find(ID);
      if(iter == _entities.end())
         return NULL;
      return iter->second;
   }
   
   void RegisterEntity(Entity* entity)
   {
      assert(entity != NULL);
      assert(entity->GetID() == Entity::DEFAULT_ENTITY_ID);
      entity->SetID(++_nextEntityID);
      uint32 ID = entity->GetID();
      // Verify an entity with the same ID does not already exist.
      assert(GetEntity(entity->GetID()) == NULL);
      _entities[ID] = entity;
   }
   
   Entity* DeregisterEntity(uint32 ID)
   {
      ENTITY_MAP_ITER_T iter = _entities.find(ID);
      assert(iter != _entities.end());
      Entity* entity = iter->second;
      _entities.erase(iter);
      return entity;
   }
};

#endif /* defined(__EntityManager__) */
