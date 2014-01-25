/********************************************************************
 * File   : SystemContactListener.h
 * Project: SensorTest
 *
 ********************************************************************
 * Created on 1/21/14 By Nonlinear Ideas Inc.
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

#ifndef __SystemContactListener__
#define __SystemContactListener__

#include "CommonSTL.h"
#include "CommonProject.h"
#include "SingletonTemplate.h"
#include "Entity.h"
#include "GraphSensor.h"
#include "GraphSensorManager.h"

class SystemContactListener : public ContactListener, public SingletonDynamic<SystemContactListener>
{
private:
   
   struct CONTACT_PAIR
   {
      Entity* entA;
      Entity* entB;
      int32 contactCount;
      
      CONTACT_PAIR(Entity* entA_,Entity* entB_) :
      entA(entA_),
      entB(entB_),
      contactCount(0)
      {
         
      }
   };
   
   typedef enum
   {
      BEGIN_CONTACT,
      END_CONTACT
   } CONTACT_PHASE;
   
   vector<CONTACT_PAIR> _contactPairs;
   
   void UpdateContact(Entity* entA, Entity* entB, CONTACT_PHASE phase)
   {
      for(uint32 idx = 0; idx < _contactPairs.size(); idx++)
      {
         CONTACT_PAIR& cp = _contactPairs[idx];
         if((cp.entA == entA && cp.entB == entB) ||
            (cp.entA == entB && cp.entB == entA))
         {  // We've had this contact before.
            switch(phase)
            {
               case BEGIN_CONTACT:
                  ++cp.contactCount;
                  break;
               case END_CONTACT:
                  --cp.contactCount;
                  break;
               default:
                  assert(false);
                  break;
            }
            return;
         }
      }
      CONTACT_PAIR cp(entA,entB);
      switch(phase)
      {
         case BEGIN_CONTACT:
            ++cp.contactCount;
            break;
         case END_CONTACT:
            --cp.contactCount;
            break;
         default:
            assert(false);
            break;
      }
      _contactPairs.push_back(cp);
   }
   
public:

   virtual bool Init()
   {
      _contactPairs.reserve(128);
      return true;
   }
   
   virtual void Reset()
   {
      _contactPairs.clear();
   }
   
   inline void NotifyContacts()
   {
      for(uint32 idx = 0; idx < _contactPairs.size(); idx++)
      {
         CONTACT_PAIR& cp = _contactPairs[idx];
         GraphSensorManager& gsm = GraphSensorManager::Instance();
         GraphSensor* sensor = NULL;
         Entity* entA = cp.entA;
         Entity* entB = cp.entB;
         if(entA->IsFlagSet(Entity::EF_IS_GRAPH_SENSOR))
         {
            sensor = (GraphSensor*)entA;
            sensor->UpdateOccupyCount(cp.contactCount);
            gsm.UpdateGraphSensorState(sensor);
         }
         else if(entB->IsFlagSet(Entity::EF_IS_GRAPH_SENSOR))
         {
            sensor = (GraphSensor*)entB;
            sensor->UpdateOccupyCount(cp.contactCount);
            gsm.UpdateGraphSensorState(sensor);
         }
         else
         {  // Must be two objects that ARE NOT SENSORS.
            // Notify somebody...
         }
      }
      _contactPairs.clear();
   }
   
   void PreSolve(b2Contact* contact, const b2Manifold* oldManifold)
   {
      
   }
   
   // BEWARE:  You may get multiple calls for the same event.
   inline void BeginContact(b2Contact* contact)
   {
      Entity* entA = (Entity*)contact->GetFixtureA()->GetBody()->GetUserData();
      Entity* entB = (Entity*)contact->GetFixtureB()->GetBody()->GetUserData();
      
      assert(entA != NULL);
      assert(entB != NULL);
      UpdateContact(entA,entB,BEGIN_CONTACT);
   }
   
   // BEWARE:  You may get multiple calls for the same event.
   inline void EndContact(b2Contact* contact)
   {
      Entity* entA = (Entity*)contact->GetFixtureA()->GetBody()->GetUserData();
      Entity* entB = (Entity*)contact->GetFixtureB()->GetBody()->GetUserData();
      
      assert(entA != NULL);
      assert(entB != NULL);
      UpdateContact(entA,entB,END_CONTACT);
   }
};

#endif /* defined(__SystemContactListener__) */
