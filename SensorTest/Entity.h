/********************************************************************
 * File   : Entity.h
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

#ifndef __Entity__
#define __Entity__

#include "CommonSTL.h"
#include "CommonProject.h"

class Entity
{
public:
   enum
   {
      // Flags 1 << 0 through 1 << 15 are reserved
      // for the base class.
      // Flags above this may be used by
      // derived classes.
      EF_IS_GRAPH_SENSOR   = 1 << 0,
      EF_CAN_MOVE          = 1 << 1,
      EF_FLAG_MAX          = 1 << 31
   };
   
   enum
   {
      DEFAULT_ENTITY_ID = -1,
   };
private:
   uint32 _ID;
   uint32 _flags;
   // Every entity has one "main" body which it
   // controls in some way.  Or not.
   b2Body* _body;
   // Every entity has a scale size from 1 to 100.
   // This maps on to the meters size of 0.1 to 10
   // in the physics engine.
   uint32 _scale;
   
protected:
   void SetScale(uint32 value)
   {
      assert(value >= 1);
      assert(value <= 100);
      _scale = value;
   }
   
public:

   void SetBody(b2Body* body)
   {
      assert(_body == NULL);
      if(_body != NULL)
      {
         CCLOG("BODY SHOULD BE NULL BEFORE ASSIGNING");
         _body->GetWorld()->DestroyBody(_body);
         _body = NULL;
      }
      _body = body;
      if(body != NULL)
      {
         _body->SetUserData(this);
         for (b2Fixture* f = _body->GetFixtureList(); f; f = f->GetNext())
         {
            f->SetUserData(this);
         }
      }
   }
   
   
   inline void SetFlag(uint32 flag)
   {
      _flags |= flag;
   }
   
   inline void ClearFlag(uint32 flag)
   {
      _flags &= ~flag;
   }
   
   inline bool IsFlagSet(uint32 flag) const
   {
      return (_flags & flag) > 0;
   }
   
   inline bool IsFlagClear(uint32 flag) const
   {
      return (_flags & flag) == 0;
   }
   
   inline void SetID(uint32 ID)
   {
      _ID = ID;
   }
   
   inline uint32 GetID() const
   {
      return _ID;
   }
   
   virtual string ToString(bool updateDescription = false)
   {
      string descr = "ID: ";
      descr += _ID;
      descr += "Flags: ";
      if(IsFlagSet(EF_IS_GRAPH_SENSOR))
         descr += "IS_FLAG_SENSOR ";
      return descr;
   }
   
   
   Entity() :
   _ID(DEFAULT_ENTITY_ID),
   _flags(0),
   _body(NULL),
   _scale(1)
   {
   }
   
   Entity(uint32 flags, uint32 scale) :
   _ID(DEFAULT_ENTITY_ID),
   _flags(flags),
   _body(NULL),
   _scale(scale)
   {
      
   }
   
   virtual void Update()
   {
      
   }
   
   virtual ~Entity()
   {
      if(_body != NULL)
      {
         _body->GetWorld()->DestroyBody(_body);
      }
   }
   
   inline static float32 ScaleToMeters(uint32 scale)
   {
      return 0.1*scale;
   }
   
   inline Body* GetBody()
   {
      return _body;
   }
   
   inline const Body* GetBody() const
   {
      return _body;
   }
   
   inline uint32 GetScale()
   {
      return _scale;
   }
   
   inline float32 GetSizeMeters()
   {
      return ScaleToMeters(_scale);
   }
};


#endif /* defined(__Entity__) */
