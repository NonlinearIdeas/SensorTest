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
      EF_IS_GRAPH_SENSOR = 1 << 0,
      EF_FLAG_MAX = 1 << 31
   };
   
   enum
   {
      DEFAULT_ENTITY_ID = -1,
   };
private:
   uint32 _ID;
   uint32 _flags;
   
public:
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
   _flags(0)
   {
   }
   
};


#endif /* defined(__Entity__) */
