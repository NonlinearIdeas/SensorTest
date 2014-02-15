/********************************************************************
 * File   : HasFlags.h
 * Project: Multiple
 *
 ********************************************************************
 * Created on 2/15/14 By Nonlinear Ideas Inc.
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

#ifndef __HasFlags__
#define __HasFlags__

#include "CommonProject.h"
#include "CommonSTL.h"

class HasFlags
{
private:
   uint32 _flags;
public:
   enum
   {
      // ---------------------------------
      // Flags that reflect "what this is or can do"
      // ---------------------------------
      // Is this a graph sensor?
      HF_IS_GRAPH_SENSOR         = 1 << 0,
      // Can this entity move at all?
      HF_CAN_MOVE                = 1 << 1,
      // Is this a connected node?
      HF_IS_CONNECTED            = 1 << 2,
      // ---------------------------------
      // Priority Scheduling Flags
      // ---------------------------------
      // Every Frame
      HF_UPDATE_PRIO_1           = 1 << 17,
      // Every Other Frame (about 15x per second)
      HF_UPDATE_PRIO_2           = 1 << 18,
      // Every 4 Frames (about 7.5x per second)
      HF_UPDATE_PRIO_3           = 1 << 19,
      // Every 15 Frames (about 2x per second)
      HF_UPDATE_PRIO_4           = 1 << 20,
      // Every 30 Frames (about 1x per second)
      HF_UPDATE_PRIO_5           = 1 << 21,
      // Does not need to update the diplay regularly.
      HF_NO_DISPLAY_UPDATE       = 1 << 16,
   };
   
   
   
   HasFlags(uint32 flags) : _flags(flags)
   {
      
   }
   
   HasFlags() : _flags(0)
   {
      
   }
   
   inline void SetFlags(uint32 value)
   {
      _flags = value;
   }
   
   inline uint32 GetFlags() const
   {
      return _flags;
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
};

#endif /* defined(__HasFlags__) */
