/********************************************************************
 * File   : GraphSensor.h
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

#ifndef __GraphSensor__
#define __GraphSensor__

#include "Entity.h"

class GraphSensor : public Entity
{
private:
   int32 _occupyCount;
   b2Body* _body;
   
public:
   
   inline void ResetOccupyCount()
   {
      _occupyCount = 0;
   }
   
   inline bool IsClear()
   {
      return (_occupyCount == 0);
   }
   
   inline void UpdateOccupyCount(int32 count)
   {
      _occupyCount+= count;
   }
   
   inline int32 GetOccupyCount()
   {
      return _occupyCount;
   }
   
   void SetBody(b2Body* body)
   {
      _body = body;
   }
   
   b2Body* GetBody()
   {
      return _body;
   }
   
   GraphSensor() :
   _occupyCount(0),
   _body(NULL)
   {
      SetFlag(EF_IS_GRAPH_SENSOR);
   }
   
   ~GraphSensor()
   {
      _body->GetWorld()->DestroyBody(_body);
   }
};

#endif /* defined(__GraphSensor__) */
