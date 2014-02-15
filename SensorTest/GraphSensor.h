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
#include "GraphCommon.h"

class GraphSensor : public Entity
{
private:
   int32 _contactCount;
   int32 _index;
   GraphNode* _graphNode;
   
public:
   
   /* This is not terribly efficient, but for now,
    * the GraphSensor is "twinned" with a GraphNode.
    * This allows the sensor to update the node based
    * on its occupancy.
    */
   inline void SetGraphNode(GraphNode* graphNode)
   {
      _graphNode = graphNode;
   }
   
   inline void ResetContactCount()
   {
      _contactCount = 0;
   }
   
   inline bool IsClear()
   {
      return (_contactCount == 0);
   }
   
   void UpdateContactCount(int32 count);
   
   inline int32 GetContactCount()
   {
      return _contactCount;
   }
   
   inline int32 GetIndex()
   {
      return _index;
   }
   
   inline void SetIndex(int32 index)
   {
      _index = index;
   }
      
   GraphSensor() :
   _contactCount(0),
   _graphNode(NULL)
   {
      SetFlag(HF_IS_GRAPH_SENSOR);
   }
   
   ~GraphSensor()
   {
   }
};

#endif /* defined(__GraphSensor__) */
