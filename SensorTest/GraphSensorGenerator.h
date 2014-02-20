/********************************************************************
 * File   : GraphSensorGenerator.h
 * Project: SensorTest
 *
 ********************************************************************
 * Created on 1/25/14 By Nonlinear Ideas Inc.
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

#ifndef __GraphSensorGenerator__
#define __GraphSensorGenerator__

#include "CommonSTL.h"
#include "CommonProject.h"
#include "EntityManager.h"
#include "GraphSensor.h"
#include "GridCalculator.h"
#include "GraphCommon.h"

/* This abstract base class is a generator for the
 * graph sensor nodes.  It is used by the GraphSensorManager
 * to create the nodes so the GSM does not have to
 * know the actual geometry/positions of each node.
 *
 * The generator is responsible for generating the nodes
 * (internally) and then allowing the GSM to access each
 * one so that it may be registered with the EntityManager
 * and placed into the GSM for management.
 *
 * The generator is also responsible for generating
 * adjacency information for each sensor.  This is
 * used in graphs to create edges between graph
 * nodes.
 *
 */
class GraphSensorGenerator
{
public:
   typedef vector<GraphSensor*> SENSORS_T;
   typedef vector< vector<uint32> > SENSORS_ADJ_T;
private:
   SENSORS_T _sensors;
   SENSORS_ADJ_T _adjacentSensors;
   GridCalculator _gridCalculator;
   
protected:
   GraphSensorGenerator()
   {
      
   }
   

   
   /* Registers the sensors with the EntityManager.
    *
    */
   void RegisterSensors()
   {
      assert(_sensors.size() > 0);
      for(int idx = 0; idx < _sensors.size(); ++idx)
      {
         EntityManager::Instance().Register(_sensors[idx]);
      }
   }
   
   /* Override this function in derived classes to
    * create the sensors.
    *
    * They are stored in the (internal) list returned
    * by GetSensors().
    */
   virtual void GenerateSensors() = 0;
   
   /* Override this function to generate the sensor
    * adjacency information.  This information is 
    * stored in the member returned by GetAdjacentSensors().
    */
   virtual void GenerateAdjacency() = 0;
   
   
public:
   bool Create()
   {  // This should only be called ONE TIME
      assert(_sensors.size() == 0);
      assert(_adjacentSensors.size() == 0);
      if(_sensors.size() == 0 && _adjacentSensors.size() == 0)
      {
         GenerateSensors();
         GenerateAdjacency();
         assert(_sensors.size() == _adjacentSensors.size());
         RegisterSensors();
      }
      return _sensors.size() == _adjacentSensors.size();
   }
   
   GridCalculator& GetGridCalculator()
   {
      return _gridCalculator;
   }
   
   /* Generate a graph of the sensors if
    * somebody needs one.
    */
   void GenerateGraph(Graph& sensorGraph)
   {
      // Load the nodes into the graph
      sensorGraph.Reset();
      
      // Add the nodes in first.
      for(int idx = 0; idx < _sensors.size(); ++idx)
      {
         // Get the raw sensor from the generator
         GraphSensor* sensor = _sensors[idx];
         NavGraphNode* node = new NavGraphNode(idx);
         node->SetPos(sensor->GetBody()->GetWorldCenter());
         sensor->SetGraphNode(node);
         sensorGraph.AddNode(node);
      }
      // Now fill in the adjacdency lists.
      for(int idx = 0; idx < _sensors.size(); ++idx)
      {
         const vector<uint32>& adjVec = _adjacentSensors[idx];
         Vec2 srcPos = _sensors[idx]->GetBody()->GetWorldCenter();
         for(uint32 adjIdx = 0; adjIdx < adjVec.size(); ++adjIdx)
         {
            uint32 adjNodeIdx = adjVec[adjIdx];
            Vec2 desPos = _sensors[adjNodeIdx]->GetBody()->GetWorldCenter();
            NavGraphEdge* edge = new NavGraphEdge(idx,adjNodeIdx,srcPos,desPos);
            sensorGraph.AddEdge(edge);
         }
      }
      //   sensorGraph.Dump();
   }
   
   SENSORS_T& GetSensors()
   {
      return _sensors;
   }
   
   SENSORS_ADJ_T& GetAdjacentSensors()
   {
      return _adjacentSensors;
   }
};

#endif /* defined(__GraphSensorGenerator__) */
