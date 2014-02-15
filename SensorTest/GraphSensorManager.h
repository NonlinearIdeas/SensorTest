/********************************************************************
 * File   : GraphSensorManager.h
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

#ifndef __GraphSensorManager__
#define __GraphSensorManager__

#include "CommonSTL.h"
#include "CommonProject.h"
#include "SingletonTemplate.h"
#include "GraphSensor.h"
#include "GraphSensorGenerator.h"
#include "EntityManager.h"

#define SHOW_SENSOR_BODIES


/* This class is a container and manager for
 * GraphSensor objects.  It creates them and
 * loads them into the EntityManager.  When
 * a new set of sensors is created, this object
 * notifies the EntityManager which is responsible
 * for destroying the underlying objects.
 */
class GraphSensorManager : public SingletonDynamic<GraphSensorManager>
{
public:
   typedef set<GraphSensor*> SENSOR_SET_T;
   typedef set<GraphSensor*>::iterator SENSOR_SET_ITER_T;

private:
   
   vector<GraphSensor*> _sensors;
   SENSOR_SET_T _changedSensors;
   
   void RemoveSensors()
   {
      _changedSensors.clear();
      _sensors.clear();
   }
   
public:
   const vector<GraphSensor*> GetSensors() const
   {
      return _sensors;
   }
   
   const SENSOR_SET_T GetChangedSensors() const
   {
      return _changedSensors;
   }
   
   void ClearChangedSensors()
   {
      _changedSensors.clear();
   }
   
   virtual bool Init()
   {
      Reset();
      return true;
   }
   
   virtual void Reset()
   {
      RemoveSensors();
   }
   
   virtual void Shutdown()
   {
      Reset();
   }
   
   /* This function will load sensors from the GraphSensorGenerator
    * container.
    */
   void LoadSensors(GraphSensorGenerator& generator)
   {
      const vector<GraphSensor*> sensors = generator.GetSensors();
      assert(sensors.size() > 0);
      _sensors.resize(sensors.size());
      for(uint32 idx = 0; idx < sensors.size(); ++idx)
      {
         _sensors[idx] = sensors[idx];
      }
   }
   

   /* This method is called by the physics engine when a sensor
    * has a beginContact/endContact event.  The call is made
    * AFTER all the events have occured and the sensor itself 
    * as accumulated the begin/end count (so it should be positive
    * or 0).
    *
    * A client comes along (later) and reads the changed sensors.
    * After they are read, they should be cleared for the next
    * round of physics.  They don't have to be cleared every round,
    * just on a regular basis.
    *
    */
   void UpdateGraphSensorState(GraphSensor* sensor)
   {
      _changedSensors.insert(sensor);
#ifdef SHOW_SENSOR_BODIES
      sensor->GetBody()->SetDebugDraw(sensor->GetContactCount() != 0);
#endif
   }
   
   GraphSensorManager()
   {
   }
};

#endif /* defined(__GraphSensorManager__) */
