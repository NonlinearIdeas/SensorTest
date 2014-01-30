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

#include "GraphSensor.h"

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
 */
class GraphSensorGenerator
{
private:
   vector<GraphSensor*> _sensors;
protected:
   vector<GraphSensor*>& GetSensors()
   {
      return _sensors;
   }
   
public:
   virtual void CreateSensors() = 0;
   
   const vector<GraphSensor*>& GetSensorsConst() const
   {
      return _sensors;
   }
};

#endif /* defined(__GraphSensorGenerator__) */
