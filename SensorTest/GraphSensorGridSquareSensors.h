/********************************************************************
 * File   : GraphSensorGridSquareSensors.h
 * Project: SensorTest
 *
 ********************************************************************
 * Created on 2/20/14 By Nonlinear Ideas Inc.
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

#ifndef __GraphSensorGridSquareSensors__
#define __GraphSensorGridSquareSensors__

#include "CommonProject.h"
#include "CommonSTL.h"
#include "GraphSensorGenerator.h"
#include "Viewport.h"

/* This class generates a graph sensor array on a rectangular
 * grid.
 *
 * Each sensor is a circle with a given radius and a separation
 * between the edges.
 *
 * The size of the world is gleaned from the Viewport.
 */
class GraphSensorGridSquareSensors : public GraphSensorGenerator
{
private:
   // Private constructor.
   GraphSensorGridSquareSensors()
   {
      
      
   }
   
   b2World* _world;
   float32 _diameter;
   float32 _separation;
   
   virtual void GenerateSensors()
   {
      Viewport& vp = Viewport::Instance();
      CCSize worldSize = vp.GetWorldSizeMeters();
      SENSORS_T& sensors = GetSensors();
      GridCalculator& gridCalc = GetGridCalculator();
      
      gridCalc.Init(worldSize.width, worldSize.height, _separation);
      
      // Calculate the rows and columns so that there is at least one gap between each
      // shape.
      int32 cols = gridCalc.GetCols();
      int32 rows = gridCalc.GetRows();
      int32 count = gridCalc.GetCount();
      
      
      b2BodyDef bodyDef;
      bodyDef.type = b2_staticBody;
      
      /*
       b2CircleShape shape;
       shape.m_radius = _diameter/2;
       shape.m_p = b2Vec2(0,0);
       */
      b2PolygonShape shape;
      shape.SetAsBox(_diameter/2, _diameter/2);
      
      
      b2FixtureDef fixtureDef;
      fixtureDef.shape = &shape;
      fixtureDef.isSensor = true;
      
      GraphSensor* sensor = NULL;
      b2Body* body = NULL;
      b2Fixture* fixture = NULL;
      
      CCLOG("Rows = %d, Cols = %d",rows,cols);
      
      for(int32 idx = 0; idx < count; ++idx)
      {
         Vec2 pos = gridCalc.CalcPosition(idx);
         bodyDef.position = pos;
         
         body = _world->CreateBody(&bodyDef);
         fixture = body->CreateFixture(&fixtureDef);
         
         sensor = new GraphSensor();
         sensor->SetBody(body);
         sensor->SetIndex(sensors.size());
         
         // We don't need to draw these in box2d debug
         body->SetDebugDraw(false);
         
         sensors.push_back(sensor);
      }
   }
   
   virtual void GenerateAdjacency()
   {
      Viewport& vp = Viewport::Instance();
      CCSize worldSize = vp.GetWorldSizeMeters();
      SENSORS_T& sensors = GetSensors();
      SENSORS_ADJ_T& adj = GetAdjacentSensors();
      GridCalculator& gridCalc = GetGridCalculator();
      
      gridCalc.Init(worldSize.width, worldSize.height, _separation);
      adj.resize(sensors.size());
      
      int32 row = 0, col = 0;
      for(int idx = 0; idx < sensors.size(); idx++)
      {
         row = gridCalc.CalcRow(idx);
         col = gridCalc.CalcCol(idx);
         // Since this is a "grid", we will look at the
         // 8 cardinal directions as adjacent.
         int32 calcIdx;
         
         typedef struct RCOFFSET_T
         {
            int32 rOff;
            int32 cOff;
         } RCOFFSET;
         
         static RCOFFSET rcOffs[] =
         {
            // Cardinal Directions
            {  0,    1  },
            {  1,    1  },
            {  1,    0  },
            {  1,    -1 },
            {  0,    -1 },
            {  -1,   1  },
            {  -1,   0  },
            {  -1,   -1 },
         };
         
         const  int32 rcOffMax = sizeof(rcOffs)/sizeof(rcOffs[0]);
         
         for(int32 offIdx = 0; offIdx < rcOffMax; offIdx++)
         {
            calcIdx = gridCalc.CalcIndex(row + rcOffs[offIdx].rOff, col + rcOffs[offIdx].cOff);
            if(calcIdx >= 0 && calcIdx < sensors.size())
            {
               adj[idx].push_back(calcIdx);
            }
         }
      }
   }
   
   
public:
   GraphSensorGridSquareSensors(b2World* world, float32 diameter, float32 separation) :
   _world(world),
   _diameter(diameter),
   _separation(separation)
   {
      assert(_diameter > 0.001);
      assert(_separation > 0.001);
   }
   
};

#endif /* defined(__GraphSensorGridSquareSensors__) */
