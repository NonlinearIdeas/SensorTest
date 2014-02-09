/********************************************************************
 * File   : MainScene.cpp
 * Project: ToolsDemo
 *
 ********************************************************************
 * Created on 9/21/13 By Nonlinear Ideas Inc.
 * Copyright (c) 2013 Nonlinear Ideas Inc. All rights reserved.
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

#include "MainScene.h"
#include "Box2DDebugDrawLayer.h"
#include "GridLayer.h"
#include "DebugLinesLayer.h"
#include "TapDragPinchInput.h"
#include "Notifier.h"
#include "Viewport.h"
#include "Entity.h"
#include "EntityManager.h"
#include "GraphSensorGenerator.h"
#include "GraphSensorManager.h"
#include "SystemContactListener.h"
#include "GraphSensorContactLayer.h"
#include "Box2DShapeCache.h"
#include "SunBackgroundLayer.h"
#include "SpriteBatchLayer.h"
#include "EntityUtilities.h"
#include "EntityScheduler.h"
#include "Spaceship.h"

#include "GraphCommon.h"

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

      // NOTE:  These are REFERENCES to the underlying values.
      // THESE ARE NOT LOCAL VARIABLES.
      int32& cols = Cols();
      int32& rows = Rows();
      
      // Calculate the rows and columns so that there is at least one gap between each
      // shape.
      cols = (uint32)ceil(worldSize.width/_separation);
      rows = (uint32)ceil(worldSize.height/_separation);
      sensors.clear();
      
      if(cols%2 != 0)
         cols++;
      if(rows%2 != 0)
         rows++;
      
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
      
      //CCLOG("Rows = %d, Cols = %d",rows,cols);
      
      for(int32 row = 0; row <= rows; ++row)
      {
         for(int32 col = 0; col <= cols; ++col)
         {
            b2Vec2 pos((col-cols/2) * _separation, (row-rows/2) * _separation);
            bodyDef.position = pos;
            
            body = _world->CreateBody(&bodyDef);
            fixture = body->CreateFixture(&fixtureDef);
            
            sensor = new GraphSensor();
            sensor->SetBody(body);
            sensor->SetIndex(sensors.size());
            
            // We don't need to draw these in box2d debug
            body->SetDebugDraw(false);
            
            sensors.push_back(sensor);
            
            /* Just a little unit testing */
            /*
            int32 calcIdx, calcRow, calcCol;
            
            calcIdx = CalcIndex(row, col);
            CalcIndex(calcIdx,calcRow,calcCol);
            
             CCLOG("AIdx = %04ld, Index = %04d, agrid(%d,%d), grid(%d,%d)",
             sensors.size()-1,
             calcIdx,
             row,
             col,
             calcRow,
             calcCol
             );
            assert(calcIdx == sensors.size()-1);
            assert(row == calcRow);
            assert(col == calcCol);
             */
         }
      }
   }
   
   virtual void GenerateAdjacency()
   {
      SENSORS_ADJ_T& adj = GetAdjacentSensors();
      adj.resize(GetSensorsConst().size());
      const SENSORS_T& sensors = GetSensorsConst();
      int32 row = 0, col = 0;
      for(int idx = 0; idx < sensors.size(); idx++)
      {
         // Get the row, col for this index.
         CalcIndex(idx, row,col);
         // Since this is a "grid", we will look at the
         // 8 cardinal directions as adjacent.
         int32 calcIdx;
         
         calcIdx = CalcIndex(row+1, col+1);
         if(calcIdx >= 0 && calcIdx < sensors.size()) {  adj[idx].push_back(calcIdx); }
         
         calcIdx = CalcIndex(row+1, col);
         if(calcIdx >= 0 && calcIdx < sensors.size()) {  adj[idx].push_back(calcIdx); }
         
         calcIdx = CalcIndex(row+1, col-1);
         if(calcIdx >= 0 && calcIdx < sensors.size()) {  adj[idx].push_back(calcIdx); }
         
         calcIdx = CalcIndex(row, col+1);
         if(calcIdx >= 0 && calcIdx < sensors.size()) {  adj[idx].push_back(calcIdx); }
         
         calcIdx = CalcIndex(row, col-1);
         if(calcIdx >= 0 && calcIdx < sensors.size()) {  adj[idx].push_back(calcIdx); }
         
         calcIdx = CalcIndex(row-1, col+1);
         if(calcIdx >= 0 && calcIdx < sensors.size()) {  adj[idx].push_back(calcIdx); }
         
         calcIdx = CalcIndex(row-1, col);
         if(calcIdx >= 0 && calcIdx < sensors.size()) {  adj[idx].push_back(calcIdx); }
         
         calcIdx = CalcIndex(row-1, col-1);
         if(calcIdx >= 0 && calcIdx < sensors.size()) {  adj[idx].push_back(calcIdx); }
      }
   }
   
   virtual int32 CalcIndex(int32 row, int32 col)
   {
      return row * (Cols() + 1) + col;
   }
   
   virtual void CalcIndex(int32 idx, int32& outRow, int32& outCol)
   {
      outRow = idx / (Cols() + 1);
      outCol = idx % (Cols() + 1);
   }

   
public:
   GraphSensorGridSquareSensors(b2World* world, float32 diameter = 1.0f, float32 separation = 2.5f) :
   _world(world),
   _diameter(diameter),
   _separation(separation)
   {
      assert(_diameter > 0.001);
      assert(_separation > 0.001);
   }
   
};


MainScene::MainScene() :
   _entity(NULL)
{
}

MainScene::~MainScene()
{
}

void MainScene::CreateEntity()
{
   CCSize wSize = Viewport::Instance().GetWorldSizeMeters();
   Vec2 position(wSize.width/2,wSize.height/2);
   _entity = new Spaceship();
   _entity->Create(*_world,position, 0);
   _shipLayer->AddSprite(_entity->GetSprite());
   EntityManager::Instance().Register(_entity);
}

void MainScene::InitSystem()
{
   // Set up the viewport
   static const float32 worldSizeMeters = 100.0;
   
   // Initialize the Viewport
   Viewport::Instance().Init(worldSizeMeters);
   Viewport::Instance().SetScale(1.4f);
   
   EntityManager::Instance().Init();
   GraphSensorManager::Instance().Init();
   SystemContactListener::Instance().Init();
   EntityScheduler::Instance().Init();
   
   
   Box2DShapeCache::instance().addShapesWithFile("Asteroids.plist");
   Box2DShapeCache::instance().addShapesWithFile("Entity.plist");
   
   
}

void MainScene::CreatePhysics()
{   
   _world = new b2World(Vec2(0.0,0.0));
   // Do we want to let bodies sleep?
   // No for now...makes the debug layer blink
   // which is annoying.
   _world->SetAllowSleeping(false);
   _world->SetContinuousPhysics(true);
   
   // Set the contact callback.
   _world->SetContactListener(&SystemContactListener::Instance());
}

void MainScene::CreateSensors()
{
   GraphSensorGridSquareSensors sensorGrid(_world);
   sensorGrid.Create();
   
   GraphSensorManager::Instance().LoadSensors(sensorGrid);
}

bool MainScene::init()
{
   InitSystem();
   return true;
}

MainScene* MainScene::create()
{
   MainScene *pRet = new MainScene();
   if (pRet && pRet->init())
   {
      pRet->autorelease();
      return pRet;
   }
   else
   {
      CC_SAFE_DELETE(pRet);
      return NULL;
   }
}

void MainScene::onEnter()
{
   CCScene::onEnter();
   // Create physical world
   CreatePhysics();
   
   // Add a color background.  This will make it easier on the eyes.
   addChild(SunBackgroundLayer::create());
   
   // Adding the debug lines so that we can draw the path followed.
   addChild(DebugLinesLayer::create());
   
   // Touch Input
   addChild(TapDragPinchInput::create(this));
   
   // Grid
   addChild(GridLayer::create());

   // Box2d Debug
   //addChild(Box2DDebugDrawLayer::create(_world));
   
   // Asteroids
   _asteroidLayer = SpriteBatchLayer::create("Asteroids_ImgData.png", "Asteroids_ImgData.plist");
   assert(_asteroidLayer != NULL);
   addChild(_asteroidLayer);

   // Space Ships
   _shipLayer = SpriteBatchLayer::create("EntitySpriteImages.png", "EntitySpriteImages.plist");
   assert(_shipLayer != NULL);
   addChild(_shipLayer);
   
   // Create the Anchor
   CreateAnchor();
   
   // Asteroids
   CreateAsteroids();
   
   // Populate physical world
   CreateEntity();
   
   // Create the sensor grid
   CreateSensors();
   
   // Contact Counts
   //addChild(GraphSensorContactLayer::create());
   
   // Register for events
   Notifier::Instance().Attach(this, NE_VIEWPORT_CHANGED);
   
   // Kickstart all sizes
   ViewportChanged();
}

void MainScene::onExit()
{
   CCScene::onExit();
}

void MainScene::onEnterTransitionDidFinish()
{
   // Schedule Updates
   scheduleUpdate();
}

void MainScene::onExitTransitionDidStart()
{
   CCScene::onExitTransitionDidStart();
   
   // Turn off updates
   unscheduleUpdate();
}


void MainScene::UpdateEntity()
{
   if(_entity != NULL)
   {
      _entity->Update();
   }
}

void MainScene::UpdatePhysics()
{
   const int velocityIterations = 8;
   const int positionIterations = 1;
   float32 fixedDT = SECONDS_PER_TICK;
   // Instruct the world to perform a single step of simulation. It is
   // generally best to keep the time step and iterations fixed.
   _world->Step(fixedDT, velocityIterations, positionIterations);
   
   SystemContactListener::Instance().NotifyContacts();
}

void MainScene::update(float dt)
{
   UpdatePhysics();
   UpdateEntity();
   EntityScheduler::Instance().Update();
   //   UpdateAsteroids();
}




// Handler for Tap/Drag/Pinch Events
void MainScene::TapDragPinchInputTap(const TOUCH_DATA_T& point)
{
   _entity->CommandSeek(Viewport::Instance().Convert(point.pos));
   TestBFS();
}
void MainScene::TapDragPinchInputLongTap(const TOUCH_DATA_T& point)
{
}
void MainScene::TapDragPinchInputPinchBegin(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1)
{
   _entity->CommandIdle();
   _camera.PinchBegin(point0, point1);
}
void MainScene::TapDragPinchInputPinchContinue(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1)
{
   _camera.PinchContinue(point0, point1);
}
void MainScene::TapDragPinchInputPinchEnd(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1)
{
   _camera.PinchEnd(point0, point1);
}
void MainScene::TapDragPinchInputDragBegin(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1)
{
   _entity->CommandSeek(Viewport::Instance().Convert(point0.pos));
}
void MainScene::TapDragPinchInputDragContinue(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1)
{
   _entity->CommandSeek(Viewport::Instance().Convert(point1.pos));
   Notifier::Instance().Notify(NE_RESET_DRAW_CYCLE);
}
void MainScene::TapDragPinchInputDragEnd(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1)
{
   Notifier::Instance().Notify(NE_RESET_DRAW_CYCLE);
}

bool MainScene::Notify(NOTIFIED_EVENT_TYPE_T eventType, const bool& value)
{
   bool result =  true;
   switch(eventType)
   {
      case NE_VIEWPORT_CHANGED:
         ViewportChanged();
         break;
      default:
         result = false;
         break;
   }
   return result;
}


void MainScene::SetZoom(float scale)
{
   Viewport::Instance().SetScale(scale);
}

void MainScene::CreateAsteroids()
{
   Vec2 center(0,0);
   
   const string names[] =
   {
      "Asteroid_01",
      "Asteroid_02",
      "Asteroid_03",
      "Asteroid_04",
      "Asteroid_05",
      "Asteroid_06",
      "Asteroid_07",
      //       "Asteroid_08",
   };
   
   const int MAX_NAMES = sizeof(names)/sizeof(names[0]);
   
   typedef struct
   {
      float32 radius;
      uint32 asteroids;
   } RING_DATA_T;
   
   RING_DATA_T ringData[] =
   {
      {1, 1},
      {9, 3},
      {17, 7},
      {25, 13},
      {35, 20},
   };
   
   const int MAX_RINGS = sizeof(ringData)/sizeof(ringData[0]);
   
   float32 angleRads = 0;
   uint32 nameIdx = 0;
   
   
   for(int ring = 0; ring < MAX_RINGS; ring++)
   {
      // Inner ring asteroids
      float32 targetRadius = ringData[ring].radius;
      uint32 asteroids = ringData[ring].asteroids;
      
      for(int idx = 0; idx < asteroids; idx++)
      {
         Vec2 offset =  Vec2::FromPolar(targetRadius, angleRads);
         Vec2 position = center + offset;
         Asteroid* asteroid = new Asteroid();
         asteroid->Create(*_world,
                          names[nameIdx%MAX_NAMES],
                          position,
                          9.0 + RanNumGen::RandFloat(-1.0, 1.0));
         
         asteroid->GetBody()->SetDebugDraw(false);
         
         EntityManager::Instance().Register(asteroid);
         _asteroids.push_back(asteroid);
         
         
         // All asteroids have a distance joint to the anchor
         // Now create the joint.
         b2RopeJointDef jointDef;
         jointDef.bodyA = _anchor;
         jointDef.bodyB = asteroid->GetBody();
         jointDef.maxLength = b2Distance(jointDef.bodyA->GetPosition(), jointDef.bodyB->GetPosition());
         jointDef.collideConnected = true;
         _world->CreateJoint(&jointDef);
         
         nameIdx++;
         angleRads += (2*M_PI)/asteroids + RanNumGen::RandFloat(-M_PI/(12*asteroids), M_PI/(12*asteroids));
      }
   }
   
   
   for(int idx = 0;idx < _asteroids.size(); idx++)
   {
      _asteroidLayer->AddSprite(_asteroids[idx]->GetSprite());
      EntityScheduler::Instance().Register(_asteroids[idx]);
      // Give it at least one update to start.
      _asteroids[idx]->Update();
   }
}


void MainScene::UpdateAsteroids()
{
   for(int idx = 0; idx < _asteroids.size(); idx++)
   {
      _asteroids[idx]->Update();
      _asteroids[idx]->UpdateDisplay();
   }
}

void MainScene::CreateAnchor()
{
   b2BodyDef bodyDef;
   bodyDef.position = Vec2(0.0f,0.0f);
   bodyDef.type = b2_staticBody;
   _anchor = _world->CreateBody(&bodyDef);
   assert(_anchor != NULL);
   
   b2CircleShape circle;
   b2FixtureDef fixtureDef;
   circle.m_radius = 0.25;
   fixtureDef.shape = &circle;
   fixtureDef.density = 1.0;
   fixtureDef.friction = 1.0;
   fixtureDef.isSensor = true;
   _anchor->CreateFixture(&fixtureDef);
   _anchor->SetUserData(NULL);
}

void MainScene::ViewportChanged()
{
   for(int idx = 0; idx < _asteroids.size(); idx++)
   {
      Asteroid* asteroid = _asteroids[idx];
      EntityUtilities::AdjustNodeScale(asteroid->GetSprite(),asteroid,1.01,Viewport::Instance().GetPTMRatio());
   }
   
   EntityUtilities::AdjustNodeScale(_entity->GetSprite(), _entity, 1.01, Viewport::Instance().GetPTMRatio());
}
