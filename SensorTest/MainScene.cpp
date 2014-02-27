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
#include "GraphSensorGridSquareSensors.h"
#include "PlayerGameControlsLayer.h"

#define SENSOR_DIAMETER (2.4f)
#define SENSOR_SEPARATION (2.5f)
#define TRACK_ENTITY_CELL_INDEX
#define TAG_DEBUG_BOX2D (1000)
#define TAG_DEBUG_GRID (1001)
#define TAG_DEBUG_LINES (1002)


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
   GraphSensorGridSquareSensors sensorGrid(_world,SENSOR_DIAMETER,SENSOR_SEPARATION);
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
   DebugLinesLayer* linesLayer = DebugLinesLayer::create();
   assert(linesLayer != NULL);
   addChild(DebugLinesLayer::create());
   linesLayer->setTag(TAG_DEBUG_LINES);
   addChild(linesLayer);
   
   // Grid
   // This should be at the bottom of the layer stack.
   GridLayer* gridLayer = GridLayer::create(5);
   assert(gridLayer != NULL);
   gridLayer->setTag(TAG_DEBUG_GRID);
   addChild(gridLayer);
   
   // Box2d Debug
   Box2DDebugDrawLayer* debugLayer = Box2DDebugDrawLayer::create(_world);
   assert(debugLayer != NULL);
   debugLayer->setTag(TAG_DEBUG_BOX2D);
   addChild(debugLayer);
   
   // Asteroids
   _asteroidLayer = SpriteBatchLayer::create("Asteroids_ImgData.png", "Asteroids_ImgData.plist");
   assert(_asteroidLayer != NULL);
   addChild(_asteroidLayer);

   // Space Ships
   _shipLayer = SpriteBatchLayer::create("EntitySpriteImages.png", "EntitySpriteImages.plist");
   assert(_shipLayer != NULL);
   addChild(_shipLayer);
   
   
   // Touch Input
   addChild(TapDragPinchInput::create(this));

   // User Controls
   addChild(PlayerGameControlsLayer::create());
   
   
   // Create the Anchor
   CreateAnchor();
   
   // Asteroids
   CreateAsteroids();
   
   // Populate physical world
   CreateEntity();
   
   // Create the sensor grid
   CreateSensors();
   
   // Contact Counts
   addChild(GraphSensorContactLayer::create());
   
   // Register for events
   Notifier::Instance().Attach(this, NE_VIEWPORT_CHANGED);
   Notifier::Instance().Attach(this, NE_GAME_COMMAND_ZOOM_IN);
   Notifier::Instance().Attach(this, NE_GAME_COMMAND_ZOOM_OUT);
   Notifier::Instance().Attach(this, NE_GAME_COMMAND_TRACK);
   
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
#ifdef TRACK_ENTITY_CELL_INDEX
   static int32 entCellIdxLast = -1;
   int32 entCellIdx = GraphSensorManager::Instance().GetGridCalculator().CalcIndex(_entity->GetBody()->GetWorldCenter());
   if(entCellIdx != entCellIdxLast)
   {
      CCLOG("Entity now in cell %d.",entCellIdx);
      entCellIdxLast = entCellIdx;
   }
#endif
}


void MainScene::NavigateToPosition(Vec2 pos)
{
   _entity->CommandNavigateToPoint(pos);
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
   // Update all the debug info
   Notifier::Instance().Notify(NE_UPDATE_DEBUG_INFO,true);
   // Clear out any old information
   GraphSensorManager::Instance().ClearChangedSensors();
}




// Handler for Tap/Drag/Pinch Events
void MainScene::TapDragPinchInputTap(const TOUCH_DATA_T& point)
{
   NavigateToPosition(Viewport::Instance().Convert(point.pos));
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
}
void MainScene::TapDragPinchInputDragContinue(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1)
{
}
void MainScene::TapDragPinchInputDragEnd(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1)
{
}

bool MainScene::Notify(NOTIFIED_EVENT_TYPE_T eventType, const bool& value)
{
   bool result =  true;
   switch(eventType)
   {
      case NE_VIEWPORT_CHANGED:
         ViewportChanged();
         break;
      case NE_GAME_COMMAND_ZOOM_IN:
         break;
      case NE_GAME_COMMAND_ZOOM_OUT:
         break;
      case NE_GAME_COMMAND_TRACK:
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
