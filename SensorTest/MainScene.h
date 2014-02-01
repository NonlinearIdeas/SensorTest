/********************************************************************
 * File   : MainScene.h
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

#ifndef __MainScene__
#define __MainScene__

#include "CommonProject.h"
#include "CommonSTL.h"

#include "DebugLinesLayer.h"
#include "TapDragPinchInput.h"
#include "Notifier.h"
#include "ViewportCamera.h"
#include "MovingEntity.h"
#include "SpriteBatchLayer.h"
#include "Asteroid.h"
#include "Spaceship.h"

class MovingEntityIFace;

class MainScene : public CCScene, public TapDragPinchInputTarget, public Notified
{
private:
   // This class follows the "create"/"autorelease" pattern.
   // Private constructor.
   MainScene();
   ViewportCamera _camera;

   // Box2d Physics World
   b2World* _world;
   Spaceship* _entity;
   SpriteBatchLayer* _asteroidLayer;
   SpriteBatchLayer* _shipLayer;
   vector<Asteroid*> _asteroids;
   b2Body* _anchor;

   
protected:
   // This is protected so that derived classes can call it
   // in their create methods.
   bool init();
   
private:
   void InitSystem();
   void CreatePhysics();
   void CreateSensors();
   void CreateEntity();
   void CreateAsteroids();
   void CreateAnchor();
   void SetZoom(float zoom);
   void UpdateEntity();
   void UpdatePhysics();
   void UpdateAsteroids();
   void ViewportChanged();
public:
   
   static MainScene* create();
   
   ~MainScene();
   
   virtual void onEnter();
   virtual void onExit();
   virtual void onEnterTransitionDidFinish();
   virtual void onExitTransitionDidStart();
   virtual void update(float dt);
      
   // Handler for Tap/Drag/Pinch Events
   virtual void TapDragPinchInputTap(const TOUCH_DATA_T& point);
   virtual void TapDragPinchInputLongTap(const TOUCH_DATA_T& point);
   virtual void TapDragPinchInputPinchBegin(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1);
   virtual void TapDragPinchInputPinchContinue(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1);
   virtual void TapDragPinchInputPinchEnd(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1);
   virtual void TapDragPinchInputDragBegin(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1);
   virtual void TapDragPinchInputDragContinue(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1);
   virtual void TapDragPinchInputDragEnd(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1);
   
   // Get notified about cetain events
   virtual bool Notify(NOTIFIED_EVENT_TYPE_T eventType, const bool& value);
};


#endif /* defined(__MainScene__) */
