/********************************************************************
 * File   : ViewportCamera.h
 * Project: SensorTest
 *
 ********************************************************************
 * Created on 1/18/14 By Nonlinear Ideas Inc.
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

#ifndef __ViewportCamera__
#define __ViewportCamera__

/* This class implements a simple interface for controlling the 
   Viewport like a camera.
 */

#include "CommonProject.h"
#include "CommonSTL.h"

class ViewportCamera
{
public:
   typedef enum
   {
      // No tracking
      TM_OFF,
      // Smoothly follow the position
      // when it gets near the edge.
      TM_FOLLOW_EDGE,
      // Force the center of the viewport
      // to be the new postion.
      TM_JUMP,
   }TRACKING_MODE;

protected:
private:
   void PinchViewport(const CCPoint& p0Org,const CCPoint& p1Org,
                                      const CCPoint& p0,const CCPoint& p1);
   
   void UpdateTracking();
   void UpdateZooming();
   // Keep the last center point during a pinch.
   Vec2 _viewportCenterOrg;
   CCPoint _pinchPoint0;
   CCPoint _pinchPoint1;
   float32 _viewportScaleOrg;
   float32 _trackingPercentage;
   Vec2 _trackingPosition;
   TRACKING_MODE _trackingMode;
   bool _scalingEnabled;
   float32 _scaleTarget;
   float32 _scaleStart;
   uint32  _scaleStepsLeft;
   
public:
   
   
	ViewportCamera();
	virtual ~ViewportCamera();
   void PinchBegin(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1);
   void PinchContinue(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1);
   void PinchEnd(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1);
   
   void SetTrackingMode(TRACKING_MODE trackingMode);
   void SetTrackingModePercentage(float32 trackingPercentage);
   void UpdateTrackingPosition(const Vec2& position);
   inline bool IsTrackingEnabled() { return _trackingMode != TM_OFF; }
   
   void ZoomViewport(float32 scaleTarget);

   // Call this every cycle to update the camera.
   void Update();
};

#endif /* defined(__SensorTest__ViewportCamera__) */
