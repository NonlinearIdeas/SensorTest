/********************************************************************
 * File   : ViewportCamera.cpp
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


#include "ViewportCamera.h"
#include "Viewport.h"
#include "MathUtilities.h"

#define SCALE_STEPS_START (TICKS_PER_SECOND/2)
#define SCALE_MIN 0.0625
#define SCALE_MAX 2.0


ViewportCamera::ViewportCamera() :
   _trackingMode(TM_OFF),
   _trackingPercentage(0.2f),
   _scalingEnabled(false)
{
	
}

ViewportCamera::~ViewportCamera()
{
	
}

void ViewportCamera::PinchBegin(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1)
{
   _viewportCenterOrg = Viewport::Instance().GetCenterMeters();
   _viewportScaleOrg = Viewport::Instance().GetScale();
   _pinchPoint0 = point0.pos;
   _pinchPoint1 = point1.pos;
   PinchViewport(_pinchPoint0, _pinchPoint1, point0.pos, point1.pos);
}

void ViewportCamera::PinchContinue(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1)
{
   PinchViewport(_pinchPoint0, _pinchPoint1, point0.pos, point1.pos);
}

void ViewportCamera::PinchEnd(const TOUCH_DATA_T& point0, const TOUCH_DATA_T& point1)
{
   PinchViewport(_pinchPoint0, _pinchPoint1, point0.pos, point1.pos);   
}

void ViewportCamera::PinchViewport(const CCPoint& p0Org,const CCPoint& p1Org,
                              const CCPoint& p0,const CCPoint& p1)
{
   Viewport& vp = Viewport::Instance();
   float32 distOrg = ccpDistance(p0Org, p1Org);
   float32 distNew = ccpDistance(p0, p1);
   
   if(distOrg < 1)
      distOrg = 1;
   if(distNew < 1)
      distNew = 1;
   
   float32 scaleAdjust = distNew/distOrg;
   Vec2 centerOld = vp.Convert(ccpMidpoint(p0Org, p1Org));
   Vec2 centerNew = vp.Convert(ccpMidpoint(p0, p1));
   
   vp.SetCenter(_viewportCenterOrg-centerNew+centerOld);
   vp.SetScale(scaleAdjust*_viewportScaleOrg);
}

void ViewportCamera::SetTrackingMode(TRACKING_MODE trackingMode)
{
   switch(trackingMode)
   {
      case TM_OFF:
      case TM_FOLLOW_EDGE:
      case TM_JUMP:
         _trackingMode = trackingMode;
         break;
      default:
         assert(false);
         break;
   }
}

void ViewportCamera::SetTrackingModePercentage(float32 trackingPercentage)
{
   assert(trackingPercentage <= 0.5);
   assert(trackingPercentage >= 0.0);
   if(trackingPercentage <= 0.5 && trackingPercentage >= 0.0)
   {
      _trackingPercentage = trackingPercentage;
   }
}

void ViewportCamera::UpdateTracking()
{
   if(_trackingMode == TM_OFF)
      return;
   
   Vec2& position = _trackingPosition;
   Viewport& vp = Viewport::Instance();
   Vec2 vBotLeft = vp.GetBottomLeftMeters();
   Vec2 vTopRight = vp.GetTopRightMeters();
   Vec2 vCenter = vp.GetCenterMeters();
   float32 percent = _trackingPercentage;
   
   
   float32 leftEdge = MathUtilities::LinearTween(percent, vBotLeft.x, vTopRight.x);
   float32 rightEdge = MathUtilities::LinearTween(1-percent, vBotLeft.x, vTopRight.x);
   float32 topEdge = MathUtilities::LinearTween(1-percent, vBotLeft.y, vTopRight.y);
   float32 botEdge = MathUtilities::LinearTween(percent, vBotLeft.y, vTopRight.y);
   bool needsUpdate = false;
   
   if(position.x < leftEdge)
   {
      needsUpdate = true;
      vCenter.x -= (leftEdge-position.x);
   }
   if(position.x > rightEdge)
   {
      needsUpdate = true;
      vCenter.x += (position.x-rightEdge);
   }
   if(position.y < botEdge)
   {
      needsUpdate = true;
      vCenter.y -= (botEdge-position.y);
   }
   if(position.y > topEdge)
   {
      needsUpdate = true;
      vCenter.y += position.y-topEdge;
   }
   
   if(needsUpdate)
   {
      switch(_trackingMode)
      {
         case TM_FOLLOW_EDGE:
            vp.SetCenter(vCenter);
            break;
         case TM_JUMP:
            vp.SetCenter(position);
            break;
         case TM_OFF:
            break;
         default:
            assert(false);
            break;
      }
   }
}

/* Update the viewport to track a position.  A percentage value is
 * supplied with the call.  This is the percent of the viewport, from
 * any side, that the point must be in.  The range is [0,0.5].
 */
void ViewportCamera::UpdateTrackingPosition(const Vec2& position)
{
   _trackingPosition = position;
}

void ViewportCamera::Update()
{
   UpdateTracking();
   UpdateZooming();
}

void ViewportCamera::UpdateZooming()
{
   if(!_scalingEnabled)
      return;
   
   if(_scaleStepsLeft > 0)
   {
      _scaleStepsLeft--;
      float32 time = 1.0 - (1.0*_scaleStepsLeft)/SCALE_STEPS_START;
      float32 scale = MathUtilities::LinearTween(time, _scaleStart, _scaleTarget);
      Viewport::Instance().SetScale(scale);
   }
   else
   {
      _scalingEnabled = false;
   }
}

void ViewportCamera::ZoomViewport(float32 scaleTarget)
{
   if(!_scalingEnabled && scaleTarget >= SCALE_MIN && scaleTarget <= SCALE_MAX)
   {
      _scaleTarget = scaleTarget;
      _scaleStart = Viewport::Instance().GetScale();
      _scalingEnabled = true;
      _scaleStepsLeft = SCALE_STEPS_START;
   }
   else
   {
      _scalingEnabled = false;
   }
}
