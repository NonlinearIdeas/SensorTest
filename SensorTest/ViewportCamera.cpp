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


ViewportCamera::ViewportCamera()
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
