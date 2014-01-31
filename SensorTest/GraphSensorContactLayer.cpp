/********************************************************************
 * File   : GraphSensorContactLayer.cpp
 * Project: Multiple
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

#include "GraphSensorContactLayer.h"
#include "GraphSensorManager.h"
#include "Viewport.h"

void GraphSensorContactLayer::update(float dt)
{
   UpdateSensorLabels();
}

void GraphSensorContactLayer::InitSensorLabels()
{
   // Clear out anything we may have from the past.
   removeAllChildren();
}

void GraphSensorContactLayer::UpdateSensorLabels()
{
   
}

void GraphSensorContactLayer::onEnterTransitionDidFinish()
{
   scheduleUpdate();
   InitSensorLabels();
}

void GraphSensorContactLayer::onExitTransitionDidStart()
{
   unscheduleUpdate();
}


bool GraphSensorContactLayer::init()
{
   Notifier::Instance().Attach(this, NE_VIEWPORT_CHANGED);
   Notifier::Instance().Attach(this, NE_DEBUG_TOGGLE_VISIBILITY);
   return true;
}

GraphSensorContactLayer* GraphSensorContactLayer::create()
{
   GraphSensorContactLayer *pRet = new GraphSensorContactLayer();
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

bool GraphSensorContactLayer::Notify(NOTIFIED_EVENT_TYPE_T eventType, const bool& value)
{
   bool result = true;
   switch (eventType)
   {
      case NE_VIEWPORT_CHANGED:
         ViewportChanged();
         break;
      case NE_DEBUG_TOGGLE_VISIBILITY:
         setVisible(!isVisible());
         break;
      default:
         assert(false);
         result = false;
         break;
   }
   return result;
}


/*
void GridLayer::InitScaleLabel()
{
   CCSize scrSize = CCDirector::sharedDirector()->getWinSize();
   char buffer[32];
   
   sprintf(buffer,"Scale\n%4.2f",Viewport::Instance().GetScale());
   
   CCLabelBMFont* label = CCLabelBMFont::create(buffer, "Arial_32_Green.fnt",100,kCCTextAlignmentCenter);
   label->setPosition(ccp(0.95f*scrSize.width,0.95f*scrSize.height));
   label->setTag(TAG_LABEL_SCALE);
   addChild(label);
}
*/