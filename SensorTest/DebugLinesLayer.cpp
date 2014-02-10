/********************************************************************
 * File   : DebugLinesLayer.cpp
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
#include "DebugLinesLayer.h"
#include "Viewport.h"

DebugLinesLayer::DebugLinesLayer()
{
   
}

DebugLinesLayer::~DebugLinesLayer()
{
}

bool DebugLinesLayer::init()
{
   if(!CCLayer::init())
      return false;
   CCSize scrSize = CCDirector::sharedDirector()->getWinSize();
   _renderTexture = CCRenderTexture::create(scrSize.width, scrSize.height, kCCTexture2DPixelFormat_RGBA8888);
   _renderTexture->setPosition(ccp(scrSize.width/2,scrSize.height/2));
   addChild(_renderTexture);
   
   Reset();
   
   
   Notifier::Instance().Attach(this, NE_RESET_DRAW_CYCLE);
   Notifier::Instance().Attach(this, NE_DEBUG_LINE_DRAW_ADD_LINE);
   Notifier::Instance().Attach(this, NE_DEBUG_TOGGLE_VISIBILITY);
   
   return true;
}

void DebugLinesLayer::AddLine(const LINE_METERS_DATA_T& lmd)
{
   LINE_PIXELS_DATA_T lpd;
   
   lpd.start = Viewport::Instance().Convert(lmd.start);
   lpd.end = Viewport::Instance().Convert(lmd.end);
   
   _lineData.push_back(lpd);
}

void DebugLinesLayer::AddLine(const LINE_PIXELS_DATA_T& lpd)
{
   _lineData.push_back(lpd);
}

void DebugLinesLayer::Reset()
{
   _lineData.clear();
   _renderTexture->clear(0.0, 0, 0, 0.0);
   _enabled = true;
}


void DebugLinesLayer::draw()
{
   CCLayer::draw();
   if(_enabled)
   {
      _renderTexture->begin();
      for(int idx = 0; idx < _lineData.size(); idx++)
      {
         LINE_PIXELS_DATA_T& ld = _lineData[idx];
         ccDrawColor4F(ld.color.r, ld.color.g, ld.color.b, ld.color.a);
         if(ld.markerRadius > 0.0)
         {
            ccDrawCircle(ccp(ld.start.x,ld.start.y), ld.markerRadius, 0, 20, false);
         }
         ccDrawLine(ccp(ld.start.x,ld.start.y), ccp(ld.end.x,ld.end.y));
      }
      _lineData.clear();
      _renderTexture->end();
   }
}

bool DebugLinesLayer::Notify(NOTIFIED_EVENT_TYPE_T eventType, const LINE_PIXELS_DATA_T& value)
{
   bool result = true;
   switch(eventType)
   {
      case NE_DEBUG_LINE_DRAW_ADD_LINE:
         AddLine(value);
         break;
      default:
         result = false;
         assert(false);
         break;
   }
   return result;
}

bool DebugLinesLayer::Notify(NOTIFIED_EVENT_TYPE_T eventType, const LINE_METERS_DATA_T& value)
{
   bool result = true;
   switch(eventType)
   {
      case NE_DEBUG_LINE_DRAW_ADD_LINE:
         AddLine(value);
         break;
      default:
         result = false;
         assert(false);
         break;
   }
   return result;
}


bool DebugLinesLayer::Notify(NOTIFIED_EVENT_TYPE_T eventType, const bool& value)
{
   bool result = true;
   switch(eventType)
   {
      case NE_RESET_DRAW_CYCLE:
         Reset();
         break;
      case NE_DEBUG_TOGGLE_VISIBILITY:
         setVisible(!isVisible());
         break;
      default:
         result = false;
         assert(false);
         break;
   }
   return result;
}

DebugLinesLayer* DebugLinesLayer::create(bool createVisible)
{
   DebugLinesLayer *pRet = new DebugLinesLayer();
   if (pRet && pRet->init())
   {
      pRet->autorelease();
      pRet->setVisible(createVisible);
      return pRet;
   }
   else
   {
      CC_SAFE_DELETE(pRet);
      return NULL;
   }
}
