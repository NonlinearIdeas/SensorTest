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
   
   
   Notifier::Instance().Attach(this, NE_VIEWPORT_CHANGED);
   Notifier::Instance().Attach(this, NE_RESET_DRAW_CYCLE);
   Notifier::Instance().Attach(this, NE_DEBUG_LINE_DRAW_ADD_LINE);
   Notifier::Instance().Attach(this, NE_DEBUG_TOGGLE_VISIBILITY);
   
   return true;
}

void DebugLinesLayer::ViewportChanged()
{
   _renderTexture->clear(0.0, 0, 0, 0.0);
   _lineMetersDataToDraw.clear();
   _lineMetersDataToDraw.insert(_lineMetersDataToDraw.end(), _lineMetersData.begin(),_lineMetersData.end());
}

void DebugLinesLayer::AddLine(const LINE_METERS_DATA_T& lmd)
{
   if(_enabled)
   {
      _lineMetersDataToDraw.push_back(lmd);
      _lineMetersData.push_back(lmd);
   }
}

void DebugLinesLayer::Reset()
{
   _lineMetersData.clear();
   _lineMetersDataToDraw.clear();
   _renderTexture->clear(0.0, 0, 0, 0.0);
   _enabled = true;
}


void DebugLinesLayer::draw()
{
   CCLayer::draw();
   if(_lineMetersDataToDraw.size() > 0 && _enabled)
   {
      list<LINE_METERS_DATA>::iterator iter;
      _renderTexture->begin();
      Viewport& vp = Viewport::Instance();
      
      for(list<LINE_METERS_DATA>::iterator iter = _lineMetersDataToDraw.begin();
          iter != _lineMetersDataToDraw.end();
          ++iter)
      {
         CCPoint start = vp.Convert(iter->start);
         CCPoint end = vp.Convert(iter->end);
         ccDrawColor4F(iter->color.r, iter->color.g, iter->color.b, iter->color.a);
         if(iter->markerRadius > 0.0)
         {
            ccDrawCircle(start, iter->markerRadius, 0, 20, false);
            ccDrawCircle(end, iter->markerRadius, 0, 20, false);
         }
         ccDrawLine(start,end);
      }
      _renderTexture->end();
      _lineMetersDataToDraw.clear();
   }
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
      case NE_VIEWPORT_CHANGED:
         ViewportChanged();
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
