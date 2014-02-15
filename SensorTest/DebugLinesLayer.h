/********************************************************************
 * File   : DebugLinesLayer.h
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

#ifndef __DebugLinesLayer__
#define __DebugLinesLayer__

#include "CommonSTL.h"
#include "CommonProject.h"
#include "Notifier.h"


class DebugLinesLayer : public CCLayer, public Notified
{
private:
   
   list<LINE_METERS_DATA_T> _lineMetersData;
   list<LINE_METERS_DATA_T> _lineMetersDataToDraw;
   bool _enabled;
   CCRenderTexture* _renderTexture;
   
   bool init();
   DebugLinesLayer();
   ~DebugLinesLayer();
   void ViewportChanged();
   
   
public:
   
   void Reset();
   void SetEnabled(bool enabled) { _enabled = enabled; }
   bool GetEnabled() { return _enabled; }
   void AddLine(const LINE_METERS_DATA_T& lmd);

   virtual void draw();
   virtual bool Notify(NOTIFIED_EVENT_TYPE_T eventType, const LINE_METERS_DATA_T& value);
   virtual bool Notify(NOTIFIED_EVENT_TYPE_T eventType, const bool& value);
   
   static DebugLinesLayer* create(bool createVisible = true);
};

#endif /* defined(__DebugLinesLayer__) */
