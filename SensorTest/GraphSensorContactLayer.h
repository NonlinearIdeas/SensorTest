/********************************************************************
 * File   : GraphSensorContactLayer.h
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

#ifndef __GraphSensorContactLayer__
#define __GraphSensorContactLayer__

#include "CommonSTL.h"
#include "CommonProject.h"
#include "Notifier.h"

class GraphSensorContactLayer : public CCLayer, Notified
{
private:
   void UpdateSensorLabels();
   void InitSensorLabels();
   void ViewportChanged();
   
protected:
   bool init();
   
public:
   static GraphSensorContactLayer* create();
   virtual bool Notify(NOTIFIED_EVENT_TYPE_T eventType, const bool& value);
   virtual void draw();
   virtual void update(float dt);
   virtual void onEnterTransitionDidFinish();
   virtual void onExitTransitionDidStart();
};

#endif /* defined(__GraphSensorContactLayer__) */
