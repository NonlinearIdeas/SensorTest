/********************************************************************
 * File   : GridLayer.h
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

#ifndef __GridLayer__
#define __GridLayer__

#include "CommonSTL.h"
#include "CommonProject.h"
#include "Notifier.h"

class GridLayer : public CCLayer, Notified
{
private:
   // Pixel Space Coordinates
   vector<LINE_METERS_DATA_T> _worldLines;
   vector<LINE_PIXELS_DATA_T> _pixelLines;
   uint32 _subdivisions;
   
   void DrawGridLines();
   void UpdateGridLabels();
   void UpdateGridScale();
   void InitScaleLabel();
   void InitGridLabels();
   void CalculateWorldLines();
   void CalculateGridLines();
protected:
   bool init(uint32 subdivisions);
   
public:
   static GridLayer* create(uint32 subdivisions = 5);
   virtual bool Notify(NOTIFIED_EVENT_TYPE_T eventType, const bool& value);
   virtual void draw();
};

#endif /* defined(__GridLayer__) */
