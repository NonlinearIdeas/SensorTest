/********************************************************************
 * File   : GridCalculator.h
 * Project: Multiple
 *
 ********************************************************************
 * Created on 2/9/14 By Nonlinear Ideas Inc.
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

#ifndef __GridCalculator__
#define __GridCalculator__

#include "CommonProject.h"
#include "CommonSTL.h"

class GridCalculator
{
private:
   int32 _cols;
   int32 _rows;
   float32 _width;
   float32 _height;
   float32 _separation;
   
public:
   void Init(float32 width, float32 height, float32 separation)
   {
      assert(width > 0.0f);
      assert(height > 0.0f);
      assert(separation > 0.0f);
      if(width > 0.0f &&
         height > 0.0f &&
         separation > 0.0f)
      {
         _width = width;
         _height = height;
         _separation = separation;
         _cols = (uint32)ceil(_width/_separation);
         _rows = (uint32)ceil(_height/_separation);
         if(_cols%2 != 0)
            _cols++;
         if(_rows%2 != 0)
            _rows++;
      }
   }
   
   GridCalculator(float32 width, float32 height, float32 separation)
   {
      Init(width,height,separation);
   }
   
   GridCalculator()
   {
      Init(100.0f,100.0f,1.0f);
   }
   
   int32 CalcIndex(int32 row, int32 col)
   {
      if(row < 0)
         row = 0;
      if(row >= _rows)
         row = _rows -1;
      if(col < 0)
         col = 0;
      if(col >= _cols)
         col = _cols -1;
      int32 result = row * (_cols + 1) + col;
      return result;
   }
   
   int32 CalcRow(int32 idx)
   {
      int32 result = idx / (_cols + 1);
      if(result < 0)
         return 0;
      if(result > _rows-1)
         return _rows-1;
      return result;
   }
   
   int32 CalcCol(int32 idx)
   {
      int32 result =  idx % (_cols + 1);
      if(result < 0)
         return 0;
      if(result > _cols-1)
         return _cols-1;
      return result;
   }
   
   int32 CalcIndex(const Vec2& pos)
   {
      int32 col = (int32)(pos.x/_separation) + _cols/2;
      int32 row = (int32)(pos.y/_separation) + _rows/2;
      return CalcIndex(row, col);
   }
   
   Vec2 CalcPosition(int32 row, int32 col)
   {
      Vec2 pos((col-_cols/2) * _separation, (row-_rows/2) * _separation);
      return pos;
   }
};

#endif /* defined(__GridCalculator__) */
