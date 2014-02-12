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
   int32 _count;
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
         _cols = (int32)ceil(_width/_separation);
         _rows = (int32)ceil(_height/_separation);
         if(_cols%2 == 0)
            _cols++;
         if(_rows%2 == 0)
            _rows++;
         _count = _rows * _cols;
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
   
   inline int32 GetCols() { return _cols; }
   inline int32 GetRows() { return _rows; }
   inline int32 GetCount() { return _count; }

   
   inline int32 CalcRow(int32 idx)
   {
      int32 result = idx / (_cols);
      if(result < 0)
         result = 0;
      if(result >= _rows)
         result = _rows-1;
      return result;
   }
   
   inline int32 CalcCol(int32 idx)
   {
      int32 result =  idx % (_cols);
      if(result < 0)
         result = 0;
      if(result >= _rows)
         result = _rows -1;
      return result;
   }

   inline int32 CalcIndex(int32 row, int32 col)
   {
      if(row < 0)
         return -1;
      if(col < 0)
         return -1;
      if(row >= _rows)
         return -1;
      if(col >= _cols)
         return -1;
      int32 result = row * (_cols) + col;
      return result;
   }
   
   inline int32 CalcIndex(const Vec2& pos)
   {
      int32 col = (int32)(pos.x+_separation/2)/_separation + _cols/2;
      if(col < 0)
         col = 0;
      if(col >= _cols)
         col = _cols-1;
      int32 row = (int32)(pos.y+_separation/2)/_separation + _rows/2;
      if(row < 0)
         row = 0;
      if(row >= _rows)
         row = _rows-1;
      int32 idx = CalcIndex(row,col);
      /*
      CCLOG("Position (%4.1f,%4.1f) => idx = %d, (%d,%d)",
            pos.x,pos.y,idx,col,row);
       */
      return idx;
   }
   
   inline Vec2 CalcPosition(int32 row, int32 col)
   {
      Vec2 pos;
      pos.x = (col-_cols/2)*_separation;
      pos.y = (row-_rows/2)*_separation;
      return pos;
   }
   
   inline Vec2 CalcPosition(int32 idx)
   {
      int32 row = CalcRow(idx);
      int32 col = CalcCol(idx);
      Vec2 pos = CalcPosition(row, col);
      //      int32 idx2 = CalcIndex(row, col);
      //      CCLOG("Position for idx = %d (%d) (%d,%d) = (%4.1f,%4.1f)",idx,idx2,col,row,pos.x,pos.y);
      return pos;
   }

};

#endif /* defined(__GridCalculator__) */
