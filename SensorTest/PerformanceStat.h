/********************************************************************
 * File   : PerformanceStat.h
 * Project: Multiple
 *
 ********************************************************************
 * Created on 3/4/14 By Nonlinear Ideas Inc.
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

#ifndef __PerformanceStat__
#define __PerformanceStat__

#include "Stopwatch.h"

class PerformanceStat
{
private:
   StopWatch _stopWatch;
   unsigned int _samples;
   double _totalSeconds;
   double _value;
public:
	PerformanceStat()
   {
      Reset();
   }
   
	virtual ~PerformanceStat()
   {
      
   }
   
   inline void Reset()
   {
      _stopWatch.Stop();
      _samples = 0;
      _totalSeconds = 0.0;
      _value = 0.0;
   }
   
   inline void Start()
   {
      _stopWatch.Start();
   }
   
   inline void Stop()
   {
      _stopWatch.Stop();
      _totalSeconds += _stopWatch.GetSeconds();
      ++_samples;
   }
   
   inline void Stop(double value)
   {
      _stopWatch.Stop();
      _totalSeconds += _stopWatch.GetSeconds();
      ++_samples;
      _value += value;
   }
   
   inline double GetTotalSeconds() { return _totalSeconds; }
   inline unsigned int GetSamples() { return _samples; }
   inline double GetValue() { return _value; }
   inline double GetAverageSeconds() { return _samples > 0?_totalSeconds/_samples:0.0; }
   inline double GetAverageValue() { return _samples > 0?_value/_samples:0.0; }
};

#endif /* defined(__PerformanceStat__) */
