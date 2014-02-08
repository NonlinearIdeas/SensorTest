/********************************************************************
 * File   : CommonSTL.h
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

#ifndef _CommonSTL_h
#define _CommonSTL_h

#include <iostream>
#include <string>
#include <list>
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <stack>
#include <algorithm>
#include <cassert>
#include <sstream>
#include <stdexcept>
#include <ostream>
#include <fstream>

using namespace std;

typedef signed char int8;
typedef signed short int16;
typedef signed int int32;
typedef signed long long int64;

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef unsigned long long uint64;

typedef double float64;

#define Min(a,b) (a<b?a:b)
#define Max(a,b) (a<b?b:a)


#endif // _CommonSTL_h
