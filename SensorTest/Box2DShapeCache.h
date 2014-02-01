//
//  Box2DShapeCache.h
//  Box2DTestBed
//
//  Created by James Wucher on 8/3/13.
//
//

#ifndef __Box2DTestBed__Box2DShapeCache__
#define __Box2DTestBed__Box2DShapeCache__

//
//  GB2ShapeCache-x.cpp
//
//  Loads physics sprites created with http://www.PhysicsEditor.de
//  To be used with cocos2d-x
//
//  Generic Shape Cache for box2d
//
//  Created by Thomas Broquist
//
//      http://www.PhysicsEditor.de
//      http://texturepacker.com
//      http://www.code-and-web.de
//
//  All rights reserved.
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//  THE SOFTWARE.
//


#include "CommonSTL.h"
#include "CommonProject.h"
class BodyDef;

namespace cocos2d
{
	class Box2DShapeCache
   {
	public:
		// Static interface
		static Box2DShapeCache& instance(void);
      
	public:
		bool init();
		void addShapesWithFile(const std::string &plist);
      b2Fixture* addHullFixtureToBody(b2Body *body, const std::string &shape);
   
      /* Shapes created in the physics editor are in pixel dimensions.  When they are loaded
       * they are normalized to the size of maximum of the width/height.
       *
       */
		void addFixturesToBody(b2Body *body, const std::string &shape, float32 scaleMeters);
		CCPoint anchorPointForShape(const string& shape);
      CCSize imageSizeForShape(const string& shape);
      const std::vector<b2Vec2>& hullPointsForShape(const std::string &shape);
		void reset();
		float getPtmRatio() { return ptmRatio; }
		~Box2DShapeCache() {}
      
	private:
		std::map<std::string, BodyDef *> shapeObjects;
		Box2DShapeCache(void) {}
		float ptmRatio;
	};
}

#endif /* defined(__Box2DTestBed__Box2DShapeCache__) */
