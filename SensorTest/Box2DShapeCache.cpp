//
//  Box2DShapeCache.cpp
//  Box2DTestBed
//
//  Created by James Wucher on 8/3/13.
//
//

#include "Box2DShapeCache.h"

//
//  Box2DShapeCache.cpp
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

#include "Box2DShapeCache.h"
#include "Box2D.h"
#include "CCNS.h"

using namespace cocos2d;

/**
 * Internal class to hold the fixtures
 */
class FixtureLink
{
public:
   FixtureLink() :
      next(NULL)
   {
   }
   
   ~FixtureLink()
   {
      delete next;
      delete fixture.shape;
   }
   
   FixtureLink *next;
   b2FixtureDef fixture;
   int callbackData;
};

class BodyDef
{
public:
	BodyDef() :
      fixtures(NULL)
   {
   }
   
	~BodyDef()
   {
		if (fixtures)
			delete fixtures;
	}
   
	FixtureLink *fixtures;
   std::vector<b2Vec2> hullPoints;
	CCPoint anchorPoint;
   uint32 imageWidth;
   uint32 imageHeight;
};

static Box2DShapeCache *_sharedShapeCache = NULL;

Box2DShapeCache& Box2DShapeCache::instance(void)
{
	if (!_sharedShapeCache)
   {
		_sharedShapeCache = new Box2DShapeCache();
      _sharedShapeCache->init();
	}
   
	return *_sharedShapeCache;
}

bool Box2DShapeCache::init() {
	return true;
}

void Box2DShapeCache::reset() {
	std::map<std::string, BodyDef *>::iterator iter;
	for (iter = shapeObjects.begin() ; iter != shapeObjects.end() ; ++iter) {
		delete iter->second;
	}
	shapeObjects.clear();
}

void Box2DShapeCache::addFixturesToBody(b2Body *body, const std::string &shape, float32 scale)
{
	std::map<std::string, BodyDef *>::iterator pos = shapeObjects.find(shape);
	assert(pos != shapeObjects.end());
   
	BodyDef *so = (*pos).second;
   
	FixtureLink *fix = so->fixtures;
   while (fix)
   {
      switch(fix->fixture.shape->m_type)
      {
         case b2Shape::e_polygon:
         {
            b2PolygonShape* shapeOrg = (b2PolygonShape*)fix->fixture.shape;
            b2PolygonShape polyShape;
            polyShape = *(b2PolygonShape*)fix->fixture.shape;
            polyShape.ApplyScale(scale);
            fix->fixture.shape = &polyShape;
            body->CreateFixture(&fix->fixture);
            fix->fixture.shape = shapeOrg;
         }
            break;
         case b2Shape::e_circle:
         {
            b2CircleShape* shapeOrg = (b2CircleShape*)fix->fixture.shape;
            b2CircleShape circleShape;
            circleShape = *(b2CircleShape*)fix->fixture.shape;
            circleShape.ApplyScale(scale);
            fix->fixture.shape = &circleShape;
            body->CreateFixture(&fix->fixture);
            fix->fixture.shape = shapeOrg;
         }
            break;
         default:
            assert(false);
            break;
      }      fix = fix->next;
   }
}

cocos2d::CCPoint Box2DShapeCache::anchorPointForShape(const std::string &shape)
{
	std::map<std::string, BodyDef *>::iterator pos = shapeObjects.find(shape);
	assert(pos != shapeObjects.end());
   
	BodyDef *bd = (*pos).second;
	return bd->anchorPoint;
}

b2Fixture* Box2DShapeCache::addHullFixtureToBody(b2Body *body, const std::string &shape)
{
	std::map<std::string, BodyDef *>::iterator pos = shapeObjects.find(shape);
	assert(pos != shapeObjects.end());
   
	BodyDef *so = (*pos).second;
   assert(so);
   
   if(so->hullPoints.size() > 2)
   {
      b2ChainShape chain;
      chain.CreateLoop(&so->hullPoints[0], so->hullPoints.size());
      FixtureLink *fix = so->fixtures;
      if(fix)
      {
         b2FixtureDef fixture = fix->fixture;
         fixture.shape = &chain;
         fixture.userData = NULL;
         return body->CreateFixture(&fixture);
      }
   }
   return NULL;
}

const std::vector<b2Vec2>& Box2DShapeCache::hullPointsForShape(const std::string &shape)
{
	std::map<std::string, BodyDef *>::iterator pos = shapeObjects.find(shape);
	assert(pos != shapeObjects.end());
   
	BodyDef *bd = (*pos).second;
	return bd->hullPoints;
}


void Box2DShapeCache::addShapesWithFile(const std::string &plist)
{
   
	//const char *fullName = CCFileUtils::sharedFileUtils()->fullPathForFilename(plist.c_str()).c_str();
   
   CCDictionary *dict = CCDictionary::createWithContentsOfFile(plist.c_str());
   // not triggered - cocos2dx delivers empty dict if non was found
   
	CCAssert(dict != NULL, "Shape-file not found");
   
   CCAssert(dict->count() != 0, "plist file empty or not existing");
   
	CCDictionary *metadataDict = (CCDictionary *)dict->objectForKey("metadata");
   
   int format = static_cast<CCString *>(metadataDict->objectForKey("format"))->intValue();
   ptmRatio = static_cast<CCString *>(metadataDict->objectForKey("ptm_ratio"))->floatValue();
   CCLOG("ptmRatio = %f",ptmRatio);
	CCAssert(format == 1, "Format not supported");
   
   
	CCDictionary *bodyDict = (CCDictionary *)dict->objectForKey("bodies");
   
   b2Vec2 vertices[b2_maxPolygonVertices];
   
   CCDictElement *dictElem;
   std::string bodyName;
	CCDictionary *bodyData;
   //iterate body list
   CCDICT_FOREACH(bodyDict,dictElem )
   {
      bodyData = (CCDictionary*)dictElem->getObject();
      bodyName = dictElem->getStrKey();
      
      
      BodyDef *bodyDef = new BodyDef();
      bodyDef->anchorPoint = CCPointFromString(static_cast<CCString *>(bodyData->objectForKey("anchorpoint"))->getCString());
      bodyDef->imageWidth = static_cast<CCString *>(bodyData->objectForKey("imageWidth"))->intValue();
      bodyDef->imageHeight = static_cast<CCString *>(bodyData->objectForKey("imageHeight"))->intValue();
      
      float32 maxDim = MAX(bodyDef->imageHeight,bodyDef->imageWidth);
      
      CCArray *fixtureList = (CCArray*)(bodyData->objectForKey("fixtures"));
      FixtureLink **nextFixtureDef = &(bodyDef->fixtures);
      
      //iterate fixture list
      CCObject *arrayElem;
      CCARRAY_FOREACH(fixtureList, arrayElem)
      {
         b2FixtureDef basicData;
         CCDictionary* fixtureData = (CCDictionary*)arrayElem;
         
         basicData.filter.categoryBits = static_cast<CCString *>(fixtureData->objectForKey("filter_categoryBits"))->intValue();
         
         basicData.filter.maskBits = static_cast<CCString *>(fixtureData->objectForKey("filter_maskBits"))->intValue();
         basicData.filter.groupIndex = static_cast<CCString *>(fixtureData->objectForKey("filter_groupIndex"))->intValue();
         basicData.friction = static_cast<CCString *>(fixtureData->objectForKey("friction"))->floatValue();
         
         basicData.density = static_cast<CCString *>(fixtureData->objectForKey("density"))->floatValue();
         
         basicData.restitution = static_cast<CCString *>(fixtureData->objectForKey("restitution"))->floatValue();
         
         basicData.isSensor = (bool)static_cast<CCString *>(fixtureData->objectForKey("isSensor"))->intValue();
         
         CCString *cb = static_cast<CCString *>(fixtureData->objectForKey("userdataCbValue"));
         
         int callbackData = 0;
         
			if (cb)
         {
				callbackData = cb->intValue();
         }
         
         // Get the HULL points
         CCArray* hullPointList = (CCArray *)(fixtureData->objectForKey("hull"));
         if(hullPointList != NULL)
         {
            CCObject *piter;
            CCARRAY_FOREACH(hullPointList, piter)
            {
               CCString *verStr = (CCString*)piter;
               CCPoint offset = CCPointFromString(verStr->getCString());
               bodyDef->hullPoints.push_back(b2Vec2(offset.x/maxDim,offset.y/maxDim));
            }
         }
         
			std::string fixtureType = static_cast<CCString *>(fixtureData->objectForKey("fixture_type"))->m_sString;
         
			if (fixtureType == "POLYGON")
         {
				CCArray *polygonsArray = (CCArray *)(fixtureData->objectForKey("polygons"));
            
            CCObject *dicArrayElem;
            CCARRAY_FOREACH(polygonsArray, dicArrayElem)
            {
               FixtureLink *fix = new FixtureLink();
               fix->fixture = basicData; // copy basic data
               fix->callbackData = callbackData;
               
               b2PolygonShape *polyshape = new b2PolygonShape();
               int vindex = 0;
               
               CCArray *polygonArray = (CCArray*)dicArrayElem;
               
               assert(polygonArray->count() <= b2_maxPolygonVertices);
               
               CCObject *piter;
               CCARRAY_FOREACH(polygonArray, piter)
               {
                  CCString *verStr = (CCString*)piter;
                  CCPoint offset = CCPointFromString(verStr->getCString());
                  vertices[vindex].x = (offset.x / maxDim) ;
                  vertices[vindex].y = (offset.y / maxDim) ;
                  vindex++;
               }
               
               polyshape->Set(vertices, vindex);
               fix->fixture.shape = polyshape;
               
               // create a list
               *nextFixtureDef = fix;
               nextFixtureDef = &(fix->next);
            }
            
            
			}
         else if (fixtureType == "CIRCLE")
         {
				FixtureLink *fix = new FixtureLink();
            fix->fixture = basicData; // copy basic data
            fix->callbackData = callbackData;
            
            CCDictionary *circleData = (CCDictionary *)fixtureData->objectForKey("circle");
            
            b2CircleShape *circleShape = new b2CircleShape();
            
            circleShape->m_radius = static_cast<CCString *>(circleData->objectForKey("radius"))->floatValue() / maxDim;
				CCPoint p = CCPointFromString(static_cast<CCString *>(circleData->objectForKey("position"))->getCString());
            circleShape->m_p = b2Vec2(p.x / maxDim, p.y / maxDim);
            fix->fixture.shape = circleShape;
            
            // create a list
            *nextFixtureDef = fix;
            nextFixtureDef = &(fix->next);
            
			}
         else
         {
				CCAssert(0, "Unknown fixtureType");
			}
		}
      // add the body element to the hash
      shapeObjects[bodyName] = bodyDef;
      
   }
   
}