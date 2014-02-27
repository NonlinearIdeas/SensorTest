//
//  TwoStateButton.cpp
//  Box2DTestBed
//
//  Created by James Wucher on 9/1/13.
//
//

#include "TwoStateButton.h"


bool TwoStateButton::init(TwoStateButtonTarget* target, CCSprite* spriteDown, CCSprite* spriteUp)
{
   assert(target != NULL);
   assert(spriteDown != NULL);
   assert(spriteUp != NULL);
   assert(dynamic_cast<TwoStateButtonTarget*>(target) != NULL);
   
   _target = target;
   _spriteDown = spriteDown;
   _spriteUp = spriteUp;
   
   addChild(_spriteDown);
   addChild(_spriteUp);
   
   _spriteUp->setVisible(true);
   _spriteDown->setVisible(false);
   
   _enabled = true;
   
   return true;
}


TwoStateButton::~TwoStateButton()
{
   
}

// The class registers/unregisters on entry
// or exit of the layer.  This
void TwoStateButton::onEnterTransitionDidFinish()
{
   CCNode::onEnterTransitionDidFinish();
   CCDirector::sharedDirector()->getTouchDispatcher()->addTargetedDelegate(this, 0, true);
}

void TwoStateButton::onExitTransitionDidStart()
{
   CCNode::onExitTransitionDidStart();
   CCDirector::sharedDirector()->getTouchDispatcher()->removeDelegate(this);
}

bool TwoStateButton::ccTouchBegan(CCTouch *pTouch, CCEvent *pEvent)
{
   if(IsTouchInside(pTouch))
   {
      _target->TwoStateButtonAction(true, this);
      _spriteUp->setVisible(false);
      _spriteDown->setVisible(true);
      return true;
   }
   return false;
}

void TwoStateButton::ccTouchMoved(CCTouch *pTouch, CCEvent *pEvent)
{
   // Nothing to do here.
}

void TwoStateButton::ccTouchEnded(CCTouch *pTouch, CCEvent *pEvent)
{
   _target->TwoStateButtonAction(false, this);
   _spriteUp->setVisible(true);
   _spriteDown->setVisible(false);
}

void TwoStateButton::ccTouchCancelled(CCTouch *pTouch, CCEvent *pEvent)
{
   _target->TwoStateButtonAction(false, this);
   _spriteUp->setVisible(true);
   _spriteDown->setVisible(false);
}

TwoStateButton* TwoStateButton::create(TwoStateButtonTarget* target, CCSprite* spriteDown, CCSprite* spriteUp)
{
   TwoStateButton *pRet = new TwoStateButton();
   if (pRet && pRet->init(target,spriteDown,spriteUp))
   {
      pRet->autorelease();
      return pRet;
   }
   else
   {
      CC_SAFE_DELETE(pRet);
      return NULL;
   }
}

CCRect TwoStateButton::GetSpriteRect(CCSprite* sprite)
{
   CCRect rect = CCRectMake(
                         sprite->getPositionX() - (sprite->getTextureRect().size.width/2)*sprite->getScaleX(),
                         sprite->getPositionY() - (sprite->getTextureRect().size.height/2)*sprite->getScaleY(),
                         sprite->getTextureRect().size.width * sprite->getScaleX() ,
                         sprite->getTextureRect().size.height * sprite->getScaleY()
                         );
   return rect;
}

bool TwoStateButton::IsTouchInside(CCTouch* touch)
{
   if(!_enabled)
      return false;
   CCPoint point = touch->getLocationInView();
   point = CCDirector::sharedDirector()->convertToGL(point);
   CCRect rect = GetSpriteRect(_spriteUp);
   return rect.containsPoint(point);
}