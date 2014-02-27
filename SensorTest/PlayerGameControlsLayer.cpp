//
//  PlayerGameControlsLayer.cpp
//  Box2DTestBed
//
//  Created by James Wucher on 9/9/13.
//
//

#include "PlayerGameControlsLayer.h"
#include "GraphicsUtilities.h"
#include "Notifier.h"


void PlayerGameControlsLayer::AddButton(const char* upFrame, const char* dnFrame, uint32 tag, float32 xPercent, float32 yPercent)
{
   CCSize scrSize = CCDirector::sharedDirector()->getWinSize();
   const uint32 SPRITE_MAX_DIM = 100;
   
   CCSprite* spriteUp = CCSprite::createWithSpriteFrameName(upFrame);
   spriteUp->setPosition(ccp(scrSize.width*xPercent,scrSize.height*yPercent));
   GraphicsUtilities::AdjustSpriteSizeToMaximumPixels(spriteUp, SPRITE_MAX_DIM);
   
   CCSprite* spriteDn = CCSprite::createWithSpriteFrameName(dnFrame);
   spriteDn->setPosition(spriteUp->getPosition());
   spriteDn->setScale(spriteUp->getScale());
   
   TwoStateButton* button = TwoStateButton::create(this, spriteDn, spriteUp);
   button->setTag(tag);
   addChild(button);
}

PlayerGameControlsLayer::PlayerGameControlsLayer()
{
   
}

bool PlayerGameControlsLayer::init()
{
   CCSpriteFrameCache::sharedSpriteFrameCache()->addSpriteFramesWithFile("buttons.plist", "buttons.png");
   
   AddButton("btn_exit_up.png", "btn_exit_dn.png", BUTTON_TRACK, 0.1f, 0.3f);
   AddButton("btn_zoom_in_up.png", "btn_zoom_in_dn.png", BUTTON_ZOOM_IN, 0.1f, 0.9f);
   AddButton("btn_zoom_out_up.png", "btn_zoom_out_dn.png", BUTTON_ZOOM_OUT, 0.1f, 0.7f);
   AddButton("btn_debug_up.png", "btn_debug_dn.png", BUTTON_TOGGLE_DEBUG, 0.1f, 0.5f);
   
   return true;
}

PlayerGameControlsLayer::~PlayerGameControlsLayer()
{
   
}

void PlayerGameControlsLayer::TwoStateButtonAction(bool pressed, TwoStateButton* button)
{
   switch (button->getTag())
   {
      case BUTTON_TOGGLE_DEBUG:
         Notifier::Instance().Notify<bool>(NE_DEBUG_TOGGLE_VISIBILITY,pressed);
         break;
      case BUTTON_ZOOM_IN:
         Notifier::Instance().Notify<bool>(NE_GAME_COMMAND_ZOOM_IN,pressed);
         break;
      case BUTTON_ZOOM_OUT:
         Notifier::Instance().Notify<bool>(NE_GAME_COMMAND_ZOOM_OUT,pressed);
         break;
      case BUTTON_TRACK:
         Notifier::Instance().Notify<bool>(NE_GAME_COMMAND_TRACK,pressed);
         break;
      default:
         assert(false);
         break;
   }
}

PlayerGameControlsLayer* PlayerGameControlsLayer::create()
{
   PlayerGameControlsLayer *pRet = new PlayerGameControlsLayer();
   if (pRet && pRet->init())
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
