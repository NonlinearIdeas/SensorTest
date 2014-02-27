//
//  TwoStateButton.h
//  Box2DTestBed
//
//  Created by James Wucher on 9/1/13.
//
//

#ifndef __Box2DTestBed__TwoStateButton__
#define __Box2DTestBed__TwoStateButton__

#include "CommonSTL.h"
#include "CommonProject.h"

/* This class is a simple two-state graphical
 * button (no text).  
 *
 * This button uses a delegate approach to 
 * handle the button events.  The rationale
 * (at this level) is that a button will be 
 * placed with other buttons that all "live"
 * in a layer.  That layer should process
 * the button message and decide what to 
 * with it.
 *
 */

class TwoStateButton;

class TwoStateButtonTarget
{
public:
   virtual void TwoStateButtonAction(bool pressed, TwoStateButton* button) = 0;
};

class TwoStateButton : public CCNode, public CCTargetedTouchDelegate
{
private:
   
   CCSprite* _spriteDown;  // Weak Ref
   CCSprite* _spriteUp;    // Weak Ref
   bool _enabled;
   CCRect GetSpriteRect(CCSprite* sprite);
   TwoStateButtonTarget* _target;
   bool init(TwoStateButtonTarget* target, CCSprite* spriteDown, CCSprite* spriteUp);
   
   /* Liberated from the code base for CCControl.
    */
   bool IsTouchInside(CCTouch* touch);
public:
   
   virtual ~TwoStateButton();
   
   bool GetEnabled() { return _enabled; }
   void SetEnabled(bool enabled) { _enabled = enabled; }
   
   // The class registers/unregisters on entry
   // or exit of the layer.  This
   virtual void onEnterTransitionDidFinish();
   virtual void onExitTransitionDidStart();
   
   virtual bool ccTouchBegan(CCTouch *pTouch, CCEvent *pEvent);
   virtual void ccTouchMoved(CCTouch *pTouch, CCEvent *pEvent);
   virtual void ccTouchEnded(CCTouch *pTouch, CCEvent *pEvent);
   virtual void ccTouchCancelled(CCTouch *pTouch, CCEvent *pEvent);
   
   static TwoStateButton* create(TwoStateButtonTarget* target, CCSprite* spriteDown, CCSprite* spriteUp);
};


#endif /* defined(__Box2DTestBed__TwoStateButton__) */
