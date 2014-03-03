//
//  PlayerGameControlsLayer.h
//  Box2DTestBed
//
//  Created by James Wucher on 9/9/13.
//
//

#ifndef __Box2DTestBed__PlayerGameControlsLayer__
#define __Box2DTestBed__PlayerGameControlsLayer__

#include "CommonSTL.h"
#include "CommonProject.h"
#include "TwoStateButton.h"

class PlayerGameControlsLayer : public CCLayer, public TwoStateButtonTarget
{
private:
   void AddButton(const char* upFrame, const char* dnFrame, uint32 tag, float32 xPercent, float32 yPercent);
   PlayerGameControlsLayer();
   enum
   {
      BUTTON_TRACK = 1000,
      BUTTON_ZOOM_IN,
      BUTTON_ZOOM_OUT,
      BUTTON_TOGGLE_DEBUG,
      BUTTON_NEXT,
   };
protected:
   bool init();
public:
   ~PlayerGameControlsLayer();
   virtual void TwoStateButtonAction(bool pressed, TwoStateButton* button);
   static PlayerGameControlsLayer* create();
};

#endif /* defined(__Box2DTestBed__PlayerGameControlsLayer__) */
