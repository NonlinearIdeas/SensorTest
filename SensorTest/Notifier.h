/********************************************************************
 * File   : Notifier.h
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

#ifndef __Notifier__
#define __Notifier__

#include "CommonSTL.h"
#include "SingletonTemplate.h"

/*
 The Notifier is a singleton implementation of the Subject/Observer design
 pattern.  Any class/instance which wishes to participate as an observer
 of an event can derive from the Notified base class and register itself
 with the Notiifer for enumerated events.
 
 Notifier derived classes MUST implement the notify function, which has
 a prototype of:
 
 void Notify(const NOTIFIED_EVENT_TYPE_T& event)
 
 This is a data object passed from the Notifier class.  The structure
 passed has a void* in it.  There is no illusion of type safety here
 and it is the responsibility of the user to ensure it is cast properly.
 In most cases, it will be "NULL".
 
 Classes derived from Notified do not need to deregister (though it may
 be a good idea to do so) as the base class destrctor will attempt to
 remove itself from the Notifier system automatically.
 
 The event type is an enumeration and not a string as it is in many
 "generic" notification systems.  In practical use, this is for a closed
 application where the messages will be known at compile time.  This allows
 us to increase the speed of the delivery by NOT having a
 dictionary keyed lookup mechanism.  Some loss of generality is implied
 by this.
 
 This class/system is NOT thread safe, but could be made so with some
 mutex wrappers.  It is safe to call Attach/Detach as a consequence
 of calling Notify(...).
 
 */

typedef enum
{
   NE_MIN = 0,
   NE_DEBUG_BUTTON_PRESSED = NE_MIN,
   NE_DEBUG_LINE_DRAW_ADD_LINE_PIXELS,
   NE_DEBUG_TOGGLE_VISIBILITY,
   NE_DEBUG_MESSAGE,
   NE_RESET_DRAW_CYCLE,
   NE_VIEWPORT_CHANGED,
   NE_MAX,
} NOTIFIED_EVENT_TYPE_T;


class Notified
{
public:
   virtual bool Notify(NOTIFIED_EVENT_TYPE_T eventType, const bool& value)
   { return false; };
   virtual ~Notified();
   
};

class Notifier : public SingletonDynamic<Notifier>
{
public:
   
private:
   typedef vector<NOTIFIED_EVENT_TYPE_T> NOTIFIED_EVENT_TYPE_VECTOR_T;
   
   typedef map<Notified*,NOTIFIED_EVENT_TYPE_VECTOR_T> NOTIFIED_MAP_T;
   typedef map<Notified*,NOTIFIED_EVENT_TYPE_VECTOR_T>::iterator NOTIFIED_MAP_ITER_T;
   
   typedef vector<Notified*> NOTIFIED_VECTOR_T;
   typedef vector<NOTIFIED_VECTOR_T> NOTIFIED_VECTOR_VECTOR_T;
   
   NOTIFIED_MAP_T _notifiedMap;
   NOTIFIED_VECTOR_VECTOR_T _notifiedVector;
   NOTIFIED_MAP_ITER_T _mapIter;
   
   
   // This vector keeps a temporary list of observers that have completely
   // detached since the current "Notify(...)" operation began.  This is
   // to handle the problem where a Notified instance has called Detach(...)
   // because of a Notify(...) call.  The removed instance could be a dead
   // pointer, so don't try to talk to it.
   vector<Notified*> _detached;
   int32 _notifyDepth;
   
   void RemoveEvent(NOTIFIED_EVENT_TYPE_VECTOR_T& orgEventTypes, NOTIFIED_EVENT_TYPE_T eventType);
   void RemoveNotified(NOTIFIED_VECTOR_T& orgNotified, Notified* observer);
   
public:
   
   virtual void Reset();
   virtual bool Init() { Reset(); return true; }
   virtual void Shutdown() { Reset(); }
   
   void Attach(Notified* observer, NOTIFIED_EVENT_TYPE_T eventType);
   // Detach for a specific event
   void Detach(Notified* observer, NOTIFIED_EVENT_TYPE_T eventType);
   // Detach for ALL events
   void Detach(Notified* observer);
   
   // This template function (defined in the header file) allows you to
   // add interfaces to Notified easily and call them as needed.  Variants
   // will be generated at compile time by this template.
   template <typename T>
   bool Notify(NOTIFIED_EVENT_TYPE_T eventType, const T& value)
   {
      if(eventType < NE_MIN || eventType >= NE_MAX)
      {
         throw std::out_of_range("eventType out of range");
      }
      
      // Keep a copy of the list.  If it changes while iterating over it because of a
      // deletion, we may miss an object to update.  Instead, we keep track of Detach(...)
      // calls during the Notify(...) cycle and ignore anything detached because it may
      // have been deleted.
      NOTIFIED_VECTOR_T notified = _notifiedVector[eventType];
      
      // If a call to Notify leads to a call to Notify, we need to keep track of
      // the depth so that we can clear the detached list when we get to the end
      // of the chain of Notify calls.
      _notifyDepth++;
      
      // Loop over all the observers for this event.
      // NOTE that the the size of the notified vector may change if
      // a call to Notify(...) adds/removes observers.  This should not be a
      // problem because the list is a simple vector.
      bool result = true;
      for(int idx = 0; idx < notified.size(); idx++)
      {
         Notified* observer = notified[idx];
         if(_detached.size() > 0)
         {  // Instead of doing the search for all cases, let's try to speed it up a little
            // by only doing the search if more than one observer dropped off during the call.
            // This may be overkill or unnecessary optimization.
            switch(_detached.size())
            {
               case 0:
                  break;
               case 1:
                  if(_detached[0] == observer)
                     continue;
                  break;
               default:
                  if(std::find(_detached.begin(), _detached.end(), observer) != _detached.end())
                     continue;
                  break;
            }
         }
         result = result && observer->Notify(eventType,value);
         assert(result == true);
      }
      // Decrement this each time we exit.
      _notifyDepth--;
      if(_notifyDepth == 0 && _detached.size() > 0)
      {  // We reached the end of the Notify call chain.  Remove the temporary list
         // of anything that detached while we were Notifying.
         _detached.clear();
      }
      assert(_notifyDepth >= 0);
      return result;
   }
   
   // Some syntatic sugar
   bool Notify(NOTIFIED_EVENT_TYPE_T event)
   {
      return Notify<bool>(event,true);
   }
   /* Used for CPPUnit.  Could create a Mock...maybe...but this seems
    * like it will get the job done with minimal fuss.  For now.
    */
   // Return all events that this object is registered for.
   vector<NOTIFIED_EVENT_TYPE_T> GetEvents(Notified* observer);
   // Return all objects registered for this event.
   vector<Notified*> GetNotified(NOTIFIED_EVENT_TYPE_T event);
};


#endif /* defined(__Notifier__) */
