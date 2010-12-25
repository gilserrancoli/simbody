/* -------------------------------------------------------------------------- *
 *                      SimTK Core: SimTKcommon                               *
 * -------------------------------------------------------------------------- *
 * This is part of the SimTK Core biosimulation toolkit originating from      *
 * Simbios, the NIH National Center for Physics-Based Simulation of           *
 * Biological Structures at Stanford, funded under the NIH Roadmap for        *
 * Medical Research, grant U54 GM072970. See https://simtk.org.               *
 *                                                                            *
 * Portions copyright (c) 2008-9 Stanford University and the Authors.         *
 * Authors: Michael Sherman                                                   *
 * Contributors:                                                              *
 *                                                                            *
 * Permission is hereby granted, free of charge, to any person obtaining a    *
 * copy of this software and associated documentation files (the "Software"), *
 * to deal in the Software without restriction, including without limitation  *
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,   *
 * and/or sell copies of the Software, and to permit persons to whom the      *
 * Software is furnished to do so, subject to the following conditions:       *
 *                                                                            *
 * The above copyright notice and this permission notice shall be included in *
 * all copies or substantial portions of the Software.                        *
 *                                                                            *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS, CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,    *
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR      *
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE  *
 * USE OR OTHER DEALINGS IN THE SOFTWARE.                                     *
 * -------------------------------------------------------------------------- */


/**@file
 *
 * Implementation of non-inline methods from the Event classes.
 */

#include "SimTKcommon/basics.h"
#include "SimTKcommon/internal/Event.h"

#include <cassert>
#include <string>

namespace SimTK {


const char* Event::getCauseName(Cause cause) {
    switch(cause) {
    case Cause::Initialization:    return "Initialization";
    case Cause::Triggered:         return "Triggered";
    case Cause::Scheduled:         return "Scheduled";
    case Cause::TimeAdvanced:      return "TimeAdvanced";
    case Cause::Signaled:          return "Signaled";
    case Cause::Termination:       return "Termination";
    case Cause::Invalid:           return "Invalid";
    }
    return "UNRECOGNIZED EVENT CAUSE";
}


std::string Event::eventTriggerString(Trigger e) {
    // Catch special combos first
    if (e==NoEventTrigger)        return "NoEventTrigger";
    if (e==Falling)               return "Falling";
    if (e==Rising)                return "Rising";
    if (e==AnySignChange)         return "AnySignChange";

    // Not a special combo; unmask one at a time.
    const Trigger triggers[] =
     { PositiveToNegative,NegativeToPositive,NoEventTrigger };
    const char *triggerNames[] =
     { "PositiveToNegative","NegativeToPositive" };

    String s;
    for (int i=0; triggers[i] != NoEventTrigger; ++i)
        if (e & triggers[i]) {
            if (s.size()) s += "|";
            s += triggerNames[i];
            e = Trigger((unsigned)e & ~((unsigned)triggers[i])); 
        }

    // should have accounted for everything by now
    if (e != NoEventTrigger) {
        char buf[128];
        std::sprintf(buf, "0x%x", (unsigned)e);
        if (s.size()) s += " + ";
        s += "UNRECOGNIZED EVENT TRIGGER GARBAGE ";
        s += buf;
    }
    return s;
}



////////////////////////////
// EVENT TRIGGER INFO REP //
////////////////////////////

class EventTriggerInfo::EventTriggerInfoRep {
public:
    explicit EventTriggerInfoRep(EventTriggerInfo* h)
      : myHandle(h), eventId(EventId(InvalidIndex)), triggerOnRising(true), triggerOnFalling(true), localizationWindow(0.1)
    {
        assert(h);
    }

private:
    EventTriggerInfo* myHandle;
    friend class EventTriggerInfo;

    EventId  eventId;
    bool triggerOnRising;
    bool triggerOnFalling;
    Real localizationWindow;
};



    ////////////////////////
    // EVENT TRIGGER INFO //
    ////////////////////////

EventTriggerInfo::EventTriggerInfo() : rep(0) {
    rep = new EventTriggerInfoRep(this);
}
EventTriggerInfo::~EventTriggerInfo() {
    if (getRep().myHandle == this)
        delete rep;
    rep = 0;
}

EventTriggerInfo::EventTriggerInfo(EventId eventId) : rep(0) {
    rep = new EventTriggerInfoRep(this);
    rep->eventId = eventId;
}

EventTriggerInfo::EventTriggerInfo(const EventTriggerInfo& src) : rep(0) {
    rep = new EventTriggerInfoRep(src.getRep());
    rep->myHandle = this;
}

EventTriggerInfo& 
EventTriggerInfo::operator=(const EventTriggerInfo& src) {
    if (&src != this) {
        if (getRep().myHandle == this)
            delete rep;
        rep = new EventTriggerInfoRep(src.getRep());
        rep->myHandle = this;
    }
    return *this;
}

EventId EventTriggerInfo::getEventId() const {
    return getRep().eventId;
}
bool EventTriggerInfo::shouldTriggerOnRisingSignTransition() const {
    return getRep().triggerOnRising;
}
bool EventTriggerInfo::shouldTriggerOnFallingSignTransition() const {
    return getRep().triggerOnFalling;
}
Real EventTriggerInfo::getRequiredLocalizationTimeWindow()    const {
    return getRep().localizationWindow;
}

EventTriggerInfo& 
EventTriggerInfo::setEventId(EventId id) {
    updRep().eventId = id; 
    return *this;
}
EventTriggerInfo& 
EventTriggerInfo::setTriggerOnRisingSignTransition(bool shouldTrigger) {
    updRep().triggerOnRising = shouldTrigger; 
    return *this;
}
EventTriggerInfo& 
EventTriggerInfo::setTriggerOnFallingSignTransition(bool shouldTrigger) {
    updRep().triggerOnFalling = shouldTrigger; 
    return *this;
}
EventTriggerInfo& 
EventTriggerInfo::setRequiredLocalizationTimeWindow(Real w) {
    assert(w > 0);
    updRep().localizationWindow = w; 
    return *this;
}
    ////////////////////////////
    // EVENT TRIGGER INFO REP //
    ////////////////////////////

// All inline currently.

} // namespace SimTK

