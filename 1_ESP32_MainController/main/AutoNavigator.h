/**********************************************************************
 * AutoNavigator.h
 *
 * 基于超声波数据的自动寻路模块。
 *********************************************************************/

#ifndef SQUID_AUTO_NAVIGATOR_H
#define SQUID_AUTO_NAVIGATOR_H

#include <cstdint>

#include "ForwardControl.h"
#include "TurnControl.h"
#include "UltrasonicManager.h"

class AutoNavigator {
public:
    AutoNavigator();

    void begin(ForwardControl* forwardControl, TurnControl* leftTurn, TurnControl* rightTurn);

    void setEnabled(bool enabled);
    bool isEnabled() const;

    void update(const UltrasonicManager& ultrasonicManager, uint32_t nowMs);

private:
    ForwardControl* _forwardControl;
    TurnControl* _leftTurnControl;
    TurnControl* _rightTurnControl;
    bool _enabled;
    uint32_t _decisionLockUntilMs;
};

#endif  // SQUID_AUTO_NAVIGATOR_H
