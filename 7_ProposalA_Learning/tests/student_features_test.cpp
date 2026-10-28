#include "ProposalStudentFeatures.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

namespace cfg = proposal_model_config;
static bool close(float a, float b) { return std::fabs(a-b) < 0.00001f; }

int main() {
    static_assert((cfg::kInputDim == 65 || cfg::kInputDim == 125) && cfg::kForwardIndex == 34 && cfg::kForwardHistoryOffset == 35,
                  "Training and runtime offsets disagree");
    ProposalStudentFeatures history;
    float values[cfg::kInputDim] = {};
    assert(!history.observe(1,255,false,1000));
    for (uint32_t t=1005;t<4000;t+=5) assert(!history.observe(1,255,false,t));
    assert(!history.ready());
    assert(!history.observe(1,255,false,4000));
    assert(history.ready());
    // The current flag changes immediately, but does not rewrite past slots.
    assert(!history.observe(1,255,true,4000));
    history.fill(values,-2,.5f,-.1f,1.1f);
    assert(values[cfg::kForwardIndex] == 1);
    for (int i=0;i<cfg::kHistory;++i) {
        assert(close(values[cfg::kCommandOffset+i],1));
        assert(close(values[cfg::kForwardHistoryOffset+i],0));
    }
    for (uint32_t t=4005;t<=4100;t+=5) assert(!history.observe(1,255,true,t));
    float guarded[cfg::kInputDim+2] = {};
    guarded[0]=-77;guarded[cfg::kInputDim+1]=-88;
    history.fill(guarded+1,-2,.5f,-.1f,1.1f);
    assert(guarded[0]==-77 && guarded[cfg::kInputDim+1]==-88);
    for (int i=0;i<cfg::kInputDim;++i) std::printf("%.9g%s",guarded[i+1],i+1==cfg::kInputDim?"\n":" ");

    history.reset();
    history.observe(1,255,false,1000,20,.25f);
    for (uint32_t t=1005;t<=4000;t+=5) history.observe(1,255,false,t,20+(t-1000)/100.0f,.25f);
    history.fill(values,0,.25f,0,0,50);
    if (cfg::kInputDim == 125) {
        for (int i=0;i<30;++i) {
            assert(close(values[65+2*i],-30.0f+i));
            assert(close(values[65+2*i+1],.25f));
        }
    }
    assert(history.observe(0,0,false,4005,0,0,false));
    assert(!history.ready());

    // Jitter crosses a slot boundary: 25ms off + 75ms on, not a 125ms slot.
    history.reset();
    history.observe(2,255,false,1000);
    history.observe(2,255,true,1025);
    history.observe(2,255,true,1075);
    history.observe(2,255,true,1125);
    history.fill(values,0,0,0,0);
    assert(!history.ready());
    assert(close(values[cfg::kCommandOffset+29],-1));
    assert(close(values[cfg::kForwardHistoryOffset+29],.75f));
    assert(history.observe(2,255,true,1176));
    history.fill(values,0,0,0,0);
    assert(!history.ready() && values[cfg::kForwardIndex]==1);
    for (int i=0;i<cfg::kHistory;++i) {
        assert(values[cfg::kCommandOffset+i]==0);
        assert(values[cfg::kForwardHistoryOffset+i]==0);
    }
    // Unsigned millis() wrap does not erase a valid interval.
    history.reset();
    const uint32_t start=std::numeric_limits<uint32_t>::max()-49U;
    history.observe(1,127,false,start);
    assert(!history.observe(1,127,true,start+25U));
    assert(!history.observe(1,127,true,start+75U));
    assert(!history.observe(1,127,true,start+100U));
    history.fill(values,0,0,0,0);
    assert(close(values[cfg::kCommandOffset+29],127.0f/255));
    assert(close(values[cfg::kForwardHistoryOffset+29],.75f));
    return 0;
}
