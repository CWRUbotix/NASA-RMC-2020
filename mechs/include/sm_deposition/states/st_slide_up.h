#include <smacc/smacc.h>

namespace sm_atomic
{
using namespace cl_numbers_subscription;
using namespace smacc::default_transition_tags;

// STATE DECLARATION
struct SlideUp: smacc::SmaccState<SlideUp, SmDeposition>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<
    
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, SlideDown, SUCCESS>
    
    >reactions;

    
// STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrSubscriber, CbTimerCountdownLoop>(3);  // EvTimer triggers each 3 client ticks
    }

    void runtimeConfigure()
    {
    }

    void onEntry()
    {
        ROS_INFO("On Entry!");
    }

    void onExit()
    {
        ROS_INFO("On Exit!");
    }
  
};
} // namespace sm_atomic