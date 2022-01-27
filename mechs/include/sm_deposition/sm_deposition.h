#include <smacc/smacc.h>

// CLIENTS
#include <ros_timer_client/cl_ros_timer.h>

// ORTHOGONALS
#include <sm_deposition/orthogonals/or_timer.h>

//CLIENT BEHAVIORS


using namespace boost;
using namespace smacc;

namespace sm_deposition
{

//STATE
class State1;
class State2;

//--------------------------------------------------------------------
//STATE_MACHINE
struct SmDeposition
    : public smacc::SmaccStateMachineBase<SmDeposition, State1>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<OrTimer>();
    }
};

} // namespace sm_deposition

#include <sm_deposition/states/st_slide_up.h>
#include <sm_depositionstates/st_slide_down.h>