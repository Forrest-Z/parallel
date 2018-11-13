#include <Planner/impl/PlannerBase.h>
USING_NAMESPACE_NOX;
using namespace nox::app;


string PlannerBase::ParseErrorCode(PlannerBase::ErrorCode code)
{
    switch (code)
    {
        case Success:
            return "Plan Successfully";
        case Fail:
            return "Plan Failed";
        case InCollision:
            return "In Collision";
        default:
            return "Unknown Error";
    }
}
