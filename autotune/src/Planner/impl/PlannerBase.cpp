#include <Planner/impl/PlannerBase.h>
USING_NAMESPACE_NOX;
using namespace nox::app;

PlannerBase::Result PlannerBase::Plan(
    const type::Trajectory &stitch_trajectory,
    PlannerBase::Frame frame,
    type::Trajectory & result)
{
    struct PtrReferenceLineComparator
    {
        bool operator()(const Ptr<ReferenceLine> & a, const Ptr<ReferenceLine> & b)
        {
            return not a->IsPriorThan(*b);
        }
    };
    Heap<Ptr<ReferenceLine>, PtrReferenceLineComparator> references_heap;
    for(auto & i : frame.references)
    {
        references_heap.push(i);
    }

    while (references_heap.HasNext())
    {
        auto i = references_heap.Next();
        type::Trajectory candidate;

        auto planning_result = PlanOnReferenceLine(stitch_trajectory.Back(), i, frame, candidate);
        if(planning_result.OK())
        {
            result = stitch_trajectory + candidate;
            return planning_result;
        }
        else break;
    }

    return PlannerBase::Result(ErrorCode::Fail);
}

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
    }
}
