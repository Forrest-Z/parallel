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
            return a->IsPriorThan(*b);
        }
    };
    Heap<Ptr<ReferenceLine>, PtrReferenceLineComparator> references_heap;
    for(auto & i : *frame.references)
    {
        references_heap.push(AddressOf(i));
    }

    while (references_heap.HasNext())
    {
        auto i = references_heap.Next();
        Ptr<type::Trajectory> candidate;

        auto planning_result = PlanOnReferenceLine(stitch_trajectory.Back(), *i, frame, candidate);
        if(planning_result.OK())
        {
            result = stitch_trajectory + *candidate;
            return planning_result;
        }
    }

    return PlannerBase::Result(ErrorCode::Fail);
}
