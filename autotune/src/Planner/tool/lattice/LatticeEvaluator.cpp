#include <Planner/tool/lattice/LatticeEvaluator.h>
#include <nox>
#include <Planner/type/ConstantAccelerationCurve.h>

using namespace nox::app;


LatticeEvaluator::LatticeEvaluator(
    const nox::math::Derivative<2> &init_state,
    const ReferenceLine::Target &target,
    const lattice::Bundle &lon_bundles,
    const lattice::Bundle &lat_bundles,
    Ptr<type::Vehicle> vehicle,
    nox::Ptr<nox::app::STGraph> st_graph,
    ReferenceLine &reference)
    : _init_state(init_state, 0),
      _vehicle(vehicle),
      _st_graph(st_graph),
      _reference_line(reference),
      _constaint_checker(vehicle)
{
    InitParameter();
    ComputeLonGuideVelocity(target);

    double start_time = _st_graph->TRange().Start;
    double end_time = _st_graph->TRange().End;
    double stop_point = target.s;

    for(auto & lon_traj : lon_bundles)
    {
        double lon_end_s = lon_traj->Calculate(0, end_time);

        /// 若轨迹的终点跑到了停止点后边，则不予考虑
        if(_init_state[0] < stop_point and lon_end_s + _param._stop_in_range_threshold > stop_point)
            continue;

        /// 不满足车辆的约束（加速度、速度等边界内），则不予考虑
        if(!_constaint_checker.CheckLongitudinal(lon_traj))
            continue;

        for(auto & lat_traj : lat_bundles)
        {
            // TODO: 检查轨迹合法性
            lattice::Combination candidate(lon_traj, lat_traj);
            Evaluate(target, candidate);
            _candidates.push(std::move(candidate));
        }
    }
}

void LatticeEvaluator::InitParameter()
{
//    _param._vehicle._min_lon_a = _vehicle->param.limit.lon.a.Lower;
    // TODO: 使用真实的数据
}


void LatticeEvaluator::ComputeLonGuideVelocity(const LatticeEvaluator::Target &target)
{
    _reference_v.clear();

    double brake_a = _param._vehicle._comfort_a_factor * _param._vehicle._min_lon_a;
    assert(brake_a < 0);

    if(target.IsStop())
    {
        //region 有需要停止的目标，按速度为零计算
        double ds = target.s - _init_state[0];

        if(ds < _param._stop_in_range_threshold)
        {
            //region 已经在停止点附近，因此整条轨迹速度为零
            ConstantAccelerationCurve lon_traj(_init_state[0], 0);
            lon_traj.PushSegment(0, _param._planning_temporal_length);

            for(double t : range(0, _param._time_resolution, _param._planning_temporal_length))
            {
                _reference_v.emplace_back(lon_traj.Calculate(1, t));
            }
            //endregion
        }

        double cruise_v = _vehicle->v.x;
        double brake_s = -cruise_v * cruise_v * 0.5 / brake_a;

        if(brake_s > ds)
        {
            //region 刹车距离不够，只能刹车到某个速度
            double reachable_v = std::sqrt(-2.0 * brake_a * ds);
            double brake_t = -reachable_v / brake_a;

            ConstantAccelerationCurve lon_traj(_init_state[0], reachable_v);
            lon_traj.PushSegment(brake_a, brake_t);

            if(lon_traj.Boundary() < _param._planning_temporal_length)
            {
                lon_traj.PushSegment(0, _param._planning_temporal_length - lon_traj.Boundary());
            }

            for(double t : range(0, _param._time_resolution, _param._planning_temporal_length))
            {
                _reference_v.emplace_back(lon_traj.Calculate(1, t));
            }
            //endregion
        }
        else
        {
            //region 有足够的刹车距离，先继续走，最后再停下来
            double brake_t = -cruise_v / brake_a;
            double cruise_s = ds - brake_s;
            double cruise_t = cruise_s / cruise_v;

            ConstantAccelerationCurve lon_traj(_init_state[0], cruise_v);
            lon_traj.PushSegment(0, cruise_t);
            lon_traj.PushSegment(brake_a, brake_t);

            if(lon_traj.Boundary() < _param._planning_temporal_length)
            {
                lon_traj.PushSegment(0, _param._planning_temporal_length - lon_traj.Boundary());
            }

            for(double t : range(0, _param._time_resolution, _param._planning_temporal_length))
            {
                _reference_v.emplace_back(lon_traj.Calculate(1, t));
            }
            //endregion
        }

        //endregion
    }
    else
    {
        //region 没有停止的目标，按轨迹巡航速度计算
        ConstantAccelerationCurve lon_traj(_init_state[0], target.v);
        lon_traj.PushSegment(0, _param._planning_temporal_length);

        for(double t : range(0, _param._time_resolution, _param._planning_temporal_length))
        {
            _reference_v.emplace_back(lon_traj.Calculate(1, t));
        }
        //endregion
    }
}

void LatticeEvaluator::Evaluate(const LatticeEvaluator::Target &target, lattice::Combination &candidate) const
{
    candidate.cost_sum = Evaluate(target, candidate.lon, candidate.lat, candidate.costs);
}

double LatticeEvaluator::Evaluate(
    const LatticeEvaluator::Target &target,
    const nox::Ptr<nox::math::Parametric<1>> &lon_traj,
    const nox::Ptr<nox::math::Parametric<1>> &lat_traj,
    std::vector<double> &costs) const
{
    // Costs: (copy from apollo)
    // 1. Cost of missing the objective
    // 2. Cost of logitudinal jerk
    // 3. Cost of logitudinal collision
    // 4. Cost of lateral offsets
    // 5. Cost of lateral comfort

    double lon_objective_cost   = LonObjectiveCost(lon_traj, target);
    double lon_jerk_cost        = LonComfortCost(lon_traj);
    double lon_collision_cost   = LonCollisionCost(lon_traj);
    double centripetal_acc_cost = CentripetalAccelerationCost(lon_traj);

    double evaluation_horizon = std::min(_param._planning_distance, lon_traj->Boundary());
    double lat_offset_cost    = LatOffsetCost(lat_traj, evaluation_horizon);
    double lat_comfort_cost   = LatComfortCost(lon_traj, lat_traj);

    costs.push_back(lon_objective_cost);
    costs.push_back(lon_jerk_cost);
    costs.push_back(lon_collision_cost);
    costs.push_back(lat_offset_cost);

    return
        _param._weight._cost._lon_objective * lon_objective_cost +
        _param._weight._cost._lon_jerk * lon_jerk_cost +
        _param._weight._cost._lon_collision * lon_collision_cost +
        _param._weight._cost._centripetal_acc * centripetal_acc_cost +
        _param._weight._cost._lat_offset * lat_offset_cost +
        _param._weight._cost._lat_comfort * lat_comfort_cost;
}

double LatticeEvaluator::LonObjectiveCost(
const Ptr <math::Parametric<1>> &lon_traj,
const LatticeEvaluator::Target &target) const
{
    double t_max = lon_traj->Boundary();
    double ds = lon_traj->Calculate(0, t_max) - lon_traj->Calculate(0, 0);

    double speed_cost_sqr_sum = 0;
    double speed_cost_weight_sum = 0;

    for(size_t i = 0, size = _reference_v.size(); i < size; i++)
    {
        double t = i * _param._time_resolution;
        double dv = _reference_v[i] - lon_traj->Calculate(1, t);

        speed_cost_sqr_sum += t * t * std::abs(dv);
        speed_cost_weight_sum += t * t;
    }

    double speed_cost = speed_cost_sqr_sum / (speed_cost_weight_sum + type::Real::Epsilon);
    double s_travelled_cost = 1.0 / (1.0 + ds);

    return (speed_cost * _param._weight._v_reached +
        s_travelled_cost * _param._weight._s_travelled)
        / (_param._weight._v_reached + _param._weight._s_travelled);
}

double LatticeEvaluator::LonCollisionCost(const nox::Ptr<nox::math::Parametric<1>> &lon_traj) const
{
    double cost_sqr_sum = 0;
    double cost_abs_sum = 0;

    for(double t : range(0, _param._time_resolution, _param._planning_temporal_length))
    {
        double s = lon_traj->Calculate(0, t);
        double sigma = _param._lon_collision_cost_std;

        for(auto & i : _st_graph->Obstacles())
        {
            auto obstalce = _st_graph->GetObstacle(i.first);
            auto s_range = obstalce->EstimateAtTime(t);

            double ds = 0; /// 离障碍物的安全距离
            if(s < s_range.Start - _param._lon_collision_yield_buffer)
            {
                ds = s_range.Start - _param._lon_collision_yield_buffer - s;
            }
            else if(s > s_range.End + _param._lon_collision_overtake_buffer)
            {
                ds = s - s_range.End - _param._lon_collision_overtake_buffer;
            }

            double cost = std::exp(-ds * ds / (2.0 * sigma * sigma));
            cost_sqr_sum += cost * cost;
            cost_abs_sum += cost;
        }
    }

    return cost_sqr_sum / (cost_abs_sum + type::Real::Epsilon);
}

double LatticeEvaluator::LonComfortCost(const nox::Ptr<nox::math::Parametric<1>> &lon_traj) const
{
    double cost_sqr_sum = 0;
    double cost_abs_sum = 0;

    for(double t : range(0, _param._time_resolution, _param._planning_temporal_length))
    {
        double jerk = lon_traj->Calculate(3, t);
        double cost = jerk / _vehicle->param.limit.lon.jerk.Upper;
        cost_sqr_sum += cost * cost;
        cost_abs_sum += std::abs(cost);
    }

    return cost_sqr_sum / (cost_abs_sum + type::Real::Epsilon);
}

double LatticeEvaluator::CentripetalAccelerationCost(const nox::Ptr<nox::math::Parametric<1>> &lon_traj) const
{
    // Assumes the vehicle is not obviously deviate from the reference line.
    // (copy from apollo)
    double centripetal_acc_sum = 0;
    double centripetal_acc_sqr_sum = 0;

    for(double t : range(0, _param._time_resolution, _param._planning_temporal_length))
    {
        double s = lon_traj->Calculate(0, t);
        double v = lon_traj->Calculate(1, t);

        auto ref_point = _reference_line.path->PointAtDistance(s);
        double centripetal_acc = v * v * ref_point.kappa;

        centripetal_acc_sum += std::abs(centripetal_acc);
        centripetal_acc_sqr_sum += centripetal_acc * centripetal_acc;
    }

    return centripetal_acc_sqr_sum / (centripetal_acc_sum + type::Real::Epsilon);
}

double LatticeEvaluator::LatOffsetCost(const nox::Ptr<nox::math::Parametric<1>> &lat_traj, double evaluation_horizon) const
{
    double lat_offset_start = lat_traj->Calculate(0, 0);
    double cost_sqr_sum = 0.0;
    double cost_abs_sum = 0.0;

    for(double s : range(0, _param._space_resolution, evaluation_horizon))
    {
        double lat_offset = lat_traj->Calculate(0, s);
        double cost = lat_offset / _param._lat_offset_bound;

        if(lat_offset * lat_offset_start < 0)
        {
            cost_sqr_sum += cost * cost * _param._weight._opposite_side_offset;
            cost_abs_sum += std::abs(cost) * _param._weight._opposite_side_offset;
        }
        else
        {
            cost_sqr_sum += cost * cost * _param._weight._same_side_offset;
            cost_abs_sum += std::abs(cost) * _param._weight._same_side_offset;
        }
    }

    return cost_sqr_sum / (cost_abs_sum + type::Real::Epsilon);
}

double LatticeEvaluator::LatComfortCost(
    const nox::Ptr<nox::math::Parametric<1>> &lon_traj,
    const nox::Ptr<nox::math::Parametric<1>> &lat_traj) const
{
    double max_cost = 0;

    for(double t : range(0, _param._time_resolution, _param._planning_temporal_length))
    {
        double s = lon_traj->Calculate(0, t);
        double ds = lon_traj->Calculate(1, t);
        double dds = lon_traj->Calculate(2, t);
        double dl = lat_traj->Calculate(1, s);
        double ddl = lat_traj->Calculate(2, s);

        double cost = ddl * ds * ds + dl * dds;
        max_cost = std::max(max_cost, std::abs(cost));
    }

    return max_cost;
}

bool LatticeEvaluator::HasNext() const
{
    return !_candidates.empty();
}

lattice::Combination LatticeEvaluator::Next()
{
    auto result = _candidates.top();
    _candidates.pop();
    return std::move(result);
}


