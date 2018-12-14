#include <Planner/tool/lattice/LatticeEvaluator.h>
#include <nox>
#include <Planner/type/PiecewiseAccelerationCurve.h>
#include <Planner/tool/PiecewiseBrakingTrajectoryGenerator.h>
#include <iostream>
USING_NAMESPACE_NOX;
using namespace nox::app;


LatticeEvaluator::LatticeEvaluator(
    const nox::math::Derivative<2> &init_state,
    const lattice::Bundle &lon_bundles,
    const lattice::Bundle &lat_bundles,
    Ptr<type::Vehicle> vehicle,
    Ptr<STGraph> st_graph,
    Ptr<ReferenceLine> reference)
    : _init_state(init_state, 0),
      _vehicle(vehicle),
      _st_graph(st_graph),
      _reference_line(reference),
      _constaint_checker(vehicle)
{
    InitParameter();
    ComputeLonGuideVelocity();

    double s = _init_state[0];
    double start_time = _st_graph->TRange().Start;
    double end_time = _st_graph->TRange().End;
    double vehicle_length = vehicle->param.length.x;
    double stop_line = reference->GetNextStopLine(s);

    for(auto & lon_traj : lon_bundles)
    {
        double lon_end_s = lon_traj->Calculate(0, end_time);

        /// 若轨迹的终点跑到了停止点后边，则不予考虑
        if(s < stop_line and lon_end_s + vehicle_length - _param._stop_in_range_threshold > stop_line)
            continue;

        /// 不满足车辆的约束（加速度、速度等边界内），则不予考虑
        if(!_constaint_checker.CheckLongitudinal(lon_traj))
            continue;

        for(auto & lat_traj : lat_bundles)
        {
            // TODO: 检查轨迹合法性
            lattice::Combination candidate(lon_traj, lat_traj);
            Evaluate(candidate);
            _candidates.push(std::move(candidate));
        }
    }
}

void LatticeEvaluator::InitParameter()
{
//    _param._vehicle._min_lon_a = _vehicle->param.limit.lon.a.Lower;
    // TODO: 使用真实的数据
}


void LatticeEvaluator::ComputeLonGuideVelocity() // TODO：改成服从Speed Control的
{
    _reference_v.clear();

    double s = _init_state[0];
    double cruise_v = _reference_line->CruisingSpeed();
    double stop_line = _reference_line->GetNextStopLine(s);

    if(stop_line <= _reference_line->Length())
    {
        //region 有需要停止的目标，按速度为零计算
        double ds = stop_line - _init_state[0];

        if(ds < _param._stop_in_range_threshold)
        {
            //region 已经在停止点附近，因此整条轨迹速度为零
            PiecewiseAccelerationCurve lon_traj(s, 0);
            lon_traj.PushSegment(0, _param._planning_temporal_length);

            for(double t : range(0, _param._time_resolution, _param._planning_temporal_length))
            {
                _reference_v.emplace_back(lon_traj.Calculate(1, t));
            }
            //endregion
        }
        else
        {
            //region 生成减速轨迹
            double a_comfort = _param._vehicle._comfort_a_factor * _param._vehicle._max_lon_a;
            double d_comfort = -_param._vehicle._comfort_a_factor * _param._vehicle._min_lon_a;

            auto lon_traj = PiecewiseBrakingTrajectoryGenerator::Generate
            (
                stop_line, 
                s, cruise_v,
                _init_state[1], a_comfort, d_comfort,
                _param._planning_temporal_length
            );

            for(double t : range(0, _param._time_resolution, _param._planning_temporal_length))
            {
                _reference_v.emplace_back(lon_traj->Calculate(1, t));
            }
            //endregion
        }

        //endregion
    }
    else
    {
        //region 没有停止的目标，按轨迹巡航速度计算
        PiecewiseAccelerationCurve lon_traj(s, cruise_v);
        lon_traj.PushSegment(0, _param._planning_temporal_length);

        for(double t : range(0, _param._time_resolution, _param._planning_temporal_length))
        {
            _reference_v.emplace_back(lon_traj.Calculate(1, t));
        }
        //endregion
    }
}

void LatticeEvaluator::Evaluate(lattice::Combination &candidate) const
{
    candidate.cost_sum = Evaluate(candidate.lon, candidate.lat, candidate.costs) * candidate.lon->state.cost_factor.all * candidate.lat->state.cost_factor.all;
}

double LatticeEvaluator::Evaluate(
    const Ptr<lattice::Curve> &lon_traj,
    const Ptr<lattice::Curve> &lat_traj,
    std::vector<double> &costs) const
{
    // Costs: (copy from apollo)
    // 1. Cost of missing the objective
    // 2. Cost of logitudinal jerk
    // 3. Cost of logitudinal collision
    // 4. Cost of lateral offsets
    // 5. Cost of lateral comfort
    analyze_init(LatticeEvaluator);
    analyze_enable(false);

    analyze(1. Cost of missing the objective);
    double lon_objective_cost   = LonObjectiveCost(lon_traj);

    analyze(2. Cost of logitudinal jerk);
    double lon_jerk_cost        = LonComfortCost(lon_traj);

    analyze(3. Cost of logitudinal collision)
    double lon_collision_cost   = LonCollisionCost(lon_traj);

    analyze(4. Cost of centripetal acceleration);
    double centripetal_acc_cost = CentripetalAccelerationCost(lon_traj, lat_traj);

    analyze(5. Cost of lateral offsets);
    double evaluation_horizon = std::min(_param._planning_distance, std::max(lon_traj->Boundary(), lat_traj->Boundary()));
    double lat_offset_cost    = LatOffsetCost(lat_traj, evaluation_horizon);

    analyze(6. Cost of lateral comfort);
    double lat_comfort_cost   = LatComfortCost(lon_traj, lat_traj);

    lon_objective_cost   *= _param._weight._cost._lon_objective;
    lon_jerk_cost        *= _param._weight._cost._lon_comfort;
    lon_collision_cost   *= _param._weight._cost._lon_collision;
    centripetal_acc_cost *= _param._weight._cost._centripetal_acc;
    lat_offset_cost      *= _param._weight._cost._lat_offset;
    lat_comfort_cost     *= _param._weight._cost._lat_comfort;

    costs.push_back(lon_objective_cost);
    costs.push_back(lon_jerk_cost);
    costs.push_back(lon_collision_cost);
    costs.push_back(centripetal_acc_cost);
    costs.push_back(lat_offset_cost);
    costs.push_back(lat_comfort_cost);

    return
        lon_objective_cost +
        lon_jerk_cost +
        lon_collision_cost +
        centripetal_acc_cost +
        lat_offset_cost +
        lat_comfort_cost;
}

double LatticeEvaluator::LonObjectiveCost(const Ptr<lattice::Curve> &lon_traj) const
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

    double speed_cost = speed_cost_sqr_sum / (speed_cost_weight_sum + type::Real::Epsilon) * lon_traj->state.cost_factor.v_reached;
    double s_travelled_cost = 1.0 / (1.0 + ds) * lon_traj->state.cost_factor.s_travelled;

    return (speed_cost * _param._weight._v_reached +
        s_travelled_cost * _param._weight._s_travelled)
        / (_param._weight._v_reached + _param._weight._s_travelled);
}

double LatticeEvaluator::LonCollisionCost(const Ptr<lattice::Curve> &lon_traj) const
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

double LatticeEvaluator::LonComfortCost(const Ptr<lattice::Curve> &lon_traj) const
{
    double cost_sqr_sum = 0;
    double cost_abs_sum = 0;

    for(double t : range(0, _param._time_resolution, _param._planning_temporal_length))
    {
        double a = lon_traj->Calculate(2, t);
        double jerk = lon_traj->Calculate(3, t);
        double cost = jerk / _vehicle->param.limit.lon.jerk.Upper + a / _vehicle->param.limit.lon.a.Upper;
        cost_sqr_sum += cost * cost;
        cost_abs_sum += std::abs(cost);
    }

    return cost_sqr_sum / (cost_abs_sum + type::Real::Epsilon);
}

double LatticeEvaluator::CentripetalAccelerationCost(const Ptr<lattice::Curve> &lon_traj) const
{
    // Assumes the vehicle is not obviously deviate from the reference line.
    // (copy from apollo)
    double centripetal_acc_sum = 0;
    double centripetal_acc_sqr_sum = 0;

    for(double t : range(0, _param._time_resolution, _param._planning_temporal_length))
    {
        double s = lon_traj->Calculate(0, t);
        double v = lon_traj->Calculate(1, t);

        auto ref_point = _reference_line->path.PointAtDistance(s);
        double centripetal_acc = v * v * ref_point.kappa;

        centripetal_acc_sum += 1; // std::abs(centripetal_acc);
        centripetal_acc_sqr_sum += centripetal_acc * centripetal_acc;
    }

    return centripetal_acc_sqr_sum / (centripetal_acc_sum + type::Real::Epsilon);
}

double LatticeEvaluator::LatOffsetCost(const Ptr<lattice::Curve> &lat_traj, double evaluation_horizon) const
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
            cost_sqr_sum += abs(cost) * _param._weight._opposite_side_offset;
            cost_abs_sum += 1; //std::abs(cost); // * _param._weight._opposite_side_offset;
        }
        else
        {
            cost_sqr_sum += abs(cost) * _param._weight._same_side_offset;
            cost_abs_sum += 1; // std::abs(cost); // * _param._weight._same_side_offset;
        }
    }

    double result = cost_sqr_sum / (cost_abs_sum + type::Real::Epsilon) * lat_traj->state.cost_factor.lateral_offset;
    return result;
}

double LatticeEvaluator::LatComfortCost(
    const Ptr<lattice::Curve> &lon_traj,
    const Ptr<lattice::Curve> &lat_traj) const
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

double LatticeEvaluator::CentripetalAccelerationCost(
    const Ptr<lattice::Curve> &lon,
    const Ptr<lattice::Curve> &lat) const
{
    double centripetal_acc_sum = 0;
    double centripetal_acc_sqr_sum = 0;

    double t0 = _param._time_resolution;
    double s0 = lon->Calculate(0, t0);
    double s_max = _reference_line->path.Back().s;
    double last_s = -Real::Epsilon;

    for(double t : range(t0, _param._time_resolution, _param._planning_temporal_length))
    {
        math::Derivative<2> s, l;
        s[0] = lon->Calculate(0, t);

        if(last_s > 0)
            s[0] = std::max(last_s, s[0]);
        last_s = s[0];

        if(s[0] > s_max) break;

        s[1] = std::max(Real::Epsilon, lon->Calculate(1, t));
        s[2] = lon->Calculate(2, t);

        double ds = s[0] - s0;
        l[0] = lat->Calculate(0, ds);
        l[1] = lat->Calculate(1, ds);
        l[2] = lat->Calculate(2, ds);

        auto nearest_point = _reference_line->path.PointAtDistance(s[0]);

        TrajectoryPoint point;
        math::Cartesian original_point;

        math::Frenet2Cartesian(
            math::Cartesian(nearest_point.pose.x, nearest_point.pose.y, nearest_point.pose.theta),
            nearest_point.s, nearest_point.kappa, nearest_point.dkappa,
            s, l,
            original_point,
            point.kappa,
            point.v,
            point.a
        );

        double centripetal_acc = point.v * point.v * point.kappa;
        centripetal_acc_sum += 1;
        centripetal_acc_sqr_sum += abs(centripetal_acc);
    }

    return centripetal_acc_sqr_sum / (centripetal_acc_sum + type::Real::Epsilon);
}


