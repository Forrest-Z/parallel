#include <Simulator/Simulator.h>
#include <random>
using namespace nox::app;
USING_NAMESPACE_NOX;

void Simulator::Initialize()
{
    SetFrequency(10);

    KeyBoard::Register("Obstacle Simulator", [&](KeyBoard::Key key, KeyBoard::Flag flag) {

        Synchronized(this)
        {
            if(key <= bound(KeyBoard::Number0, KeyBoard::Number9))
            {
                Select(key - KeyBoard::Number0);
            }
            else
            {
                switch (key)
                {
                    case KeyBoard::X: case KeyBoard::x:
                        Add();
                        return true;
                    case KeyBoard::R: case KeyBoard::r:
                        Clear();
                        return true;
                    case KeyBoard::H: case KeyBoard::h:
                        PrintHelp();
                        return true;
                }

                if(_current_index == -1)
                {
                    PrintHelp();
                    Logger::W("Simulator") << "There is not obstacle to execute your operation !";
                    return true;
                }

                switch (key)
                {
                    case KeyBoard::W: case KeyBoard::w:
                        Move(1);
                        break;
                    case KeyBoard::S: case KeyBoard::s:
                        Move(-1);
                        break;
                    case KeyBoard::A: case KeyBoard::a:
                        Turn(1);
                        break;
                    case KeyBoard::D: case KeyBoard::d:
                        Turn(-1);
                        break;
                    case KeyBoard::C: case KeyBoard::c:
                        Delete();
                        break;
                    case KeyBoard::Q: case KeyBoard::q:
                        Shift(-1);
                        break;
                    case KeyBoard::E: case KeyBoard::e:
                        Shift(1);
                        break;
                    case KeyBoard::N: case KeyBoard::n:
                        StretchY(-0.1);
                        break;
                    case KeyBoard::M: case KeyBoard::m:
                        StretchY(0.1);
                        break;
                    case KeyBoard::O: case KeyBoard::o:
                        StretchX(-0.1);
                        break;
                    case KeyBoard::P: case KeyBoard::p:
                        StretchX(0.1);
                        break;
                }
            }
        }

        return true;
    });

    PrintHelp();
}

void Simulator::Select(int index)
{
    bool status = index < _obstacles.size();
    if(status)
        _current_index = index;

    PrintHelp();

    if(not status)
        Logger::W("Simulator").Print("Selection %d is out of index. (size: %d)", index, _obstacles.size());
}

void Simulator::Add()
{
    static std::normal_distribution<double> distributionLon(0, 40);
    static std::normal_distribution<double> distributionLat(0, 8);
    static std::normal_distribution<double> distributionAngle(0, M_PI * 2);
    static std::default_random_engine generator;

    double lon = distributionLon(generator);
    double lat = distributionLat(generator);
    double x = _reference_pose.x;
    double y = _reference_pose.y;
    double theta = _reference_pose.theta;
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);

    type::Obstacle obstacle;
    obstacle.pose.x = x + lon * cos_theta + lat * sin_theta;
    obstacle.pose.y = y + lon * sin_theta - lat * cos_theta;
    obstacle.pose.theta = distributionAngle(generator);

    SetDefault(obstacle);
    Add(obstacle);
}

void Simulator::Add(const type::Obstacle &obstacle)
{
    static scene::ID id = 0;

    bool status = _obstacles.size() < 10;
    if(status)
    {
        _obstacles.push_back(obstacle);
        _obstacles.back().id = id++;
        _current_index = _obstacles.size() - 1;
    }

    PrintHelp();

    if(status)
    {
        Logger::I("Simulator").Print("Add New Obstacle. [index: %d]", _current_index);
    }
    else
    {
        Logger::W("Simulator").Print("Obstacles are too many. Cannot add new obstacle.");
    }
}

void Simulator::Delete()
{
    _obstacles.erase(_obstacles.begin() + _current_index);
    _current_index = _obstacles.size() - 1;
    PrintHelp();

    Logger::W("Simulator") << "Obstacle List is refreshed.";
}

void Simulator::Clear()
{
    _obstacles.clear();
    PrintHelp();
}

void Simulator::Move(double ds)
{
    _obstacles[_current_index].pose = _obstacles[_current_index].pose.Move(ds);
    PrintHelp();
    RefreshObstacle(_obstacles[_current_index]);
}

void Simulator::Turn(double da)
{
    _obstacles[_current_index].pose.theta = (Radian(_obstacles[_current_index].pose.theta) + Degree(da)).Get<Radian>();
    PrintHelp();
    RefreshObstacle(_obstacles[_current_index]);
}

void Simulator::Shift(double dv)
{
    auto & obstacle = _obstacles[_current_index];
    auto & v = obstacle.v;
    v += dv;
    if(v < 0)
        v = 0;

    RefreshObstacle(obstacle);
    PrintHelp();
}

void Simulator::StretchX(double dlx)
{
    auto & x = _obstacles[_current_index].length.x;
    x += dlx;
    if(x < 1)
        x = 1;
    RefreshObstacle(_obstacles[_current_index]);
    PrintHelp();
}

void Simulator::StretchY(double dly)
{
    auto & y = _obstacles[_current_index].length.y;
    y += dly;
    if(y < 1)
        y = 1;
    RefreshObstacle(_obstacles[_current_index]);
    PrintHelp();
}

void Simulator::PrintHelp()
{
    Console::Screen::Clear();
    Console::ForeColor::Green();

    using std::cout;
    using std::endl;

    cout
        << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl
        << "[   X, x   ]: Add obstacle. At most 10 obstacles." << endl
        << "[   R, r   ]: Clear obstacles." << endl
        << "[   0 - 9  ]: Select obstacle." << endl
        << "[   C, c   ]: Delete the selected obstacle." << endl
        << "[W, A, S, D]: Control the selected obstacle." << endl
        << "[   Q, E   ]: Control the speed of the selected obstacle." << endl
        << "[   N, M   ]: Stretch x direction of the selected obstacle." << endl
        << "[   O, P   ]: Stretch y direction of the selected obstacle." << endl
        << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;

    Console::ForeColor::Purple();

    printf("| No. | %10s | %10s | %10s | %10s | %10s | %10s |\n", "x", "y", "theta", "v", "lx", "ly");

    for(size_t i = 0, size = _obstacles.size(); i < size; ++i)
    {
        auto & o = _obstacles[i];

        if(i == _current_index)
        {
            Console::Screen::Bold();
            Console::ForeColor::Cyan();
        }

        printf("| %3lu | %10.6lf | %10.6lf | %10.6lf | %10.6lf | %10.6lf | %10.6lf |\n",
               i, o.pose.t.x, o.pose.t.y, o.pose.theta * 180.0 / M_PI,
               o.v, o.length.x, o.length.y);

        if(i == _current_index)
        {
            Console::Screen::ClearStyle();
            Console::ForeColor::Purple();
        }
    }

    Console::ForeColor::Green();

    cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;

    Console::Screen::ClearStyle();
}

bool Simulator::ProcessOnobstacle(geometry_msgs::PoseWithCovarianceStamped obstacle,
                                  optional<nox_msgs::ObstacleArray> &obstacles)
{
    Synchronized(this)
    {
        type::Obstacle obstacle_;
        obstacle_.pose.From(obstacle.pose.pose);

        SetDefault(obstacle_);
        Add(obstacle_);
    }

    return true;
}

bool Simulator::ProcessOnvehicle_state(nav_msgs::Odometry vehicle_state, optional<nox_msgs::ObstacleArray> &obstacles)
{
    Synchronized(this)
        _reference_pose.From(vehicle_state.pose.pose);
    return true;
}

void Simulator::Process(optional<geometry_msgs::PoseWithCovarianceStamped> obstacle,
                        optional<nav_msgs::Odometry> vehicle_state, optional<nox_msgs::ObstacleArray> &obstacles)
{
    Synchronized(this)
    {
        obstacles.emplace();

        size_t size = _obstacles.size();
        obstacles.value().obstacles.resize(size);

        for(size_t i = 0; i < size; ++i)
            _obstacles[i].To(obstacles.value().obstacles[i]);
    }
}

void Simulator::SetDefault(type::Obstacle &obstacle)
{
    obstacle.length = {1, 1, 1.5};
    RefreshObstacle(obstacle);
}

void Simulator::RefreshObstacle(type::Obstacle &obstacle)
{
    type::Box box(obstacle.pose, obstacle.length.x, obstacle.length.y);
    obstacle.vertexes = box.Corners();

    obstacle.prediction.Clear();

    if(obstacle.v > 0)
    {
        for(double t : range(0, 1, 8))
        {
            TrajectoryPoint point;
            point.pose = obstacle.pose.Move(t * obstacle.v);
            point.v = obstacle.v;
            point.t = t;
            obstacle.prediction.Add(point);
        }
    }
}


