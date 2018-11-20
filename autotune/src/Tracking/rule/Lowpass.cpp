#include <Tracking/rule/Lowpass.h>

namespace nox::app::rule
{
    Lowpass::Lowpass(size_t bufferSize, double init_data)
    {
        _buffer.SetBufferSize(bufferSize);
        _buffer.Fill(init_data);
    }

    double Lowpass::Apply(double raw)
    {
        auto size = _buffer.GetBufferSize();
        if(size == 0)
            return raw;

        _buffer.Push(raw);
        double min_data = real::MAX;
        double max_data = -real::MAX;
        double sum_data = 0;

        for(double i : _buffer)
        {
            min_data = std::min(min_data, i);
            max_data = std::max(max_data, i);
            sum_data += i;
        }

        if(size > 3)
            return (sum_data - min_data - max_data) / (size - 2);
        else
            return sum_data / size;
    }
}

