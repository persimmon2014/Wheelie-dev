#include "libhybrid/pc-poisson.hpp"
#include <iostream>
#include <boost/random.hpp>

struct helper
{
    typedef float real_t;

    helper(float dx, const std::vector<float> &data) : dx_(dx), data_(data)
    {}

    float operator[](size_t idx) const
    {
        return data_[idx];
    }

    float dx() const
    {
        return dx_;
    }

    float inf() const
    {
        return 0;
    }

    float end() const
    {
        return n()*dx();
    }

    size_t n() const
    {
        return data_.size();
    }

    float              dx_;
    size_t             n_;
    std::vector<float> data_;
};

struct drand
{
    float operator()() const
    {
        return drand48();
    }
};

int main(int argc, char **argv)
{
    const float car_length = 2*4.5;
    const float dx = 10.0f;

    std::vector<float> data;

    float x = 0;
    for(size_t i = 0; i < 200; ++i)
    {
        float v = 0.15*( (std::cos(x*0.008)+1)*(0.5*(std::cos(0.0001*x-0.5)+1)) + 0.2*(std::cos(x*0.03+0.1)+1))/car_length;
        data.push_back(v);
        x += dx;
    }
    srand48(11);
    drand randr;

    typedef pproc::inhomogeneous_poisson<drand, helper> ih_poisson_t;
    helper                                                   h(dx, data);
    ih_poisson_t                                             ip(0, h, &randr);

    std::cout << data.size() << " " << dx << " " << " ";
    for(size_t i = 0; i < data.size(); ++i)
        std::cout << data[i] << " ";
    std::cout << 0 << std::endl;

    const float sep      = 0.0f;//car_length;

    float candidate = ip.next();

    float last = -std::numeric_limits<float>::max();
    while(candidate < h.end())
    {
        const float position = candidate;
        // current_cars().push_back(sim.make_car(position,
        //                                       velocity(position, sim.gamma),
        //                                           0.0f));

        last = position;
        std::cout << position << " ";

        while(1)
        {
            ih_poisson_t ip_copy(ip);
            candidate = ip.next();
            if(candidate - last >= sep)
                break;
            ip = ip_copy;
        }
    }

    std::cout << std::endl;

    return 0;
};
