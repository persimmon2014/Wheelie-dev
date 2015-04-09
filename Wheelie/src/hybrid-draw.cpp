#include "hybrid-sim.hpp"

namespace hybrid
{
    car_interp::car_spatial::car_spatial()
    {
    }

    car_interp::car_spatial::car_spatial(const car &in_c, const hwm::lane *l)
        : c(in_c), la(l)
    {
    }

    car_interp::car_interp(simulator &s)
    {
        times[1]    = s.time;
        car_data[1] = s.get_car_hash();
    }

    void car_interp::capture(simulator &s)
    {
        std::swap(times[0], times[1]);
        std::swap(car_data[0], car_data[1]);

        times[1]    = s.time;
        car_data[1] = s.get_car_hash();
        inv_dt = 1.0/(times[1]-times[0]);
    }

    bool car_interp::in_second(size_t id) const
    {
        car_spatial tmp;
        tmp.c.id = id;
        const car_hash::const_iterator high (car_data[1].find(tmp));
        return high != car_data[1].end();
    }

    float car_interp::acceleration(size_t id, float time) const
    {
        car_spatial tmp;
        tmp.c.id = id;
        const car_hash::const_iterator low (car_data[0].find(tmp));
        const car_hash::const_iterator high(car_data[1].find(tmp));

        assert(low  != car_data[0].end());
        assert(high != car_data[1].end());

        const float w0 = 1 - (time - times[0])*inv_dt;
        const float w1 = 1 - (times[1] - time)*inv_dt;

        return low->c.acceleration*w0 + high->c.acceleration*w1;
    }

    static bool walk_lanes(const hwm::lane                        *&res,
                           float                                   &param,
                           const car_interp::car_hash::value_type  &start,
                           const car_interp::car_hash::value_type  &end,
                           float                                    rel_time)
    {
        std::vector<std::pair<float, const hwm::lane*> > visited;
        visited.push_back(std::make_pair(0, start.la));
        float            distance = start.la->length()*(1-start.c.position);
        {
            const hwm::lane *current  = start.la->downstream_lane();
            while(current != end.la)
            {
                if(!current)
                    return false;
                visited.push_back(std::make_pair(distance, current));
                distance += current->length();
                current  = current->downstream_lane();
            }
            visited.push_back(std::make_pair(distance, current));
            distance += end.la->length()*end.c.position;
        }

        const float target_distance = rel_time*distance;
        res                         = 0;
        param                       = 0;
        for(size_t i = 1; i < visited.size(); ++i)
        {
            if(target_distance < visited[i].first)
            {
                res   = visited[i-1].second;
                param = (target_distance-visited[i-1].first)/res->length();
                break;
            }
        }
        if(!res)
        {
            res   = end.la;
            param = (target_distance-visited.back().first)/res->length();
        }
        if(res == start.la)
            param += start.c.position;
        return true;
    }

    mat4x4f car_interp::point_frame(size_t id, float time, float lane_width) const
    {
        car_spatial tmp;
        tmp.c.id = id;
        const car_hash::const_iterator low (car_data[0].find(tmp));
        const car_hash::const_iterator high(car_data[1].find(tmp));

        assert(low  != car_data[0].end());
        assert(high != car_data[1].end());

        const float w0 = 1 - (time - times[0])*inv_dt;
        const float w1 = 1 - (times[1] - time)*inv_dt;

        float theta0, theta1;
        const vec3f position0(low->c.point_theta(theta0, low->la, lane_width));
        const vec3f position1(high->c.point_theta(theta1, high->la, lane_width));

        if(theta1 - theta0 > M_PI)
            theta0 += 2*M_PI;
        else if(theta0 - theta1 > M_PI)
            theta1 += 2*M_PI;

        mat4x4f res(axis_angle_matrix(theta0*w0 + theta1*w1, vec3f(0.0, 0.0, 1.0)));
        for(int i = 0; i < 3; ++i)
            res(i, 3) = position0[i]*w0 + position1[i]*w1;
        return res;
    }
}
