#include "hybrid-sim.hpp"

namespace hybrid
{
    void lane::macro_initialize(const float h_suggest)
    {
        N = static_cast<size_t>(std::ceil(length/h_suggest));
        assert(N > 0);
        h = length/N;
        inv_h = 1.0f/h;
    }

    struct lane_poisson_helper
    {
        typedef float real_t;

        lane_poisson_helper(const lane &l, const float scale) : dx_(l.h), n_(l.N), q_(l.q), scale_(scale)
        {}

        float operator[](size_t idx) const
        {
            return q_[idx].rho()*scale_;
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
            return n_;
        }

        float          dx_;
        size_t         n_;
        arz<float>::q *q_;
        float          scale_;
    };

    struct lane_poisson_helper_reverse
    {
        typedef float real_t;

        lane_poisson_helper_reverse(const lane &l, const float scale) : dx_(l.h), n_(l.N), q_(l.q), scale_(scale)
        {}

        float operator[](size_t idx) const
        {
            return q_[n_-1-idx].rho()*scale_;
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
            return n_;
        }

        float          dx_;
        size_t         n_;
        arz<float>::q *q_;
        float          scale_;
    };

    void lane::macro_instantiate(simulator &sim)
    {
        typedef pproc::inhomogeneous_poisson<simulator::rand_gen_t, lane_poisson_helper> ih_poisson_t;

        current_cars().clear();

        lane_poisson_helper helper(*this, 1.0f/sim.car_length);
        ih_poisson_t        ip(-sim.rear_bumper_offset(), helper, sim.uni);

        float candidate = ip.next();

        while(candidate < helper.end()-sim.front_bumper_offset())
        {
            const float position = candidate/helper.end();
            current_cars().push_back(sim.make_car(position,
                                                  velocity(position, sim.gamma),
                                                  0.0f));
            current_cars().back().r = 0;
            current_cars().back().g = 0;
            current_cars().back().b = 1;

            while(1)
            {
                ih_poisson_t ip_copy(ip);
                candidate = ip.next();
                if(candidate - current_cars().back().position*helper.end() >= sim.car_length)
                    break;
                ip = ip_copy;
            }
        }
    }

    bool lane::macro_find_first(float &param, const simulator &sim) const
    {
        typedef pproc::inhomogeneous_poisson<simulator::rand_gen_t, lane_poisson_helper> ih_poisson_t;

        lane_poisson_helper helper(*this, 1.0f/sim.car_length);
        ih_poisson_t        ip(-sim.rear_bumper_offset(), helper, sim.uni);

        const float candidate = ip.next();
        if(candidate < helper.end()-sim.front_bumper_offset())
        {
            param = candidate/helper.end();
            return true;
        }
        else
            return false;
    }

    bool lane::macro_find_last(float &param, const simulator &sim) const
    {
        typedef pproc::inhomogeneous_poisson<simulator::rand_gen_t, lane_poisson_helper_reverse> ih_poisson_t;

        lane_poisson_helper_reverse helper(*this, 1.0f/sim.car_length);
        ih_poisson_t                ip(sim.front_bumper_offset(), helper, sim.uni);

        const float candidate = helper.end() - ip.next();
        if(candidate + sim.rear_bumper_offset() > 0)
        {
            param = candidate/helper.end();
            return true;
        }
        else
            return false;
    }

    void lane::macro_distance_to_car(float &distance, float &vel, const float distance_max, const simulator &sim) const
    {
        float param;
        if(!fictitious && macro_find_first(param, sim))
        {
            distance += param*length;
            vel       = velocity(param, sim.gamma);
        }
        else
        {
            distance += length;
            if(distance >= distance_max || parent->end->network_boundary())
            {
                vel      = 0.0f;
                distance = distance_max;
            }
            else
            {
                const lane *downstream = downstream_lane();
                if(downstream)
                    downstream->distance_to_car(distance, vel, distance_max, sim);
                else
                    vel = 0.0f;
            }
        }

        return;
    }

    int lane::which_cell(const float pos) const
    {
        return static_cast<int>(std::floor(pos*N));
    }

    float lane::velocity(const float pos, const float gamma) const
    {
        const int   cell  = which_cell(pos);
        const float local = pos*N - cell;
        assert(cell >= 0);
        assert(cell < static_cast<int>(N));

        const arz<float>::full_q q_c(q[cell], speedlimit(), gamma);
        if(local < 0.5f)
        {
            if(cell == 0)
                return q_c.u();

            const arz<float>::full_q q_c_m(q[cell-1], speedlimit(), gamma);
            return (local+0.5f) * q_c.u() + (0.5f-local) * q_c_m.u();
        }
        //local >= 0.5f

        if(cell == static_cast<int>(N)-1)
            return q_c.u();

        const arz<float>::full_q q_c_p(q[cell+1], speedlimit(), gamma);
        return (1.5f-local) * q_c.u() + (local-0.5f) * q_c_p.u();
    }

    float lane::collect_riemann(const float gamma, const float inv_gamma)
    {
        arz<float>::full_q  full_q_buff[2];
        arz<float>::full_q *fq[2] = { full_q_buff, full_q_buff + 1 };

        const float my_speedlimit  = speedlimit();
        const float inv_speedlimit = 1.0f/my_speedlimit;

        *fq[0] = arz<float>::full_q(q[0],
                                    my_speedlimit,
                                    gamma);

        float maxspeed = 0.0f;
        lane *upstream = upstream_lane();
        if(!upstream)
        {
            rs[0].starvation_riemann(*fq[0],
                                     my_speedlimit,
                                     inv_speedlimit,
                                     gamma,
                                     inv_gamma);
            maxspeed = std::max(rs[0].speeds[1], maxspeed);
        }
        else
        {
            if(upstream->fictitious)
            {
                lane* next_upstream = upstream->upstream_lane();
                assert(next_upstream);
                assert(!next_upstream->fictitious);
                upstream = next_upstream;
            }

            const arz<float>::full_q us_end(upstream->q[upstream->N-1],
                                            my_speedlimit,
                                            gamma);

            if(upstream->speedlimit() == my_speedlimit)
                rs[0].riemann(us_end,
                              *fq[0],
                              my_speedlimit,
                              inv_speedlimit,
                              gamma,
                              inv_gamma);
            else
                rs[0].lebaque_inhomogeneous_riemann(us_end,
                                                    *fq[0],
                                                    upstream->speedlimit(),
                                                    my_speedlimit,
                                                    gamma,
                                                    inv_gamma);

            maxspeed = std::max(std::max(std::abs(rs[0].speeds[0]), std::abs(rs[0].speeds[1])),
                                maxspeed);
        }

        assert(rs[0].check());

        for(size_t i = 1; i < N; ++i)
        {
          *fq[1] = arz<float>::full_q(q[i],
                                      my_speedlimit,
                                      gamma);

          rs[i].riemann(*fq[0],
                        *fq[1],
                        my_speedlimit,
                        inv_speedlimit,
                        gamma,
                        inv_gamma);

          assert(rs[i].check());

          maxspeed = std::max(std::max(std::abs(rs[i].speeds[0]), std::abs(rs[i].speeds[1])),
                              maxspeed);
          std::swap(fq[0], fq[1]);
        }

        if(parent->end->network_boundary())
        {
            rs[N].clear();
        }
        else
        {
            lane *downstream = downstream_lane();
            if(!downstream)
            {
                rs[N].stop_riemann(*fq[0],
                                   my_speedlimit,
                                   inv_speedlimit,
                                   gamma,
                                   inv_gamma);
                maxspeed = std::max(std::abs(rs[N].speeds[0]), maxspeed);
            }
            else
            {
                if(downstream->fictitious)
                {
                    lane* next_downstream = downstream->downstream_lane();
                    assert(next_downstream);
                    assert(!next_downstream->fictitious);
                    downstream = next_downstream;
                }

                const arz<float>::full_q ds_start(downstream->q[0],
                                                  downstream->speedlimit(),
                                                  gamma);

                if(my_speedlimit == downstream->speedlimit())
                    rs[N].riemann(*fq[0],
                                  ds_start,
                                  my_speedlimit,
                                  inv_speedlimit,
                                  gamma,
                                  inv_gamma);
                else
                    rs[N].lebaque_inhomogeneous_riemann(*fq[0],
                                                        ds_start,
                                                        my_speedlimit,
                                                        downstream->speedlimit(),
                                                        gamma,
                                                        inv_gamma);

                maxspeed = std::max(std::max(std::abs(rs[N].speeds[0]), std::abs(rs[N].speeds[1])),
                                    maxspeed);
            }
        }

        assert(rs[N].check());

        return maxspeed;
    }

    void lane::update(const float dt, simulator &sim)
    {
        const float coefficient = dt*inv_h;

        //TODO(wilkie) Implement the boundaries the right way  for(size_t i = 0; i < N; ++i)
        for(size_t i = 1; i < N - 1; ++i)
        {
          q[i]     -= coefficient*(rs[i].right_fluctuation + rs[i+1].left_fluctuation);
          q[i].y() -= q[i].y()*coefficient*sim.relaxation_factor;
          q[i].fix();
        }
        lane *downstream = downstream_lane();
        if(downstream)
        {
            float param;
            if(macro_find_last(param, sim))
            {
                car c(0, param, velocity(param, sim.gamma), 0.0f);
                c.compute_intersection_acceleration(sim, *this);
                c.integrate(dt, *this, sim.hnet->lane_width);

                const int cell(std::min(which_cell(c.position), static_cast<int>(N)-1));
                assert(cell >=0);
                q[cell] = arz<float>::q(q[cell].rho(), c.velocity, speedlimit(), sim.gamma);
                if(c.position >= 1.0)
                {
                    c.position = length*downstream->inv_length*(c.position-1.0f);
                    downstream->next_cars().push_back(sim.make_car(c.position, c.velocity, c.acceleration));
                }
            }
        }
    }

    void lane::clear_macro()
    {
        memset(q, 0, sizeof(arz<float>::q)*N);
    }

    void lane::convert_cars(const simulator &sim)
    {
        for(car &c: current_cars())
        {
	    //std::cout<<"car_back: "<<sim.rear_bumper_offset()<<" car_front: "<<sim.front_bumper_offset()<<std::endl;
	  
	    // car back bumper and front bumper positions, c.position is a relative value comparing to the lane length
            const float car_back  = c.position+sim.rear_bumper_offset()*inv_length;
            const float car_front = c.position+sim.front_bumper_offset()*inv_length;
	    
            const int start_cell = which_cell(car_back);
            const int end_cell   = which_cell(car_front);
	    // std::cout<<"start_cell: "<<start_cell<<" end_cell: "<<end_cell<<std::endl;
	    
	    // std::cout<<"c.velocity: "<<c.velocity<<std::endl;
            if(start_cell == end_cell)
            {
                float coverage       = sim.car_length*inv_h; // car length / cell size
                q[start_cell].rho() += coverage; // unit: cars per car length
                q[start_cell].y()   += coverage*c.velocity;
		//std::cout<<"q[start_cell].y(): "<<q[start_cell].y()<<std::endl;
                continue;
            }

            if(start_cell >= 0)
            {
                float start_coverage = (start_cell+1) - car_back*N;
		//std::cout<<"s_c: "<<start_cell+1<<" "<<car_back*N<<" "<<(start_cell+1) - car_back*N<<std::endl;
                q[start_cell].rho() += start_coverage;
                q[start_cell].y()   += start_coverage*c.velocity;
		//std::cout<<"q[start_cell].y(): "<<q[start_cell].y()<<std::endl;
            }

            // unlikely to be executed since car is smaller than half length of a cell size
            for(int i = start_cell+1; i < end_cell-1; ++i)
            {
                q[i].rho() = 1.0f;
                q[i].y()   = c.velocity;
            }

            if(end_cell < static_cast<int>(N))
            {
                float end_coverage  = car_front*N - end_cell;
                q[end_cell].rho()  += end_coverage;
                assert(q[end_cell].rho() <= 1.0f);
                q[end_cell].y()    += end_coverage*c.velocity;
		//std::cout<<"q[end_cell].y() : "<<q[end_cell].y()<<std::endl;
            }
        }
    }

    void lane::fill_y(const float gamma)
    {
        for(size_t i = 0; i < N; ++i)
        {
            if(q[i].rho() > 0.0) {
                q[i].y() /= q[i].rho(); // to calculate u, which should be the second parameter to the following equation
	    }
	    
            q[i].y() = arz<float>::eq::y(q[i].rho(),
                                         q[i].y(),
                                         speedlimit(),
                                         gamma);
	    
	    //std::cout<<"Assign y for each lane: "<<q[i].rho()<<","<<q[i].y()<<","<<speedlimit()<<","<<gamma<<std::endl;
            q[i].fix();
        }
    }

    void simulator::macro_re_initialize(const float in_gamma, const float h_suggest, const float rf)
    {
        // Try to stay out of FP_ASSIST - enable DAZ and FTZ
        // _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
        // _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

        relaxation_factor = rf;
        min_h             = std::numeric_limits<float>::max();

        N = 0;
        // initialize new lanes, compute how many cells to allocate
        for(lane &l: lanes)
        {
            if(l.fictitious)
                continue;

            N += l.N;
            min_h = std::min(min_h, l.h);
        }

        memset(q_base, 0, sizeof(arz<float>::q)*N);
        memset(rs_base, 0, sizeof(arz<float>::riemann_solution)*(N+lanes.size()));

        size_t q_count  = 0;
        size_t rs_count = 0;
        for(lane &l: lanes)
        {
            if(l.fictitious)
                continue;

            l.q       = q_base + q_count;
            l.rs      = rs_base + rs_count;
            q_count  += l.N;
            rs_count += l.N + 1;

            // if(l.N > 5)
            // {
            //     l.q[2].rho() = 0.5;
            //     l.q[2].y()   = 1.5;
            // }
            l.fill_y(gamma);
        }
        std::cout << "min_h is " << min_h << std::endl;
    }


    void simulator::macro_initialize(const float in_gamma, const float h_suggest, const float rf)
    {
        // Try to stay out of FP_ASSIST - enable DAZ and FTZ
        // _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
        // _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

        gamma             = in_gamma;
        relaxation_factor = rf;
        min_h             = std::numeric_limits<float>::max();

        N = 0;
	
        // Initialize new lanes, compute how many cells to allocate
        for(lane &l: lanes)
        {
            if(l.fictitious)
                continue;
            l.macro_initialize(h_suggest); // specify number of cells for a lane
            N += l.N;
            min_h = std::min(min_h, l.h); // h is the actuall cell size
        }

        std::cout << "Allocating " << sizeof(arz<float>::q)*N <<  " bytes for " << N << " cells...";
        q_base = (arz<float>::q *) malloc(sizeof(arz<float>::q)*N);
	
        if(!q_base)
            throw std::exception();
        std::cout << "Done." << std::endl;

        std::cout << "Allocating " << sizeof(arz<float>::riemann_solution)*(N+lanes.size()) <<  " bytes for " << N+lanes.size() << " riemann solutions...";
        rs_base = (arz<float>::riemann_solution *) malloc(sizeof(arz<float>::riemann_solution)*(N+lanes.size()));
        if(!rs_base)
            throw std::exception();
        std::cout << "Done." << std::endl;

        memset(q_base, 0, sizeof(arz<float>::q)*N);
        memset(rs_base, 0, sizeof(arz<float>::riemann_solution)*(N+lanes.size()));

        size_t q_count  = 0;
        size_t rs_count = 0;
	
	// set memory location for q and rs of each lane
	
	std::cout << "In macro_initialize..."<< std::endl;
        for(lane &l: lanes)
        {
            if(l.fictitious)
                continue;

            l.q       = q_base + q_count;
            l.rs      = rs_base + rs_count;
            q_count  += l.N;
            rs_count += l.N + 1;

            // if(l.N > 5)
            // {
            //     l.q[2].rho() = 0.5;
            //     l.q[2].y()   = 1.5;
            // }
	    
	    // weizi: is this the place to fill all different velocity?
            l.fill_y(gamma);
        }
        std::cout << "min_h is " << min_h << std::endl;
    }

    void simulator::macro_cleanup()
    {
        free(q_base);
        free(rs_base);
    }

    void simulator::convert_cars(const sim_t sim_mask)
    {
        // MACRO = 1, MICRO = 2
        // operate on the same type of simulator and sim_mask
        for(lane &l: lanes)
        {
	    //std::cout<<l.sim_type<<" "<<sim_mask<<" "<<(l.sim_type & sim_mask)<<std::endl;
            if(l.sim_type & sim_mask && !l.fictitious)
                l.clear_macro();
        }
        for(lane &l: lanes)
        {
            if(l.sim_type & sim_mask && !l.fictitious)
                l.convert_cars(*this);
        }
        for(lane &l: lanes)
        {
            if(l.sim_type & sim_mask && !l.fictitious)
                l.fill_y(gamma);
        }
    }

    float simulator::macro_step(const float cfl)
    {
        float maxspeed = 0.0f;
        for(lane &l: lanes)
        {
          if(l.is_macro() && l.parent->active && !l.fictitious)
          {
            maxspeed = std::max(l.collect_riemann(gamma, 1.0f/gamma),
                                maxspeed);
          }
        }

        if(maxspeed < arz<float>::epsilon())
            maxspeed = min_h;

        const float dt = std::min(cfl*min_h/maxspeed, 0.5f);

        for(lane &l: lanes)
        {
          if(l.is_macro() && l.parent->active && !l.fictitious)
            l.update(dt, *this);
        }

        return dt;
    }
};
