#include "hybrid-sim.hpp"

namespace hybrid
{
  float boundaries::get_dist_out(const lane* lane_p, float car_len) const
  {
    float distance;
    if (out_bounds.find(lane_p)                     != out_bounds.end()
        and out_bounds.find(lane_p)->second.density != -1)
    {
      float density = out_bounds.find(lane_p)->second.density;

      // Car length per car.
      float m_per_cm = 1.0f / density;

      // Meters per car.
      float m_per_c = m_per_cm * car_len;

      // Minus space occupied by car.
      float sep_dist = m_per_c - car_len;

      // For interfacing reasons...
      sep_dist += car_len; // Just to emphasize that the car_length is included in the distance here.  (It will be removed later if acceleration is being calculated.)

      //The distance between cars given the density specified
      distance = sep_dist;
    }
    else
    {   
      // TODO weizi: HARD CODE!!
      float rear_bumper_rear_axle = 1;     
      if(lane_p->parent->end->network_boundary())
	distance = (1.0 - lane_p->current_cars().back().position) * lane_p->length - rear_bumper_rear_axle; 
      else
        distance = 18;
      
      
      std::cerr << "Warning (get_dist_out): outbound not defined for " << lane_p << std::endl;
      //assert(0);
      //distance = 18; // 4 car length
    }
    return distance;
  }

  float  boundaries::get_vel_out(const lane* lane_p) const
  {
    float velocity;
    if (out_bounds.find(lane_p)                     != out_bounds.end()
        and out_bounds.find(lane_p)->second.density != -1)
    {
      velocity = out_bounds.find(lane_p)->second.velocity;
    }
    else
    {
      // weizi: newly added
      if(lane_p->parent->end->network_boundary())
	velocity = 0;
      else
        velocity = 25;
      std::cerr << "Warning (get_vel_out): outbound not defined for " << lane_p << std::endl;
      //assert(0);
    }
    return velocity;
  }

  float boundaries::get_dist_in(const lane* lane_p, float car_len) const
  {
    float distance;
    if (in_bounds.find(lane_p)                     != in_bounds.end()
        and in_bounds.find(lane_p)->second.density != -1)
    {
      float density = in_bounds.find(lane_p)->second.density;

      float m_per_cm = 1.0f / density;
      float m_per_c = m_per_cm * car_len;
      float sep_dist = m_per_c - car_len;
      sep_dist += car_len; // Just to emphasize that the car_length is included in the distance here.  (It will be removed later if acceleration is being calculated.)

      //The distance between cars given the density specified
      distance = sep_dist;
    }
    else
    {
      std::cerr << "Warning (get_dist_in): outbound not defined for " << lane_p << std::endl;
      //assert(0);
      distance = 18;
    }
    return distance;
  }

  float  boundaries::get_vel_in(const lane* lane_p) const
  {
    float velocity;
    if (in_bounds.find(lane_p)                     != in_bounds.end()
        and in_bounds.find(lane_p)->second.density != -1)
    {
      velocity = in_bounds.find(lane_p)->second.velocity;
    }
    else
    {
      std::cerr << "Warning: inbound not defined for " << lane_p << std::endl;
      //assert(0);
      velocity = 26;
    }
    return velocity;
  }

  float  boundaries::get_rate(const lane* lane_p, const float car_length) const
  {
    float rate;

    if (in_bounds.find(lane_p)                     != in_bounds.end()
        and in_bounds.find(lane_p)->second.density != -1)
    {
      float b_velocity = in_bounds.find(lane_p)->second.velocity;
      float b_density = in_bounds.find(lane_p)->second.density;
      rate = b_density * b_velocity * (1.0 / car_length);
    }
    else
    {
      std::cerr << "Warning: inbound not defined for " << lane_p << std::endl;
      rate = 0.05;
    }
    return rate;
  }



  lane::serial_state::serial_state()
  {
  }

  lane::serial_state::serial_state(const lane &l) : cars(l.current_cars()),
                                                    sim_type(l.sim_type)
  {
    assert(l.next_cars().empty());
  }

  void lane::serial_state::apply(lane &l) const
  {
    l.sim_type       = sim_type;
    l.current_cars() = cars;
  }

  lane::lane() : parent(0), N(0), q(0), rs(0)
  {
  }

  void lane::initialize(hwm::lane *in_parent)
  {
    parent             = in_parent;
    parent->user_datum = this;
    length             = parent->length();
    inv_length         = 1.0f/length;
  }

  struct car_sort
  {
    inline bool operator()(const car &l, const car &r)
    {
      return l.position < r.position;
    }
  };

  void lane::car_swap()
  {
    cars[0].swap(cars[1]);
    cars[1].clear();
    std::sort(cars[0].begin(), cars[0].end(), car_sort());
  }

  bool lane::is_micro() const
  {
    return sim_type == MICRO;
  }

  bool lane::is_macro() const
  {
    return sim_type == MACRO;
  }

  bool lane::occupied() const
  {
    switch(sim_type)
    {
    case MICRO:
      return !current_cars().empty() || !next_cars().empty();
    case MACRO:
      for(size_t i = 0; i < N; ++i)
      {
        if(q[i].rho() > 3*arz<float>::epsilon())
          return true;
      }
      return false;
    default:
      assert(0);
      return false;
    }
  }

  bool lane::active() const
  {
    return parent->active;
  }

  void lane::convert(const sim_t dest_type, simulator &sim)
  {
    switch(dest_type)
    {
    case MICRO:
      convert_to_micro(sim);
      break;
    case MACRO:
      convert_to_macro(sim);
      break;
    default:
      assert(0);
      return;
    }
  };

  void lane::convert_to_micro(simulator &sim)
  {
    if(sim_type == MICRO)
      return;

    if(!fictitious)
      macro_instantiate(sim);

    sim_type = MICRO;
  }

  void lane::convert_to_macro(simulator &sim)
  {
    if(sim_type == MACRO)
      return;

    sim_type = MACRO;

    if(fictitious)
      return;

    clear_macro();
    convert_cars(sim);
    fill_y(sim.gamma);

    current_cars().clear();
    next_cars().clear();
  }

  lane *lane::left_adjacency(float &param)
  {
    hwm::lane *l(parent->left_adjacency(param));
    return l ? l->user_data<lane>() : 0;
  }

  lane *lane::right_adjacency(float &param)
  {
    hwm::lane *l(parent->right_adjacency(param));
    return l ? l->user_data<lane>() : 0;
  }

  lane *lane::upstream_lane()
  {
    hwm::lane *l(parent->upstream_lane());
    return l ? l->user_data<lane>() : 0;
  }

  const lane *lane::upstream_lane() const
  {
    const hwm::lane *l(parent->upstream_lane());
    return l ? l->user_data<const lane>() : 0;
  }

  lane *lane::downstream_lane()
  {
    hwm::lane *l(parent->downstream_lane());
    return l ? l->user_data<lane>() : 0;
  }

  const lane *lane::downstream_lane() const
  {
    const hwm::lane *l(parent->downstream_lane());
    return l ? l->user_data<const lane>() : 0;
  }

  lane::serial_state lane::serial() const
  {
    return serial_state(*this);
  }

  void lane::distance_to_car(float &distance, float &velocity, const float distance_max, const simulator &sim) const
  {
    switch(sim_type)
    {
      case MICRO:
	return micro_distance_to_car(distance, velocity, distance_max, sim);
      case MACRO:
	return macro_distance_to_car(distance, velocity, distance_max, sim);
      default:
	assert(0);
	return;
    }
  }

  simulator::serial_state::serial_state() : q_base(0)
  {
  }

  simulator::serial_state::~serial_state()
  {
    if(q_base)
      delete[] q_base;
  }

  simulator::serial_state::serial_state(const simulator &s) : car_id_counter(s.car_id_counter),
                                                              generator(*s.generator),
                                                              network_state(s.hnet->serial())
  {
    lane_states.reserve(s.lanes.size());
    for(const lane &l: s.lanes)
    {
      lane_states.push_back(l.serial());
    }

    q_base = new arz<float>::q[s.N];
    std::memcpy(q_base, s.q_base, s.N*sizeof(sizeof(arz<float>::q)));
  }

  car_interp::car_hash simulator::get_car_hash() const
  {
    car_interp::car_hash res;
    for(const lane &l: lanes)
    {
      if(!l.parent->active || !l.is_micro())
        continue;

      for(const car &c: l.current_cars())
      {
        res.insert(car_interp::car_spatial(c, l.parent));
      }
    }
    return res;
  }

  void simulator::serial_state::apply(simulator &s) const
  {
    s.car_id_counter = car_id_counter;
    *s.generator     = generator;
    network_state.apply(*s.hnet);
    assert(q_base);
    std::memcpy(s.q_base, q_base, s.N*sizeof(sizeof(arz<float>::q)));
  }

  simulator::simulator(hwm::network *net, float length, float rear_axle)
    : hnet(net),
      car_length(length),
      rear_bumper_rear_axle(rear_axle),
      time(0.0f),
      car_id_counter(1),
      q_base(0),
      rs_base(0)
  {    
    assert(hnet);
    
    // Initialize micro parameters
    generator = new base_generator_type(42ul);
    uni_dist  = new boost::uniform_real<>(0,1);
    uni       = new boost::variate_generator<base_generator_type&, boost::uniform_real<> >(*generator, *uni_dist);

    // figure out how many lanes to create
    size_t lane_count = hnet->lanes.size();
    for(const hwm::intersection_pair &ip: hnet->intersections)
    {
      for(const hwm::intersection::state &current_state: ip.second.states)
      {
        lane_count += current_state.fict_lanes.size();
      }
    }

    // Intiailize lanes structure which should first consist of actual lanes and then fictitious lanes
    lanes.resize(lane_count);
    float                       min_len         = std::numeric_limits<float>::max();
    float                       max_speedlimit  = -std::numeric_limits<float>::max();
    
    // First Intiailize actual lanes
    std::vector<lane>::iterator current         = lanes.begin();
    for(hwm::lane_map::iterator hwm_current = hnet->lanes.begin();
        hwm_current                        != hnet->lanes.end() && current != lanes.end();
        ++current, ++hwm_current)
    {
      //std::cout<<hwm_current->second.id<<std::endl;
      current->initialize(&(hwm_current->second));
      current->fictitious = false;
      
      //std::cout<<current->length<<std::endl;
      min_len             = std::min(current->length, min_len);
      max_speedlimit      = std::max(current->speedlimit(), max_speedlimit);
    }

    
    if(min_len > 1e+5) {
      std::cerr<<"Min length of regular lane is too big, check code instead!"<<std::endl;
      exit(0);
      
    }
    
    std::cout << "Min length of regular lane is: " << min_len << std::endl;
    std::cout << "Max speedlimit of regular lane is: " << max_speedlimit << std::endl;

    // Secondly initialize fictitious lanes
    min_len = std::numeric_limits<float>::max();
    for(hwm::intersection_pair &ip: hnet->intersections)
    {
      for(hwm::intersection::state &current_state: ip.second.states)
      {
        for(hwm::lane_pair &lp: current_state.fict_lanes)
        {
          assert(current != lanes.end());
          current->initialize(&(lp.second));
          current->fictitious = true;
          min_len             = std::min(current->length, min_len);
          ++current;
        }
      }
    }
    std::cout << "Min length of fict lane  is: " << min_len << std::endl;

    sec_since_car = 0;
  }

  simulator::~simulator()
  {
    delete generator;
    delete uni_dist;
    delete uni;

    micro_cleanup();
    macro_cleanup();
  }

  float simulator::rear_bumper_offset() const
  {
    return -rear_bumper_rear_axle;
  }

  float simulator::front_bumper_offset() const
  {
    return car_length-rear_bumper_rear_axle;
  }

  void simulator::car_swap()
  {
    for(lane &l: lanes)
    {
      l.car_swap();
    }
  }

  
  /**
   * make a car
   * @param position car real axle position / lane length
   * @param velocity car velocity
   * @param acceleration car acceleration
   * @return the car instance
   */
  car simulator::make_car(const float position, const float velocity,
                          const float acceleration)
  {
    return car(car_id_counter++, position, velocity, acceleration);
  }

  lane &simulator::get_lane_by_name(const str &s)
  { 
    const hwm::lane_map::iterator res(hnet->lanes.find(s));
    if(res == hnet->lanes.end())
      throw std::runtime_error("No such lane!");
    return *(res->second.user_data<lane>());
  }

  const lane &simulator::get_lane_by_name(const str &s) const
  {
    const hwm::lane_map::iterator res(hnet->lanes.find(s));
    if(res == hnet->lanes.end())
      throw std::runtime_error("No such lane!");
    return *(res->second.user_data<const lane>());
  }

  size_t simulator::ncars() const
  {
    size_t res = 0;
    for(const lane &l: lanes)
    {
      res += l.ncars();
    }
    return res;
  }

  void simulator::mass_reassign(std::vector<hwm::network_aux::road_spatial::entry> &qr)
  {
    for(lane &l: lanes)
    {
      l.updated_flag = false;
    }
    for(hwm::network_aux::road_spatial::entry &e: qr)
    {
      for(hwm::network_aux::road_rev_map::lane_cont::value_type &lcv: *e.lc)
      {
        hwm::lane    &hwm_l = *(lcv.second.lane);
        hybrid::lane &hyb_l = *(hwm_l.user_data<lane>());
        if(!hyb_l.updated_flag && hyb_l.active())
        {
          hyb_l.convert_to_micro(*this);
          hyb_l.updated_flag = true;
        }
      }
    }
    for(hwm::intersection_pair &ip: hnet->intersections)
    {
      for(hwm::intersection::state &s: ip.second.states)
      {
        for(hwm::lane_pair &lp: s.fict_lanes)
        {
          lane &hyb_l = *(lp.second.user_data<lane>());
          if(!hyb_l.updated_flag && hyb_l.active())
          {
            lane *up_l(hyb_l.upstream_lane());
            lane *dn_l(hyb_l.downstream_lane());
            if((up_l && up_l->updated_flag && up_l->is_micro()) ||
               (dn_l && dn_l->updated_flag && dn_l->is_micro()))
            {
              hyb_l.convert_to_micro(*this);
              hyb_l.updated_flag = true;
            }
          }
        }
      }
    }

    for(lane &l: lanes)
    {
      if(!l.updated_flag && l.active())
      {
        l.convert_to_macro(*this);
        l.updated_flag = true;
      }
    }
  }

  float simulator::micro_step(float dt)
  {
    // Microsimulation step.
    update(dt);

    time += dt;

    apply_incoming_bc(dt, time);

    car_swap();

    return dt;
  }

  float simulator::carticle_step(float dt)
  {
    // Microsimulation step.
    carticle_update(dt);

    time += dt;

    apply_incoming_bc(dt, time);

    car_swap();

    return dt;
  }

  float simulator::hybrid_step()
  {
    // fill in micro
    //convert_cars(MICRO);

    // macro step (also emit cars)
    float dt = macro_step(1.0f);

    // micro step
    update(dt);

    time += dt;
    apply_incoming_bc(dt, time);

    car_swap();

    return dt;
  }

  void simulator::advance_intersections(float dt)
  {
    for(hwm::intersection_pair &ip: hnet->intersections)
    {
      hwm::intersection &i = ip.second;

      i.state_time += dt;
      if(i.locked || i.state_time > i.states[i.current_state].duration)
      {
        bool free = true;
        for(const hwm::lane_pair &lp: i.states[i.current_state].fict_lanes)
        {
          if(lp.second.user_data<lane>()->occupied())
          {
            free = false;
            break;
          }
        }

        if(free)
        {
          i.unlock();
          i.advance_state();
        }
        else
          i.lock();
      }
    }
  }

  void simulator::old_apply_incoming_bc(float dt, float t)
  {
    bool POISSON = false;
    static const float MIN_SPEED_FRACTION = 0.7;
    static float rate               = 0.05;
    for(lane &l: lanes)
    {
      if(l.is_micro() && l.parent->start->network_boundary())
      {
        rate = boundary_conditions.get_rate(&l, car_length);

        car new_car;
        if(l.current_cars().empty())
        {
          const float add_prob     = (*uni)();
          const float prob_of_none = std::exp(-rate*dt);
          if(add_prob <= prob_of_none)
            continue;

          new_car = make_car(0, std::max((float)(*uni)(), MIN_SPEED_FRACTION)*l.speedlimit(), 0);
          new_car.compute_intersection_acceleration(*this, l);

          l.next_cars().push_back(new_car);

          std::cout << "Added car. " << std::endl;
        }
        else
        {
          if (POISSON)
          {
            const car &leader = l.current_car(0);
            if(!(leader.position * l.length > 1.5*car_length))
              continue;

            const float add_prob     = (*uni)();
            const float prob_of_none = std::exp(-rate*dt);

            if(add_prob <= prob_of_none)
              continue;

            new_car           = make_car(0, leader.velocity, 0);

            new_car.acceleration = new_car.car_accel(leader.velocity,
                                                     new_car.velocity,
                                                     (leader.position - new_car.position)*l.length);
            l.next_cars().push_back(new_car);
          }
          else
          {
            car &leader = l.current_car(0);

            float sec_per_car = 1/rate;

            if (sec_since_car < sec_per_car)
            {
              sec_since_car += dt;
              continue;
            }

            while ((sec_since_car >= sec_per_car) and (leader.position * l.length > 1*(car_length)))
            {
              sec_since_car -= sec_per_car;

              float sep_dist = boundary_conditions.get_dist_out(&l, car_length);

              new_car = make_car(leader.position - (sep_dist / l.length),
                                 boundary_conditions.get_vel_in(&l),
                                 0);

              new_car.acceleration = new_car.car_accel(leader.velocity,
                                                       new_car.velocity,
                                                       (leader.position - new_car.position)*l.length);

              l.next_cars().push_back(new_car);

              leader = new_car;

            }
          }
        }
      }
    }
  }

  namespace
  {
    double _random() {
      return (double) (rand()/(1.0*RAND_MAX));
      //  return (double) (rand()*(RAND_MAX+1) + rand()) / (RAND_MAX*(RAND_MAX + 2));
    }


    double normal() {
      double u, v, s(0);

      while (s == 0 || s > 1) {
        u = 2*_random()-1;
        v = 2*_random()-1;
        s = u*u + v*v;
      }

      return u * sqrt(-2*log(s)/s);
    }

  }

  void simulator::apply_incoming_bc(float dt, float t)
  {
    bool POISSON = false;
    bool NOISE = true;

    float noise_factor = 0.1;
    static const float MIN_SPEED_FRACTION = 0.7;
    static float rate               = 0.05;
    for(lane &l: lanes)
    {
      if (!l.active())
        continue;

      if(l.is_micro() && l.parent->start->network_boundary())
      {
        rate = boundary_conditions.get_rate(&l, car_length);

        car new_car;
        if(l.current_cars().empty()) // RARE
        {
          const float add_prob     = (*uni)();
          const float prob_of_none = std::exp(-rate*dt);
          if(add_prob <= prob_of_none)
            continue;

          new_car = make_car(0, std::max((float)(*uni)(), MIN_SPEED_FRACTION)*l.speedlimit(), 0);
          new_car.compute_intersection_acceleration(*this, l);

          l.next_cars().push_back(new_car);

          std::cout << "Added car. " << std::endl;
        }
        else
        {
          car &leader = l.current_car(0);

          float free_dist = leader.position * l.length;

          while(1)
          {

            float sep_dist = boundary_conditions.get_dist_in(&l, car_length);
            float vel      = boundary_conditions.get_vel_in(&l);

            if (NOISE)
            {
                  float sep_noise = l.next_noise * 2.5;
                  sep_dist += sep_noise;
                  float vel_noise = l.next_noise * 0;
                  vel += vel_noise;

                  float min_dist = car_length + 1.0f + 0.1f*vel;

                  sep_dist = std::max(sep_dist, min_dist);
                  sep_dist = std::min(sep_dist, sep_dist + (sep_dist - min_dist));
            }
            else
            {
                float min_dist = car_length + 1.0f + 0.1f*leader.velocity;
                sep_dist = std::max(sep_dist, min_dist);
            }

            free_dist -= sep_dist;

            float car_pos = leader.position - (sep_dist / l.length);

            // Don't spawn cars in the first cell. (estimated as 10m)
            if (car_pos < (10.0/l.length))
                break;

            car_pos = std::max(car_pos, 0.0f);
            vel = std::max(vel, 0.0f);

            new_car = make_car(car_pos,
                               vel,
                               0);


            float p = _random();
            if (p > 0.85) {
               new_car.r = 1.0;
               new_car.g = 0;
               new_car.b = 0;

               // Assign the special pramaters for this car.
               new_car.p_s1 = stored_parameters.s1;
               new_car.p_delta = stored_parameters.delta;
               new_car.p_T = stored_parameters.T;
               new_car.p_speedlimit_factor = stored_parameters.speedlimit_factor;
               new_car.p_a = stored_parameters.alpha;
               new_car.p_b = stored_parameters.beta;

            } else {
               new_car.r = 0;
               new_car.g = 0;
               new_car.b = 1.0;
            }

            l.next_noise = normal();

            // new_car.acceleration = new_car.car_accel(leader.velocity,
            //                                          new_car.velocity,
            //                                          sep_dist);

            l.next_cars().push_back(new_car);

            leader = new_car;
          }
        }
      }
    }
  }

   void simulator::SetSpecialCarParams(float s1,
                                       float delta,
                                       float T,
                                       float speedlimit_factor,
                                       float alpha,
                                       float beta) {

      stored_parameters.s1 = s1;
      stored_parameters.delta = delta;
      stored_parameters.T = T;
      stored_parameters.speedlimit_factor = speedlimit_factor;
      stored_parameters.alpha = alpha;
      stored_parameters.beta = beta;
   }

  simulator::serial_state simulator::serial() const
  {
    return serial_state(*this);
  }
}
