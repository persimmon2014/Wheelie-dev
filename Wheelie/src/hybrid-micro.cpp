 #include "hybrid-sim.hpp"

namespace hybrid
{
    struct lc_curve
    {
        lc_curve(float target_y) : y_(target_y) {}

        inline float operator()(float t) const
        {
            return y(t) - y_;
        }

        static inline float y(float t)
        {
            return ((((8.7025182813*t+
                       -21.7562980512)*t +
                      15.5458393998)*t +
                     -1.56245957436)*t +
                    0.0709664576132)*t +
            -0.000283090356282;
        }

        static inline float end(float speed)
        {
            return 11.50731f*std::pow(speed, -0.5f);
        }

        float y_;
    };

    mat4x4f car::point_frame(const hwm::lane* l, const float lane_width) const
    {
        mat4x4f trans;
        float pos[4];
        if (other_lane_membership.other_lane != 0)
        {
            float offset  = std::min(lc_curve::y(other_lane_membership.merge_param), (float)1.0);
            offset       *= lane_width;
            if (!other_lane_membership.is_left)
            {
                offset *= -1;
            }

            mat4x4f rotation;
            rotation(0, 0) = std::cos(other_lane_membership.theta);  rotation(0, 1) = -1*std::sin(other_lane_membership.theta);  rotation(0, 2) = 0;    rotation(0, 3) = 0;
            rotation(1, 0) = std::sin(other_lane_membership.theta);  rotation(1, 1) = std::cos(other_lane_membership.theta);     rotation(1, 2) = 0;    rotation(1, 3) = 0;
            rotation(2, 0) = 0;                rotation(2, 1) = 0;                   rotation(2, 2) = 1.0f; rotation(2, 3) = 0;
            rotation(3, 0) = 0.0f;             rotation(3, 1) = 0.0f;                rotation(3, 2) = 0.0f; rotation(3, 3) = 1.0f;

            trans = mat4x4f(other_lane_membership.other_lane->parent->point_frame(other_lane_membership.position, offset));
            pos[0] = trans(0,3); pos[1] = trans(1,3); pos[2] = trans(2,3); pos[3] = trans(3,3);

            trans = rotation*trans;
            trans(0,3) = pos[0]; trans(1,3) = pos[1]; trans(2,3) = pos[2]; trans(3,3) = pos[3];

        }
        else
            trans = mat4x4f(l->point_frame(position));

        return trans;
    }

    vec3f car::point_theta(float &theta, const hwm::lane *l, const float lane_width) const
    {
        vec3f pos;
        if (other_lane_membership.other_lane != 0)
        {
            float offset  = std::min(lc_curve::y(other_lane_membership.merge_param), (float)1.0);
            offset       *= lane_width;
            if (!other_lane_membership.is_left)
            {
                offset *= -1;
            }

            theta = other_lane_membership.theta;

            float lane_theta;
            pos = other_lane_membership.other_lane->parent->point_theta(lane_theta, other_lane_membership.position, offset);
            theta += lane_theta;
        }
        else
           pos = l->point_theta(theta, position);

        return pos;
    }

   float car::check_lane_fully(const lane* l, const float param, const simulator& sim)
   {
      // Variables
      float gap = -1;

      car full_leader = get_full_leader_from_all(l, param, sim);

      car full_follower = get_full_follower_from_all(l, param, sim);

      /* Check if leader is full leader. */
      // Found a leader
      if (full_leader.position != -1)
         // Leader is 'full distance' ahead.
         if ( (full_leader.position - param)*l->length < (sim.car_length + sim.rear_bumper_rear_axle))
         {
            gap = -1;
            goto testFailed;
         }

      /* Check if follower is full follower. */
      if (full_follower.position != -1)
         if ( (param - full_follower.position)*l->length < (sim.car_length + sim.rear_bumper_rear_axle))
         {
            gap = -1;
            goto testFailed;
         }

      // If both are found, check gap
      if (full_leader.position != -1 and full_follower.position != -1)
      {
         gap = full_leader.position - full_follower.position;

         // Make sure full_leader is leader of full_follower.
         car followersLeader = get_full_leader_from_all(l, full_follower.position, sim);

         if (followersLeader.position != full_leader.position)
         {
            gap = -1;
            goto testFailed;
         }
      }

      // Leader and no follower.
      else if (full_leader.position != -1)
      {
         gap = full_leader.position - param;
      }

      // Follower and no leader.
      else if (full_follower.position != -1)
      {
         gap = param - full_follower.position;
      }

      // Neither.
      else
      {
         gap = 1.0f;
      }

   testFailed:
      gap = (gap * l->length) - (sim.car_length + sim.rear_bumper_rear_axle);

      return gap;
   }

   car car::get_full_follower_from_all(const lane* l, const float param, const simulator& sim)
   {
      // Variables
      car   potential_follower(-1, -1, -1, -1);
      float min_dist_behind = std::numeric_limits<float>::max();

      // Constants
      float inv_lane_size = 1.0 / l->length;
      //      float rear_of_car_param = param - ((sim.car_length + sim.rear_bumper_rear_axle) * inv_lane_size);
      float rear_of_car_param = param;

      // Search current cars.
      for (int i = (int)l->current_cars().size() - 1; i >= 0; i--)
      {
         car this_car = l->current_car(i);

         if (this_car.position < rear_of_car_param)
         {
            potential_follower = this_car;
            min_dist_behind = rear_of_car_param - potential_follower.position;
            break;
         }
      }

      // Search 'next' cars.
      //   -This search is exhastive as I do not believe next cars is ordered.
      for (int i = 0; i < (int)l->next_cars().size(); i++)
      {
         // New follower if 1) fully behind and 2) closer than current follower.
         if ((l->next_car(i).position < rear_of_car_param)
             and
             (rear_of_car_param - l->next_car(i).position < min_dist_behind))
         {
            potential_follower = l->next_car(i);
            min_dist_behind = rear_of_car_param - l->next_car(i).position;
         }
      }

      // And we need to check cars merging into this lane from the left and right.
      // -Left
      float      left_param     = rear_of_car_param;
      hwm::lane* potential_left = l->parent->left_adjacency(left_param); // left_param is as named.
      lane*      left_lane      = 0;

      if (potential_left)
      {
         left_lane = potential_left->user_data<lane>();

         car c = left_lane->get_merging_follower(left_param, l);

         if (c.position > -1)
         {
            if (rear_of_car_param - c.position < min_dist_behind)
            {
               potential_follower = c;
               min_dist_behind = rear_of_car_param - c.position;
            }
         }
      }

      // -Right
      float      right_param     = rear_of_car_param;
      hwm::lane* potential_right = l->parent->right_adjacency(right_param); // right_param is as named.
      lane*      right_lane      = 0;

      if (potential_right)
      {
         right_lane = potential_right->user_data<lane>();

         car c = right_lane->get_merging_follower(right_param, l);

         if (c.position > -1)
         {
            if (rear_of_car_param - c.position < min_dist_behind)
            {
               potential_follower = c;
               min_dist_behind = rear_of_car_param - c.position;
            }
         }
      }

      return potential_follower;
   }

   car car::get_full_leader_from_all(const lane* l, const float param, const simulator& sim)
   {
      // Variables
      car   potential_leader(-1, -1, -1, -1);
      float min_dist_ahead = std::numeric_limits<float>::max();

      // Constants
      float inv_lane_size = 1.0 / l->length;
      //      float front_of_car_param = param + ((sim.car_length + sim.rear_bumper_rear_axle) * inv_lane_size);
      float front_of_car_param = param;

      // Search current cars.
      for (int i = 0; i < (int)l->current_cars().size(); i++)
      {
         car this_car  = l->current_car(i);

         if (this_car.position > front_of_car_param)
         {
            potential_leader = this_car;
            min_dist_ahead = potential_leader.position - front_of_car_param;
            break;
         }
      }

      // Search 'next' cars.
      //   -This search is exhastive as I do not believe next cars is ordered.
      for (int i = 0; i < (int)l->next_cars().size(); i++)
      {
         // New leader if 1) fully ahead and 2) closer than current leader.
         if ((l->next_car(i).position > front_of_car_param)
             and
             (l->next_car(i).position - front_of_car_param < min_dist_ahead))
         {
            potential_leader = l->next_car(i);
            min_dist_ahead = l->next_car(i).position - front_of_car_param;
         }
      }

      // And we need to check cars merging out of this lane from the left and right.
      // -Left
      float      left_param     = front_of_car_param;
      hwm::lane* potential_left = l->parent->left_adjacency(left_param); // left_param is as named.
      lane*      left_lane      = 0;

      if (potential_left)
      {
         left_lane = potential_left->user_data<lane>();

         car c = left_lane->get_merging_leader(left_param, l);
         if (c.position > -1)
         {
            if (c.position - front_of_car_param < min_dist_ahead)
            {
               potential_leader = c;
               min_dist_ahead = c.position - front_of_car_param;
            }
         }
      }

      // -Right
      float      right_param     = front_of_car_param;
      hwm::lane* potential_right = l->parent->right_adjacency(right_param); // right_param is as named.
      lane*      right_lane      = 0;

      if (potential_right)
      {
         right_lane = potential_right->user_data<lane>();

         car c = right_lane->get_merging_leader(right_param, l);
         if (c.position > -1)
         {
            if (c.position - front_of_car_param < min_dist_ahead)
            {
               potential_leader = c;
               min_dist_ahead = c.position - front_of_car_param;
            }
         }
      }

      return potential_leader;
   }

    float car::getLaneDeccel(const lane* l, const float param, const float timestep, const simulator& sim)
    {
       // Variables
       float deccel;

       car full_follower = get_full_follower_from_all(l, param, sim);

       /* Check if follower is full follower. */
       if (full_follower.position == -1)
       {
          deccel = 0;
       }

      // Follower and no leader.
      else
      {
         // TODO this is hacky.
         deccel = car_accel(full_follower.velocity, full_follower.velocity,
                            (this->position - full_follower.position) * l->length);
      }

      return deccel;
    }

    float car::getMyAccel(const lane* l, const float param, const float timestep, const simulator& sim)
    {
       // Variables
       float accel;

       car fullLeader = get_full_leader_from_all(l, param, sim);

       /* Check if follower is full follower. */
       if (fullLeader.position == -1)
       {
          accel = sim.a_max;
       }

      // Follower and no leader.
      else
      {
         // TODO this is hacky.
         accel = car_accel(fullLeader.velocity, this->velocity,
                            (fullLeader.position - this->position) * l->length);
      }

      return accel;
    }


    void car::check_if_valid_acceleration(lane& l, float timestep)
    {
        float tmp_position = position + (velocity * timestep) * l.inv_length;

        lane* curr = &l;
        lane* destination_lane = &l;

        while(tmp_position >= 1.0)
        {

            hwm::lane *hwm_downstream = curr->parent->downstream_lane();

            if (!hwm_downstream)
            {
                std::cout << "error" << std::endl;
                assert(0);
            }

            if (!hwm_downstream->active)
            {
                std::cout << "error" << std::endl;
                assert(0);
            }

            lane* downstream = hwm_downstream->user_data<lane>();

            tmp_position = (tmp_position - 1.0f) * curr->length * downstream->inv_length;

            destination_lane = downstream;

            if (tmp_position >= 1.0)
            {
                curr = downstream;
            }
        }
    }

    void car::find_free_dist_and_vel(const lane &l, float& next_velocity, float& distance, const simulator& sim)
    {
        const float  min_for_free_movement = 1000;

	// check to see if the current lane is terminated, if it has a lane_terminus or intersection_terminus return false otherwise yes
        
	if(l.parent->end->network_boundary())
        {
            next_velocity = sim.boundary_conditions.get_vel_out(&l);
            distance = sim.boundary_conditions.get_dist_out(&l, sim.car_length);

            // TODO The addition of this distance is debatable.
            //            distance += (1.0 - position) * l.length;

            return;
        }

        hwm::lane *hwm_downstream = l.parent->downstream_lane();
        next_velocity             = 0.0f;
        distance                  = (1.0 - position) * l.length - sim.rear_bumper_rear_axle; // distance = dist to lane end + car length
        if(hwm_downstream)
            hwm_downstream->user_data<lane>()->distance_to_car(distance, next_velocity, min_for_free_movement, sim);
    }

    void car::compute_intersection_acceleration(const simulator &sim, const lane &l)
    {
        float next_velocity;
        float distance;
	
        find_free_dist_and_vel(l, next_velocity, distance, sim);
        acceleration = car_accel(next_velocity, velocity, distance);
    }

    void car::integrate(const float timestep, const lane& l, const float lane_width)
    {
        //Update position and velocity
        velocity = std::max(0.0f, velocity + acceleration * timestep);
        position += (velocity * timestep) * l.inv_length;

        //Move car that is also a member of the other lane
        if (other_lane_membership.other_lane != 0)
        {
            const float old_y                  = lc_curve::y(other_lane_membership.merge_param);
            float       needed_duration        = lc_curve::end(velocity);
            other_lane_membership.merge_param += timestep / needed_duration;
            const float new_y                  = lc_curve::y(other_lane_membership.merge_param);
            float       del_y                  = lane_width*(old_y-new_y);
            if(other_lane_membership.is_left)
                del_y                         *= -1;
            if ((velocity > 0.1) and (del_y > 0))
               other_lane_membership.theta        = std::atan2(del_y, (velocity * timestep));
            other_lane_membership.position    += (velocity * timestep) * other_lane_membership.other_lane->inv_length;

            //TODO L is wheelbase -- length to rear axel
            // float L = 4.5;
            // float u_s = 1.0f/sin(other_lane_membership.theta)*velocity;
            // float d_x = u_s*cos(theta);
            // float d_theta = (u_s/L)*tan(other_lane_membership.merge_param*other_lane_membership.phi_max)*timestep;

            if (other_lane_membership.merge_param > 1)
            {
                other_lane_membership.merge_param = 0;
                other_lane_membership.other_lane  = 0;
                other_lane_membership.position    = 0;
                other_lane_membership.theta       = 0;
            }
        }
    }

    static inline float rk4(float x, float dtinvlen, const lane *la, float gamma_c)
    {
        float k[4];
        k[0] = la->velocity(x,                      gamma_c);
        k[1] = la->velocity(x + 0.5f*dtinvlen*k[0], gamma_c);
        k[2] = la->velocity(x + 0.5f*dtinvlen*k[1], gamma_c);
        k[3] = la->velocity(x +      dtinvlen*k[2], gamma_c);

        return 1.0f/6.0f*dtinvlen*(k[0] + 2.0f*k[1] + 2.0f*k[2] + k[3]);
    }

    void car::carticle_integrate(const float timestep, const lane& l, const float lane_width)
    {
        position += (velocity * timestep) * l.inv_length;

        //Move car that is also a member of the other lane
        if (other_lane_membership.other_lane != 0)
        {
            const float old_y                  = lc_curve::y(other_lane_membership.merge_param);
            float       needed_duration        = lc_curve::end(velocity);
            other_lane_membership.merge_param += timestep / needed_duration;
            const float new_y                  = lc_curve::y(other_lane_membership.merge_param);
            float       del_y                  = lane_width*(old_y-new_y);
            if(other_lane_membership.is_left)
                del_y                         *= -1;
            other_lane_membership.theta        = std::atan2(del_y, (velocity * timestep));
            other_lane_membership.position    += (velocity * timestep) * other_lane_membership.other_lane->inv_length;

            //TODO L is wheelbase -- length to rear axel
            // float L = 4.5;
            // float u_s = 1.0f/sin(other_lane_membership.theta)*velocity;
            // float d_x = u_s*cos(theta);
            // float d_theta = (u_s/L)*tan(other_lane_membership.merge_param*other_lane_membership.phi_max)*timestep;

            if (other_lane_membership.merge_param > 1)
            {
                other_lane_membership.merge_param = 0;
                other_lane_membership.other_lane  = 0;
                other_lane_membership.position    = 0;
                other_lane_membership.theta       = 0;
            }
        }
    }


    void car::write_record(std::ostream &o, const lane &l, const float lane_width) const
    {
        float theta;
        vec3f p(point_theta(theta, l.parent, lane_width));
        float nx = std::cos(theta);
        float ny = std::sin(theta);
        o << id << " " << p[0] << " " << p[1] << " " << " " << p[2]
          << " " << nx << " " << ny << " " << velocity*nx << " " << velocity*ny
          << " " << acceleration << std::endl;
    }

    float car::car_accel(const float leader_velocity,
                         const float follower_velocity,
                         float distance) const
    {
        float car_length = 4.5;

        const float delta_v = follower_velocity - leader_velocity;

        float optimal_spacing = p_s1 + p_T*follower_velocity +
           (follower_velocity*delta_v)/(2*(std::sqrt(p_a*p_b)));

        optimal_spacing = std::max(0.1f, optimal_spacing);

        distance -= car_length;

        if (distance <= 0)
            distance = std::numeric_limits<float>::min();

        float t = p_a*(1 - std::pow((follower_velocity / (p_speedlimit * p_speedlimit_factor)), p_delta) - std::pow((optimal_spacing/(distance)), 2));

        // t = std::min(a_max, t);
        // t = std::max(t, -a_max);

        return t;
    }


    float lane::acceleration(const float leader_velocity,
                             const float follower_velocity,
                             const float speedlimit,
                             float distance) const
    {
        float a_max = 1.67;
        float a_pref = 0.73;
        float car_length = 4.5;

        const float delta_v = follower_velocity - leader_velocity;

        float optimal_spacing = micro_s1 + micro_T*follower_velocity + (follower_velocity*delta_v)/(2*(std::sqrt(a_max*a_pref)));

        optimal_spacing = std::max(1.0f, optimal_spacing);

        distance -= car_length;

        if (distance <= 0)
            distance = std::numeric_limits<float>::min();

        float t = a_max*(1 - std::pow((follower_velocity / speedlimit), micro_gamma) - std::pow((optimal_spacing/(distance)), 2));

        // t = std::min(a_max, t);
        // t = std::max(t, -a_max);

        return t;
    }

    car lane::get_merging_follower(float param, const lane* other_lane)
    {
        car cur_car(-1, -1, -1, -1);
        float min_dist_behind = std::numeric_limits<float>::max();

        for (int i = current_cars().size() - 1; i >= 0; i--)
        {
            if (current_car(i).other_lane_membership.other_lane != 0)
            {
                if (current_car(i).other_lane_membership.other_lane == other_lane)
                {
                    if (current_car(i).position < param)
                    {
                        cur_car = current_car(i);
                        cur_car.position = current_car(i).other_lane_membership.position;
                        min_dist_behind = param - current_car(i).position;

                        // The lowest indexed car will be the first fully ahead car.
                        break;
                    }
                }
            }
        }

        for (int i = next_cars().size() - 1; i >= 0; i--)
        {
            if (next_car(i).other_lane_membership.other_lane != 0)
            {
                if (next_car(i).other_lane_membership.other_lane == other_lane)
                {
                    if (next_car(i).position < param)
                    {
                       if (param - next_car(i).position < min_dist_behind)
                       {
                          cur_car = next_car(i);
                          cur_car.position = next_car(i).other_lane_membership.position;
                          min_dist_behind = param - next_car(i).position;

                          // As this is assume to be unordered, do exhaustive search.
                       }
                    }
                }
            }
        }

        return cur_car;
    }

   // TODO merging_leader/follower are in lane, but other leader/follower functions are in car
    car lane::get_merging_leader(float param, const lane* other_lane)
    {
        car cur_car(-1, -1, -1, -1);
        float min_dist_ahead = std::numeric_limits<float>::max();

        for (size_t i = 0; i < current_cars().size(); i++)
        {
            if (current_car(i).other_lane_membership.other_lane != 0)
            {
                if (current_car(i).other_lane_membership.other_lane == other_lane)
                {
                    if (current_car(i).position > param)
                    {
                       // TODO Tune
                       if (current_car(i).other_lane_membership.merge_param > 0.7)
                          continue;

                        cur_car = current_car(i);
                        cur_car.position = current_car(i).other_lane_membership.position;
                        min_dist_ahead = current_car(i).position - param;

                        // The lowest indexed car will be the first fully ahead car.
                        break;
                    }
                }
            }
        }

        for (size_t i = 0; i < next_cars().size(); i++)
        {
            if (next_car(i).other_lane_membership.other_lane != 0)
            {
                if (next_car(i).other_lane_membership.other_lane == other_lane)
                {
                    if (next_car(i).position > param)
                    {
                       if (next_car(i).position - param > min_dist_ahead)
                       {
                          cur_car = next_car(i);
                          cur_car.position = next_car(i).other_lane_membership.position;
                          min_dist_ahead = next_car(i).position - param;

                          // As this is assume to be unordered, do exhaustive search.
                       }
                    }
                }
            }
        }

        return cur_car;
    }

    void lane::micro_distance_to_car(float &distance, float &velocity, const float distance_max, const simulator &sim) const
    {
        // this function is to calculate velocity and distance for the last car in the last lane in order to feed the IDM
        // the current lane is the next lane for the last car
      
        // if there is no car in current lane
        if(current_cars().empty())
        {
            distance += length; // add the lane length to the distance, which stored the value between the last car to the end of the last lane (including last car length)
            if(distance >= distance_max) // distance_max is set to 1000
            {
                velocity = 0.0f;
                distance = distance_max;
            }
            else
            {
	        // if before meeting the distance_max we meet a terminated lane
                if(parent->end->network_boundary())
                {
                    velocity = sim.boundary_conditions.get_vel_out(this);
		    
		    // weizi: why set distance like this? seems far less then previous distance + current lane length
                    distance = sim.boundary_conditions.get_dist_out(this, sim.car_length); 

                    return;
                }

                hwm::lane *hwm_downstream = parent->downstream_lane();
                if(hwm_downstream)
                    hwm_downstream->user_data<lane>()->distance_to_car(distance, velocity, distance_max, sim);
                else
                    velocity = 0.0f;
            }
        }
        else
        {
	    // if there is a car in the current lane, then it's a leader return its velocity and distance
            velocity  = current_cars().front().velocity;
            distance += current_cars().front().position*length;
        }

        return;
    }

    void lane::compute_merges(const float timestep, const simulator& sim)
    {
        float threshold = 0.2;
        for(int i = (int)current_cars().size() - 1; i >= 0; --i)
        {
            //Don't consider merging again if still merging.
            if (current_car(i).other_lane_membership.other_lane == 0)
            {
                //Check if car decides to merge
                float right_param = current_car(i).position;
                hwm::lane* potential_right = parent->right_adjacency(right_param);
                float right_accel = std::numeric_limits<int>::min();
                lane* right_lane = 0;
                if ((potential_right) and potential_right->active)
                {
                    right_lane = potential_right->user_data<lane>();
                    right_accel = current_car(i).getLaneDeccel(right_lane, right_param, timestep, sim);
                }

                float      left_param     = current_car(i).position;
                hwm::lane* potential_left = parent->left_adjacency(left_param);
                float     left_accel     = std::numeric_limits<int>::min();
                lane*      left_lane      = 0;

                if ((potential_left) and (potential_left->active))
                {
                    left_lane = potential_left->user_data<lane>();
                    left_accel = current_car(i).getLaneDeccel(left_lane, left_param, timestep, sim);
                }

                //Merge the cars
                if (right_accel  > threshold
                    or left_accel > threshold)
                {
                    if (right_accel > left_accel)
                    {
                        current_car(i).other_lane_membership.other_lane  = this;
                        current_car(i).other_lane_membership.merge_param = 0;
                        current_car(i).other_lane_membership.position    = current_car(i).position;
                        current_car(i).other_lane_membership.is_left     = false;
                        current_car(i).position                          = right_param;

                        right_lane->next_cars().push_back(current_car(i));
                    }
                    else
                    {
                        current_car(i).other_lane_membership.other_lane  = this;
                        current_car(i).other_lane_membership.merge_param = 0;
                        current_car(i).other_lane_membership.position    = current_car(i).position;
                        current_car(i).other_lane_membership.is_left     = true;
                        current_car(i).position                          = left_param;
                        left_lane->next_cars().push_back(current_car(i));
                    }
                }
                else
                {
                    next_cars().push_back(current_car(i));
                }
            }
            else
            {
                next_cars().push_back(current_car(i));
            }
        }
    }

    void lane::compute_controlled_merges(const float timestep, const simulator& sim)
    {
        float threshold = 0.2;
        for(int i = (int)current_cars().size() - 1; i >= 0; --i)
        {
            //Don't consider merging again if still merging.
            if (current_car(i).other_lane_membership.other_lane == 0)
            {
                //Check if car decides to merge
                float right_param = current_car(i).position;
                hwm::lane* potential_right = parent->right_adjacency(right_param);
                float right_accel = std::numeric_limits<int>::min();
                lane* right_lane = 0;
                if ((potential_right) and potential_right->active)
                {
                    right_lane = potential_right->user_data<lane>();
                    right_accel = current_car(i).getLaneDeccel(right_lane, right_param, timestep, sim);
                }

                float      left_param     = current_car(i).position;
                hwm::lane* potential_left = parent->left_adjacency(left_param);
                float     left_accel     = std::numeric_limits<int>::min();
                lane*      left_lane      = 0;

                if ((potential_left) and (potential_left->active))
                {
                    left_lane = potential_left->user_data<lane>();
                    left_accel = current_car(i).getLaneDeccel(left_lane, left_param, timestep, sim);
                }

                //Merge the cars
                if (right_accel  > threshold
                    or left_accel > threshold)
                {
                    if (right_accel > left_accel)
                    {
                        current_car(i).other_lane_membership.other_lane  = this;
                        current_car(i).other_lane_membership.merge_param = 0;
                        current_car(i).other_lane_membership.position    = current_car(i).position;
                        current_car(i).other_lane_membership.is_left     = false;
                        current_car(i).position                          = right_param;

                        right_lane->next_cars().push_back(current_car(i));
                    }
                    else
                    {
                        current_car(i).other_lane_membership.other_lane  = this;
                        current_car(i).other_lane_membership.merge_param = 0;
                        current_car(i).other_lane_membership.position    = current_car(i).position;
                        current_car(i).other_lane_membership.is_left     = true;
                        current_car(i).position                          = left_param;
                        left_lane->next_cars().push_back(current_car(i));
                    }
                }
                else
                {
                    next_cars().push_back(current_car(i));
                }
            }
            else
            {
                next_cars().push_back(current_car(i));
            }
        }
    }


    car& lane::find_next_car(float param)
    {
        //TODO bisect search
        for (size_t i = 0; i < current_cars().size(); i++)
        {
            if (current_car(i).position >= param)
            {
                return current_car(i);
            }
        }
        assert(0);
        return current_cars()[0];
    }

    void lane::compute_lane_accelerations(const float timestep, const simulator &sim)
    {
        // only for lanes that runs on micro simulation
        if(current_cars().empty())
            return;
	
	
	//std::cout<<current_cars().size()<<std::endl;

        for(size_t i = 0; i < current_cars().size(); ++i)
        {
            if (i < current_cars().size() - 1)
            {
                current_car(i).acceleration =
                   current_car(i).car_accel(current_car(i+1).velocity,
                                            current_car(i).velocity,
                                            (current_car(i+1).position
                                             - current_car(i).position)*length);
		   
		   
		   //std::cout<<current_car(i).acceleration<<std::endl;

            }
            else
            {
		float out_vel = sim.boundary_conditions.get_vel_out(this); // get the boundary out velocity	    	
		float _T = 400; // set the relaxation factor
                float vel = current_cars().back().velocity; // get the current car velocity
                float relax =  (out_vel - vel) / _T; // calculate the relax

                // Calculate the actual acceleration for the last car
                current_cars().back().compute_intersection_acceleration(sim, *this);

                // TODO should not need this.
                if ((current_cars().back().velocity < 0.01) and (current_cars().back().velocity < out_vel))
                {
                    current_cars().back().acceleration = std::max(relax, current_cars().back().acceleration);
                }
            }

            // Check if there are cars still merging out of this lane.
            float right_param = current_car(i).position;
            hwm::lane* potential_right = parent->right_adjacency(right_param);
	    	    
            lane* right_lane;
            float right_accel = 0;
            car next_r;
            if (potential_right)
            {
                right_lane = potential_right->user_data<lane>();

                next_r  = right_lane->get_merging_leader(right_param, this);

                if (next_r.position > -1)
                    right_accel = acceleration(next_r.velocity, current_car(i).velocity, potential_right->speedlimit, std::abs(next_r.position - right_param)*right_lane->length);
            }

            float left_param = current_car(i).position;
            hwm::lane* potential_left = parent->left_adjacency(left_param);
            lane* left_lane;
            float left_accel = 0;
            car next_l;
            if (potential_left)
            {
                left_lane = potential_left->user_data<lane>();

                next_l  = left_lane->get_merging_leader(left_param, this);
                if (next_l.position > -1)
                {
                    left_accel = acceleration(next_l.velocity, current_car(i).velocity, potential_left->speedlimit, std::abs(next_l.position - left_param)*left_lane->length);
                }
            }

            if (potential_right and next_r.position > -1)
                current_car(i).acceleration = std::min(current_car(i).acceleration, (float)right_accel);

            if (potential_left and next_l.position > -1)
                current_car(i).acceleration = std::min(current_car(i).acceleration, (float)left_accel);
        }
        
        std::cout<<"Finish compuitng lane accelerations"<<std::endl;
    }

    void lane::populate_density(const float density, const float init_vel, simulator &sim)
    {
        current_cars().clear();
        next_cars().clear();

        int cars = std::floor(density * length);
        int cars_added = 0;
        float even_spacing = length / ((1.0)*cars);
        float even_spacing_param = even_spacing / length;

        float t = 0;
        while (cars_added < cars)
        {
            current_cars().push_back(sim.make_car(t, init_vel, 0));

            t += even_spacing_param;
            cars_added++;
        }
    }

    void lane::populate(const float rate, simulator &sim)
    {
        current_cars().clear();
        next_cars().clear();

        float t = pproc::exp_rvar((*sim.uni)())/rate*inv_length;
        if(t > 1.0)
            return;
        do
        {
            current_cars().push_back(sim.make_car(t, 0, 0));

            float next;
            do
            {
                next = pproc::exp_rvar((*sim.uni)())/rate;
            }
            while(next < 2*sim.car_length);
            t += next*inv_length;
        }
        while(t < 1.0);
    }

    void simulator::clear()
    {
        for (uint i = 0; i < lanes.size(); i++)
        {
            lanes[i].current_cars().clear();
            lanes[i].next_cars().clear();
        }
    }

    void simulator::micro_initialize(const float in_a_max, const float in_a_pref, const float in_v_pref,
                                     const float in_delta)
    {
        a_max                 = in_a_max;
        a_pref                = in_a_pref;
        v_pref                = in_v_pref;
        delta                 = in_delta;
        s1 = 2;
        T = 1.6;
    }

    void simulator::compute_accelerations(const float timestep)
    {
        for(lane &l: lanes)
        {
            if (l.is_micro() && l.parent->active) {
	      
                l.compute_lane_accelerations(timestep, *this);
	    }
        }

        // for(lane& l: lanes)
        // {
        //   if (l.is_micro() and l.parent->active)
        //     l.compute_merges(timestep, *this);
        // }

        // for(lane& l: lanes)
        // {
        //   if (l.active())
        //     l.car_swap();
        // }
    }

    void simulator::update(const float timestep)
    {
        // Compute accelerations.
        compute_accelerations(timestep);

        // Move cars forward.
        for(lane &l: lanes)
        {
            if(l.is_micro() && l.parent->active)
            {
                for(car &c: l.current_cars())
                {
                    c.integrate(timestep, l, hnet->lane_width);
                }
            }
        }

        // Move cars to next lane if they move out of the current lane.
        for(lane &l: lanes)
        {
            if(l.is_micro() && l.parent->active)
            {
                for(car &c: l.current_cars())
                {
                    lane* destination_lane = &l;
                    lane* curr = &l;

                    while(c.position >= 1.0)
                    {
                        if(curr->parent->end->network_boundary())
                            goto next_car;

                        lane *downstream = curr->downstream_lane();
                        assert(downstream);
                        assert(downstream->parent->active);

                        switch(downstream->sim_type)
                        {
                        case MICRO:
                            c.position = (c.position - 1.0f) * curr->length * downstream->inv_length;
                            // downstream->next_cars().push_back(c);
                            destination_lane = downstream;
                            break;
                        case MACRO: //TODO update for correctness
                            if(downstream->fictitious)
                            {
                                lane *next_downstream = downstream->downstream_lane();
                                assert(next_downstream);
                                assert(!next_downstream->fictitious);
                                downstream = next_downstream;
                            }
                            downstream->q[0].rho() = std::min(1.0f, downstream->q[0].rho() + car_length/downstream->h);
                            downstream->q[0].y()   = std::min(0.0f, arz<float>::eq::y(downstream->q[0].rho(), c.velocity,
                                                                                      downstream->speedlimit(),
                                                                                      gamma));
                            assert(downstream->q[0].check());
                            goto next_car;
                        }

                        if (c.position >= 1.0)
                            curr = downstream;
                    }

                    assert(c.position < 1.0);

                    destination_lane->next_cars().push_back(c);

                next_car:;
                }
            }
        }
    }

    void simulator::carticle_update(const float timestep)
    {
        // Move cars forward.
        for(lane &l: lanes)
        {
            if(l.is_micro() && l.parent->active)
            {
                for(car &c: l.current_cars())
                {
                    c.carticle_integrate(timestep, l, hnet->lane_width);
                }
            }
        }

        // Move cars to next lane if they move out of the current lane.
        for(lane &l: lanes)
        {
            if(l.is_micro() && l.parent->active)
            {
                for(car &c: l.current_cars())
                {
                    lane* destination_lane = &l;
                    lane* curr = &l;

                    while(c.position >= 1.0)
                    {
                        if(curr->parent->end->network_boundary())
                            goto next_car;

                        lane *downstream = curr->downstream_lane();
                        assert(downstream);
                        assert(downstream->parent->active);

                        switch(downstream->sim_type)
                        {
                        case MICRO:
                            c.position = (c.position - 1.0f) * curr->length * downstream->inv_length;
                            // downstream->next_cars().push_back(c);
                            destination_lane = downstream;
                            break;
                        case MACRO: //TODO update for correctness
                            if(downstream->fictitious)
                            {
                                lane *next_downstream = downstream->downstream_lane();
                                assert(next_downstream);
                                assert(!next_downstream->fictitious);
                                downstream = next_downstream;
                            }
                            downstream->q[0].rho() = std::min(1.0f, downstream->q[0].rho() + car_length/downstream->h);
                            downstream->q[0].y()   = std::min(0.0f, arz<float>::eq::y(downstream->q[0].rho(), c.velocity,
                                                                                      downstream->speedlimit(),
                                                                                      gamma));
                            assert(downstream->q[0].check());
                            goto next_car;
                        }

                        if (c.position >= 1.0)
                            curr = downstream;
                    }

                    assert(c.position < 1.0);

                    destination_lane->next_cars().push_back(c);

                next_car:;
                }
            }
        }
    }

    void simulator::micro_cleanup()
    {
    }
}
