#ifndef _HYBRID_SIM_HPP_
#define _HYBRID_SIM_HPP_

#include "hwm_network.hpp"
#include "libhybrid-common.hpp"
#include "arz.hpp"
#include "pc-poisson.hpp"
#include <boost/random.hpp>

#include <set>

namespace hybrid
{

   inline double _drandom() {
      return (double) (rand() / (double)RAND_MAX);//(rand()*(RAND_MAX+1) + rand()) / (RAND_MAX*(RAND_MAX + 2));
   }

   /*!
    *  @brief       Randomly samples from the univariate standard Gaussian distribution N(0,1).
    *  @returns     A random number from the univariate standard Gaussian distribution N(0,1).
    *  \ingroup globalfunc
    */
   inline double _normal() {
      double u, v, s(0);

      while (s == 0 || s > 1) {
         u = 2*_drandom()-1;
         v = 2*_drandom()-1;
         s = u*u + v*v;
      }

      return u * sqrt(-2*log(s)/s);
   }



    typedef enum {MACRO=1, MICRO=2} sim_t;

    struct simulator;
    struct lane;

    struct boundary_flow
    {
        float density;
        float velocity;
        boundary_flow(float in_den, float in_vel)
        {
            density = in_den;
            velocity = in_vel;
        }
        boundary_flow()
        {
            density = -1;
            velocity = -1;
        }
    };

    struct boundaries
    {
        std::map<const lane*, boundary_flow> in_bounds;
        std::map<const lane*, boundary_flow> out_bounds;

        float get_dist_out(const lane* lane_p, float car_len) const;

        float get_vel_out(const lane* lane_p) const;

        float get_dist_in(const lane* lane_p, float car_len) const;

        float get_vel_in(const lane* lane_p) const;

        float get_rate(const lane* lane_p, const float car_length) const;

    };

    struct car
    {
        car() : id(0)
       {
            // Assign some reasonable defaults for micro parameters.
            p_s1 = 1;
            p_delta = 4;
            p_T = 1.6;
            p_speedlimit = 25;
            p_speedlimit_factor = 1.0;
            p_a = 1.67;
            p_b = p_a / 2.0;
            r = 0;
            g = 0;
            b = 1;

            velDist = 1;
            offsetNoise = 0;
       }
        car(const size_t in_id, const float in_position,
            const float in_velocity, const float in_acceleration)
            : id(in_id), position(in_position),
              velocity(in_velocity), acceleration(in_acceleration)
        {
            other_lane_membership.other_lane  = 0;
            other_lane_membership.merge_param = 0;
            other_lane_membership.theta       = 0;

            // Assign some reasonable defaults for micro parameters.
            p_s1 = 1;
            p_delta = 4;
            p_T = 1.6;
            p_speedlimit = 25;
            p_speedlimit_factor = 1.0;
            p_a = 1.67;
            p_b = p_a / 2.0;

            // TODO hacky
            velDist = 1 + (_normal() * 0.25);

            velDist = std::max(velDist, 0.5f);
            velDist = std::min(velDist, 2.0f);

            offsetNoise = _normal() * 0.2;
            offsetNoise = std::max(velDist, -0.5f);
            offsetNoise = std::min(velDist, .05f);
        }

        //common data
        size_t id;
        float  position;
        float  velocity;
        float  acceleration;

        // Color for drawing;
        float r, g, b;

        // Parameters for microsim.
        float p_s1;
        float p_delta;
        float p_T;
        float p_speedlimit;
        float p_speedlimit_factor;
        float p_a;
        float p_b;

       float velDist;
       float offsetNoise;

       inline bool isMerging()
       {
          return (other_lane_membership.other_lane != 0);
       }


        // micro data
        struct Other_lane_membership
        {
            bool   is_left;
            lane  *other_lane;
            float  merge_param;
            float  position;
            float  theta;
        } other_lane_membership;

        float car_accel(const float leader_velocity, const float follower_velocity, float distance) const;
        void find_free_dist_and_vel(const lane& l, float& next_velocity, float& distance, const simulator& sim);
        void compute_intersection_acceleration(const simulator &sim, const lane &l);
        void integrate(float timestep, const lane &l, float lane_width);
        void carticle_integrate(float timestep, const lane &l, float lane_width);
        void check_if_valid_acceleration(lane& l, float timestep);
       car get_full_leader_from_all(const lane* l, const float param, const simulator& sim);
       car get_full_follower_from_all(const lane* l, const float param, const simulator& sim);
        float check_lane_fully(const lane* l, const float param, const simulator& sim);
        float getLaneDeccel(const lane* l, const float param, const float timestep, const simulator& sim);
        float getMyAccel(const lane* l, const float param, const float timestep, const simulator& sim);
        float check_lane_gap_clearance(const lane* l, const float param, const float timestep, const simulator& sim);

        mat4x4f point_frame(const hwm::lane *l, float lane_width) const;
        vec3f point_theta(float &theta, const hwm::lane *l, float lane_width) const;
        void write_record(std::ostream &o, const lane &l, const float lane_width) const;

        // macro data
    };

    struct car_interp
    {
        struct car_spatial
        {
            car_spatial();
            car_spatial(const car &in_c, const hwm::lane *l);

            bool operator<(const car_spatial &o) const
            {
                return c.id < o.c.id;
            }

            car              c;
            const hwm::lane *la;
        };

        typedef std::set<car_spatial> car_hash;

        car_interp(simulator &s);
        void capture(simulator &s);

        bool in_second(size_t id) const;
        float   acceleration(size_t id, float time) const;
        mat4x4f point_frame(size_t id, float time, float lane_width) const;

        vec2f    times;
        float    inv_dt;
        car_hash car_data[2];
    };

    struct lane
    {
        struct serial_state
        {
            serial_state();
            serial_state(const lane &l);

            void apply(lane &l) const;

            std::vector<car> cars;
            sim_t            sim_type;
        };

        lane();

        // common data
        void                     initialize(hwm::lane *parent);

        const std::vector<car>  &current_cars() const { return cars[0]; }
        std::vector<car>        &current_cars()       { return cars[0]; }
        const car               &current_car(size_t i) const { return cars[0][i]; }
        car                     &current_car(size_t i)       { return cars[0][i]; };

        const std::vector<car>  &next_cars() const { return cars[1]; };
        std::vector<car>        &next_cars()       { return cars[1]; };
        const car               &next_car(size_t i) const { return cars[1][i]; }
        car                     &next_car(size_t i)       { return cars[1][i]; };

        size_t                  ncars()     const { return current_cars().size(); }
        void                    car_swap();
        bool                    is_micro()  const;
        bool                    is_macro()  const;
        bool                    occupied()  const;
        bool                    active()    const;
        void                    convert(sim_t dest_type, simulator &sim);
        void                    convert_to_micro(simulator &sim);
        void                    convert_to_macro(simulator &sim);
        lane                   *left_adjacency(float &param);
        lane                   *right_adjacency(float &param);
        lane                   *upstream_lane();
        const lane             *upstream_lane() const;
        lane                   *downstream_lane();
        const lane             *downstream_lane() const;
        float                   speedlimit() const { return parent->speedlimit; }
        void                    populate_density(const float density, const float init_vel, simulator &sim);
        void                    populate(float rate, simulator &sim);
        float                   acceleration(const float leader_velocity,
                                             const float follower_velocity,
                                             const float speedlimit,
                                             const float distance) const;

        serial_state serial() const;

        // Parameters for microsimulation.
        float micro_s1;
        float micro_T;
        float micro_gamma;

        float next_noise;

        hwm::lane        *parent;
        float             length;
        float             inv_length;
        std::vector<car>  cars[2];
        sim_t             sim_type;
        bool              updated_flag;
        bool              fictitious;

        void distance_to_car(float &distance, float &velocity, const float distance_max, const simulator &sim) const;

        // micro data
        void  micro_distance_to_car(float &distance, float &velocity, const float distance_max, const simulator &sim) const;
        void  compute_lane_accelerations(float timestep, const simulator &sim);
        //    float settle_pass(const float timestep, const float epsilon, const float epsilon_2, const simulator &sim);
        void  compute_merges(const float timestep, const simulator& sim);
        void  compute_controlled_merges(const float timestep, const simulator& sim);
        car&  find_next_car(float param);
        car   get_merging_leader(float param, const lane* other_lane);
        car   get_merging_follower(float param, const lane* other_lane);

        /**
	 * Calculating the actuall cell size and number of cells of the current lane
	 * @param h_suggest suggested cell size 
	 */
        void  macro_initialize(const float h_suggest);
        
	
	void  macro_instantiate(simulator &sim);
        bool  macro_find_first(float &param, const simulator &sim) const;
        bool  macro_find_last(float &param, const simulator &sim) const;
        void  macro_distance_to_car(float &distance, float &velocity, const float distance_max, const simulator &sim) const;
        int   which_cell(float pos) const;
        float velocity(float pos, float gamma) const;
        float collect_riemann(const float gamma, const float inv_gamma);
        void  update         (const float dt,    simulator  &sim);
        void  clear_macro();
        void  convert_cars(const simulator &sim);
        void  fill_y(const float gamma);

        float                         h;
        float                         inv_h;
        size_t                        N;
        arz<float>::q                *q;
        arz<float>::riemann_solution *rs;
    };

    struct micro_parameters {
       float s1;
       float delta;
       float T;
       float speedlimit_factor;
       float alpha;
       float beta;
    };

    struct simulator
    {
        typedef boost::rand48  base_generator_type;

        micro_parameters stored_parameters;

        struct serial_state
        {
            serial_state();
            serial_state(const simulator &s);
            ~serial_state();

            void apply(simulator &s) const;

            size_t               car_id_counter;
            arz<float>::q       *q_base;
            base_generator_type  generator;

            hwm::network::serial_state      network_state;
            std::vector<lane::serial_state> lane_states;
        };

        /*
	 * Constructor
	 * @param net hwm network
	 * @param length car length
	 * @param rear_axle length from car rear_axle to back
	 */
        simulator(hwm::network *net, float length, float rear_axle);

        ~simulator();
        void  initialize();
        float rear_bumper_offset()  const;
        float front_bumper_offset() const;
        void  car_swap();

        void clear();

       // Sets the prameters to use for the special cars created.
       void SetSpecialCarParams(float s1,
                                float delta,
                                float T,
                                float speedlimit_factor,
                                float alpha,
                                float beta);

        car   make_car(const float position, const float velocity, const float acceleration);

        lane       &get_lane_by_name(const str &s);
        const lane &get_lane_by_name(const str &s) const;

        size_t ncars() const;

        void  mass_reassign(std::vector<hwm::network_aux::road_spatial::entry> &qr);
        float hybrid_step();
        float micro_step(float dt);
        float carticle_step(float dt);
        void  advance_intersections(float dt);
        void  apply_incoming_bc(float dt, float t);
        void  old_apply_incoming_bc(float dt, float t);

        serial_state serial() const;
        car_interp::car_hash get_car_hash() const;

        hwm::network        *hnet;
        std::vector<lane>    lanes;
        float                car_length;
        float                rear_bumper_rear_axle;
        float                time;
        base_generator_type *generator;
        boost::uniform_real<> *uni_dist;
        typedef boost::variate_generator<base_generator_type&,
            boost::uniform_real<> > rand_gen_t;
        rand_gen_t          *uni;
        size_t               car_id_counter;
        boundaries           boundary_conditions;

        // micro
        void   micro_initialize(const float a_max, const float a_pref, const float v_pref,
                                const float delta);
        void   micro_cleanup();
        //    void   settle(const float timestep);
        //    float acceleration(const float leader_velocity, const float follower_velocity, const float s_limit, const float distance) const;
        void   compute_accelerations(float timestep);
        void   update(float timestep);
        void   carticle_update(float timestep);

        float a_max;
        float a_pref;
        float v_pref;
        float delta;
        float s1;
        float T;

        float sec_since_car;

        /**
	 * initialize a macro simulation
	 * @param gamma the gamma for calculating u_eq
	 * @param h_suggest suggested cell size
	 * @param relaxation the relaxiation factor
	 */
        void  macro_initialize(float gamma, float h_suggest, float relaxation);
	
	/**
	 * initialize a macro simulation
	 * @param gamma the gamma for calculating u_eq
	 * @param h_suggest suggested cell size
	 * @param relaxation the relaxiation factor
	 */
        void  macro_re_initialize(float gamma, float h_suggest, float relaxation);
        void  macro_cleanup();
        void  convert_cars(sim_t sim_mask);
        float macro_step(const float cfl=1.0f);

        arz<float>::q                *q_base;
        size_t                        N;
        arz<float>::riemann_solution *rs_base;
        float                         gamma;
        float                         h_suggest;
        float                         min_h;
        float                         relaxation_factor;
    };
}


#endif
