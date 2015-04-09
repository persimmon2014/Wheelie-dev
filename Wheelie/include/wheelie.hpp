#ifndef _WHEELIE_HPP_
#define _WHEELIE_HPP_

#include "hybrid-sim.hpp"
#include "libhybrid-common.hpp"
#include "arz.hpp"
#include "pc-poisson.hpp"

#include "hwm_network.hpp"
//#include <gtest/gtest_prod.h>

namespace wheelie {

struct CarParameters {
   float avg_car_length;
   float rear_bumper_offset;
};

struct SimMacroParameters {

   float gamma;
   float h_suggest;
   float relaxation;
};

struct SimMicroParameters {
   float max_acceleration;
   float preferred_acceleration;
   float exponential_delta;
};

// A wrapper class to define a more usable interface with the hybrid::simulator.
class Simulator
{
private:
   // A pointer to the actual simulator.
   // This class is the owner.
   hybrid::simulator* sim_;

   // A pointer to the road network.
   // This pointer is shared and gets allocated elsewhere.
   hwm::network* roads_;

   // The parameter values for the macroscopic simulator.
   SimMacroParameters macro_parameters_;

   // The parameter values for the microscopic simulator.
   SimMicroParameters micro_parameters_;

   // The parameters for the cars of either simulatior.
   // TODO: a more statistical and complex definition of cars should be used in
   // both of the simulations.
   CarParameters car_parameters_;

public:
   // Simulator constructor taking in the road network, uniform 'legth' of each
   // car, and the distance from the 'rear_axle' to the end of the car.
   Simulator();

   // Destructor for Simulator.
   ~Simulator();

   // Sets all the parameters of the simulatgor and allocates internal sim_.
   void Initialize();

   // Accessors
   hwm::network* get_roads();
   void set_roads(hwm::network* net);

   SimMacroParameters get_macro_parameters();
   void set_macro_parameters(const SimMacroParameters& macro_parameters);

   SimMicroParameters get_micro_parameters();
   void set_micro_parameters(const SimMicroParameters& micro_parameters);

   CarParameters get_car_parameters();
   void set_car_parameters(const CarParameters& car_parameters);

private:
   FRIEND_TEST(WheelieUnitTests, TestCreateNetwork);
};

} // namespace wheelie

#endif

