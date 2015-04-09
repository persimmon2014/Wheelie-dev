#include "wheelie.hpp"

namespace wheelie {

Simulator::Simulator() {
   sim_ = NULL;
}

Simulator::~Simulator() {
   if (sim_ != NULL)
      delete sim_;
}

void Simulator::Initialize() {
   assert(sim_ == NULL);
   assert(roads_ != NULL);

   // TODO Finish this interface.
   // sim_ = new hybrid::simulator s(roads_,
   //                                car_params_.avg_car_length,
   //                                car_params_.rear_axle);

   // s.micro_initialize(g_p::a_max,
   //                    g_p::a_pref,
   //                    g_p::v_pref,
   //                    g_p::delta);

   // s.macro_initialize(g_p::gamma, g_p::h_suggest, g_p::relaxation);

}

hwm::network* Simulator::get_roads() {
   return roads_;
}

void Simulator::set_roads(hwm::network* net) {
   roads_ = net;
}

SimMacroParameters Simulator::get_macro_parameters() {
   return macro_parameters_;
}

void Simulator::set_macro_parameters
(const SimMacroParameters& macro_parameters) {
   macro_parameters_ = macro_parameters;
}

SimMicroParameters Simulator::get_micro_parameters() {
   return micro_parameters_;
}

void Simulator::set_micro_parameters
(const SimMicroParameters& micro_parameters) {
   micro_parameters_ = micro_parameters;
}

CarParameters Simulator::get_car_parameters() {
   return car_parameters_;
}

void Simulator::set_car_parameters
(const CarParameters& car_parameters) {
   car_parameters_ = car_parameters;
}

}  // namespace wheelie
