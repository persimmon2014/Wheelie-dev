#include "hybrid-sim.hpp"
#include "timer.hpp"

int main(int argc, char *argv[])
{
    std::cout << libroad_package_string() << std::endl;
    std::cerr << libhybrid_package_string() << std::endl;
    if(argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <network file>" << std::endl;
        return 1;
    }

    hwm::network net(hwm::load_xml_network(argv[1], vec3f(1.0, 1.0, 1.0f)));

    net.build_intersections();
    net.build_fictitious_lanes();
    net.auto_scale_memberships();
    net.center();
    std::cerr << "HWM net loaded successfully" << std::endl;

    try
    {
        net.check();
        std::cerr << "HWM net checks out" << std::endl;
    }
    catch(std::runtime_error &e)
    {
        std::cerr << "HWM net doesn't check out: " << e.what() << std::endl;
        exit(1);
    }

    hybrid::simulator s(&net,
                        4.5f,
                        1.0);
    s.micro_initialize(0.73,
                       1.67,
                       33,
                       4);
    s.macro_initialize(0.5, 8.1*4.5, 0.0f);

    for(hybrid::lane &l: s.lanes)
    {
        l.sim_type = hybrid::MICRO;
        l.populate(0.25/s.car_length, s);
        l.convert_to_macro(s);
    }

    int   num_steps  = 0;
    float total_time = 0;
    timer step_timer;
    while(1)
    {
      //std::cout << "asdf" << num_steps<<std::endl;
        step_timer.reset();
        step_timer.start();
        float dt = s.hybrid_step();
	
	//std::cout<<dt<<std::endl;
        s.advance_intersections(dt);
        ++num_steps;
        step_timer.stop();
        total_time += step_timer.interval_S();
        //std::cout << "\r" << num_steps << " " << dt << " " << step_timer.interval_S() << " " << total_time/num_steps<<std::endl;
        //std::cout.flush();
    }
    return 0;
}
