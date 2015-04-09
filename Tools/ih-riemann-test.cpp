#include "libhybrid/arz.hpp"

int main(int argc, char *argv[])
{
    std::cerr << libhybrid_package_string() << std::endl;

    arz<float>::q left(0.00158,
                       -0.02165);

    arz<float>::q right(0.0,
                        0.0);

    arz<float>::full_q fq_left(left,
                               33.33333,
                               0.5);

    arz<float>::full_q fq_right(right,
                                20.33333,
                                0.5);

    arz<float>::riemann_solution rs;
    rs.lebaque_inhomogeneous_riemann(fq_left,
                                     fq_right,
                                     33.33333,
                                     20.33333,
                                     0.5,
                                     1.0/0.5);

    std::cout << rs << std::endl;
    return 0;
}
