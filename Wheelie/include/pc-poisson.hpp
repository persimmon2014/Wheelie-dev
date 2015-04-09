#ifndef _PC_POISSON_HPP_
#define _PC_POISSON_HPP_

#include "pc-integrate.hpp"
#include <iostream>
namespace pproc
{
    struct drand
    {
        double operator()() const
        {
            return drand48();
        }
    };

    template <typename T>
    inline T exp_rvar(const T r)
    {
        return -std::log(r);
    }

    template <typename RAND, typename PC_T>
    struct inhomogeneous_poisson
    {
        typedef typename PC_T::real_t real_t;

        inhomogeneous_poisson(const real_t in_start, const PC_T &in_pc, RAND *r) : t(in_start),
                                                                                   integrator(&in_pc),
                                                                                   arg(integrator.integrate(t)),
                                                                                   rand(r)
        {
        }

        real_t next()
        {
            const real_t u  = (*rand)();
            const real_t e  = exp_rvar(u);
            arg            += e;
            t               = integrator.inv_integrate(arg);
            return t;
        }

        real_t next_trunc(const float trunc)
        {
            const real_t        u  = (*rand)();
            pc_integrator<PC_T> i2(integrator);
            const real_t        d  = 1 - std::exp(-(i2.integrate(trunc) - arg));
            const real_t        e  = exp_rvar(1 - d*u);
            arg                   += e;
            t                      = integrator.inv_integrate(arg);
            return t;
        }

        real_t              t;
        pc_integrator<PC_T> integrator;
        real_t              arg;
        RAND               *rand;
    };

    template <typename T, typename F>
    inline T texp(const T start, const T end, F &in_pc)
    {
        const T lambda_start = in_pc.integrate(start);
        F       i2(in_pc);
        const T d            = 1 - std::exp(-(i2.integrate(end) - lambda_start));
        const T u            = drand48();
        return in_pc.inv_integrate(lambda_start + exp_rvar(1 - d*u));
    }

    template <typename RAND, typename PC_T>
    std::vector<typename PC_T::real_t> poisson_points(const typename PC_T::real_t start, const typename PC_T::real_t end, const size_t quota, const typename PC_T::real_t sep, const PC_T &pc, RAND &r)
    {
        typedef typename PC_T::real_t     real_t;
        inhomogeneous_poisson<RAND, PC_T> ipp(start, pc, r);
        std::vector<real_t>               res;
        real_t                            candidate = ipp.next();
        while(res.size() < quota && candidate < end)
        {
            res.push_back(candidate);

            while(1)
            {
                inhomogeneous_poisson<RAND, PC_T> ipp_copy(ipp);
                candidate                  = ipp.next();
                if(candidate - res.back() >= sep)
                    break;
                ipp                        = ipp_copy;
            }
        }

        return res;
    }
}
#endif
