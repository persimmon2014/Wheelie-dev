#ifndef _PC_INTEGRATE_HPP_
#define _PC_INTEGRATE_HPP_

#include <cmath>
#include <cassert>
#include <vector>
#include <algorithm>
#include <boost/foreach.hpp>
#include "libhybrid-common.hpp"

// poisson process
namespace pproc
{
    template <typename T>
    struct pc_data
    {
        typedef T                                            real_t;
        typedef std::vector<real_t>                          arr_t;

        pc_data(const real_t in_dx, const arr_t &in_data, const real_t in_inf)
            : dx_(in_dx), data_(in_data), inf_(in_inf)
        {
        }

        real_t operator[](const size_t i) const
        {
            return data_[i];
        }

        real_t dx() const
        {
            return dx_;
        }

        real_t inf() const
        {
            return inf_;
        }

        real_t end() const
        {
            return data_.size()*dx();
        }

        size_t n() const
        {
            return data_.size();
        }

        void write(std::ostream &o) const
        {
            o << data_.size() << " " << dx() << " ";
            for(size_t i = 0; i < data_.size(); ++i)
                o << data_[i] << " ";
            o << inf() << std::endl;
        }

        T     dx_;
        arr_t data_;
        T     inf_;
    };

    template <typename PC_T>
    struct pc_integrator
    {
        typedef typename PC_T::real_t real_t;

	/**
	 * initialize poisson process integrator
	 * @param in_pc lane_poisson_helper which includes cell size, number of cells, q for the entire lane and scale = 1/car_length
	 */
        pc_integrator(const PC_T *in_pc) : pc(in_pc), current_cell(0), current_sum(0.0) {}

        /**
	 * initialize poisson process integrator
	 * @param o poisson process integrator
	 */
        pc_integrator(const pc_integrator<PC_T> *o)
            : pc(o.pc), current_cell(o.current_cell), current_sum(o.current_sum) {}

        void reset()
        {
            current_cell = 0;
            current_sum  = 0;
        }

        /**
	 * Sum cell lengths till x
	 * @param x integration ending point
	 * @return total number of cars
	 */
        real_t integrate(const real_t x)
        {
            assert(x >= current_cell*pc->dx());  // current_cell is () returns the cell size
	    
	    //std::cout<<current_sum<<" "<<pc->dx()<<std::endl;
	    
            while((current_cell+1)*pc->dx() < x)
            {
                if(current_cell >= pc->n())
                    break;

                current_sum += (*pc)[current_cell]*pc->dx();
                ++current_cell;
		
		//std::cout<<current_cell<<" "<<pc->dx()<<" "<<current_sum<<std::endl;
            }

            /* i.e. if (current_cell < data.size())
             * const real_t local = x/pc->dx() - current_cell;
             * return current_sum + local*(*pc)[current_cell]*pc->dx();
             * else
             * const real_t local = x - current_cell*pc->dx();
             * return current_sum + local*pc->inf();
             */

            const real_t last_cell_density  = (current_cell < pc->n()) ? (*pc)[current_cell] : pc->inf();
            const real_t last_cell_coverage = x - current_cell*pc->dx();
	    
	    //std::cout<<"aa: "<<( current_sum + local*last)<<std::endl;
            return current_sum + last_cell_coverage*last_cell_density;
        }

        real_t inv_integrate(const real_t v)
        {
            assert(v >= current_sum);

            while(current_cell < pc->n() && current_sum + (*pc)[current_cell]*pc->dx() < v)
            {
                current_sum += (*pc)[current_cell]*pc->dx();
                ++current_cell;
            }

            const real_t denom = current_cell < pc->n() ? (*pc)[current_cell] : pc->inf();
            return (v - current_sum)/denom + current_cell*pc->dx();
        }

        const PC_T *pc; // lane_poisson_helper which includes cell size, number of cells, q for the entire lane and scale = 1/car_length
        size_t      current_cell;
        real_t      current_sum;
    };

    template <typename F, typename T>
    pc_data<T> pc_from_func(const F &func, const T dx, const size_t n)
    {
        typename pc_data<T>::arr_t data(n);
        T x = 0;
        for(T &e: data)
        {
            e  = 0.5*(func(x) + func(x+dx));
            x += dx;
        }
        return pc_data<T>(dx, data, func(x));
    }

    template <typename T>
    pc_data<T> pc_from_avg(const typename pc_data<T>::arr_t &obs, const T dx, const size_t n)
    {
        const T inv_dx = 1/dx;
        typename pc_data<T>::arr_t data(n, 0);
        for(const T &o: obs)
        {
            if(o < 0.0)
                continue;
            const size_t idx  = o*inv_dx;
            if(idx >= n)
                continue;
            data[idx]        += inv_dx;
        }
        return pc_data<T>(dx, data, 0);
    }
}
#endif
