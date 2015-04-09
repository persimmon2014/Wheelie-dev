#ifndef __ARZ_HPP__
#define __ARZ_HPP__

/**\defgroup libarz */
/**@{*/

#include "tvmet/Vector.h"
#include <iostream>
#include "libhybrid-common.hpp"

/** Class that implements the ARZ system of equations.
 *  For traffic flow.
 *  \tparam T The arithmetic type to use (to set precision).
 *  \todo Test out.
 */

/**Epsilon to use. */
#define VACUUM_EPS 1e-4

template <typename T>
struct arz
{
    /** Helper function.
     *  \returns An epsilon to use in calculations.
     */
    static       T   epsilon();

    /*@{*/
    /** \name arz_types Useful declarations.
     */
    enum   { order = 2 };
    typedef      T   prec_t;
    /*@}*/

    /** Simply class that encapsulates equations useful for ARZ.
     */
    struct eq
    {
        /** Compute 'relative velocity' y from rho and u (with u_max and gamma).
         *  \f[ \rho(u - u_{\textrm{eq}}\left(\rho, u_{\max}, \gamma\right) \f]
         *  \tparam T The arithmetic type for the computation.
         *  \param[in] rho The density.
         *  \param[in] u The velocity.
         *  \param[in] u_max The maximum velocity for the domain.
         *  \param[in] gamma  The gamma for the domain.
         *  \returns The above equation computed with the input params.
         */
        static inline T y(const T rho, const T u,
                          const T u_max,
                          const T gamma);

        /** Compute velocity u from rho and y (with u_max and gamma).
         *  \f[ \frac{y}{\rho} + u_{\textrm{eq}}\left(\rho, u_{\max}, \gamma\right) \f]
         *  \tparam T The arithmetic type for the computation.
         *  \param[in] rho The density.
         *  \param[in] y The relative velocity.
         *  \param[in] u_max The maximum velocity for the domain.
         *  \param[in] gamma  The gamma for the domain.
         *  \returns The above equation computed with the input params.
         */
        static inline T u(const T rho, const T y,
                          const T u_max,
                          const T gamma);

        /** Compute 'equilibrium velocity' u_eq from rho (with u_max and gamma).
         *  \f[ u_{\max}\left(1 - \rho^{\gamma}\right) \f]
         *  \tparam T The arithmetic type for the computation.
         *  \param[in] rho The density.
         *  \param[in] u_max The maximum velocity for the domain.
         *  \param[in] gamma  The gamma for the domain.
         *  \returns The above equation computed with the input params.
         */
        static inline T u_eq(const T rho,
                             const T u_max,
                             const T gamma);

        /** Compute inverse of 'equilibrium velocity' (rho) from u_eq (with 1/u_max and 1/gamma).
         *  \f[ \left(1 - \frac{u_{\textrm{eq}}}{u_{\max}}\right)^{\frac{1}{\gamma}} \f]
         *  \tparam T The arithmetic type for the computation.
         *  \param[in] u_eq The equilibrium velocity we are inverting.
         *  \param[in] inv_u_max One over the maximum velocity for the domain.
         *  \param[in] inv_gamma  One over the gamma for the domain.
         *  \returns The above equation computed with the input params.
         */
        static inline T inv_u_eq(const T u_eq,
                                 const T inv_u_max,
                                 const T inv_gamma);

        /** Compute derivative of 'equilibrium velocity' u_eq from rho (with u_max and gamma).
         *  \f[ -u_{\max}\gamma\rho^{\gamma-1} \f]
         *  \tparam T The arithmetic type for the computation.
         *  \param[in] rho The density.
         *  \param[in] u_max The maximum velocity for the domain.
         *  \param[in] gamma  The gamma for the domain.
         *  \returns The above equation computed with the input params.
         */
        static inline T u_eq_prime(const T rho,
                                   const T u_max,
                                   const T gamma);

        /** Compute left-middle state in inhomogeneous speedlimit situations
         *  \tparam T The arithmetic type for the computation.
         *  \param[in] flow_m_r The flux 0 component of the right cell
         *  \param[in] relv The difference between the left velocity and the left u_eq
         *  \param[in] u_max_l The speed limit of the left cell
         *  \param[in] gamma The gamma for the domain
         *  \returns The above equation computed with the input params.
         */
        static inline T rho_m_l_solve(const T flow_m_r,
                                      const T relv,
                                      const T u_max_l,
                                      const T gamma);
    };

    /** Unknowns for the ARZ equations.
     *  Based on tvmet::Vector.
     *  \f$\mathbf{q} = [ \rho, y ]^{\mathrm{T}}\f$
     */
    struct q : public tvmet::Vector<T, 2>
    {
        typedef tvmet::Vector<T, 2> base;

        /** Default constructor.
         *  Nothing is initialized.
         */
        q();
        /*@{*/
        /** Copy constructors from self, Vectors, and expression.
         */
        q(const q    &__restrict__ o);
        q(const base &__restrict__ o);
        template <class E>
        q(const tvmet::XprVector< E, 2 > &__restrict__ e);
        /*@}*/

        /** Constructor that directly sets rho and y.
         * \param in_rho The value for rho.
         * \param in_y The value for y.
         */
        q(const T in_rho, const T in_y);

        /** Constructor taking sets rho and u.
         * \param in_rho The value for rho.
         * \param in_u The speed to use.
         * \param u_max The maxiumum speed for this region.
         * \param gamma The gamma to use on this region.
         */
        q(const T in_rho, const T in_u,
          const T u_max,
          const T gamma);

        /** Accessor/mutator for rho
         * \returns rho.
         */
        /*@{*/
        inline const T  rho() const;
        inline       T& rho();
        /*@}*/

        /** Accessor/mutator for y
         * \returns y.
         */
        /*@{*/
        inline const T  y() const;
        inline       T& y();
        /*@}*/

        void fix();
        bool check() const;
    };

    /** Convenience class storing intermediate data useful in computation.
     */
    struct full_q : public q
    {
        typedef q base;

        /** Default constructor.
         *  Nothing is initialized.
         */
        full_q();

        /** Copy constructor.
         *  \param o The full_q to copy.
         */
        full_q(const full_q &__restrict__ o);

        /** Construct from a 'regular' q.
         *  \param o The q to create this instance from.
         *  \param u_max The maximum speed for this region.
         *  \param gamma The gamma on this region.
         */
        full_q(const      q &__restrict__ o,
               const T                    u_max,
               const T                    gamma);

        /** Construct from a rho, u pair.
         *  \param in_rho The density to use.
         *  \param in_u The speed to use.
         *  \param u_max The maximum speed for this region.
         *  \param gamma The gamma on this region.
         */
        full_q(const T in_rho,
               const T in_u,
               const T u_max,
               const T gamma);

        /** Zero out structure.
         */
        inline void clear();

        /** Accessor/mutator for u.
         *  \returns The speed u.
         */
        /*@{*/
        inline const T& u() const;
        inline       T& u();
        /*@}*/

        /** Accessor/mutator for u_eq.
         *  \returns The 'equilibrium' velocity u_eq.
         */
        /*@{*/
        inline const T& u_eq() const;
        inline       T& u_eq();
        /*@}*/

        /** Compute flux of this according to the ARZ equations.
         *  \returns A vector expression of the flux.
         */
        inline tvmet::XprVector<
            tvmet::XprBinOp<
                tvmet::Fcnl_mul<T, T>,
                tvmet::VectorConstReference<T, 2>,
                tvmet::XprLiteral<T>
                >,
            2 >  flux()   const;

        /** Compute flux of this according to the ARZ equations.
         *  \returns The 0- (rho-) component of the flux.
         */
        inline T flux_0() const;

        /** Compute flux of this according to the ARZ equations.
         *  \returns The 1- (y-) component of the flux.
         */
        inline T flux_1() const;

        /** Compute the first (rho) speed of the equations.
         *  \param u_max The maximum speed of in this region.
         *  \param gamma The gamma of this region.
         *  \returns The 0- (rho-) wave speed.
         */
        inline T lambda_0(const T u_max,
                          const T gamma) const;

        /** Compute the second (y) speed of the equations.
         *  \param u_max The maximum speed of in this region.
         *  \param gamma The gamma of this region.
         *  \returns The 1- (y-) wave speed.
         */
        inline T lambda_1(const T u_max,
                          const T gamma) const;

        bool check() const;

        T u_eq_; /**< Equilibrium velocity.*/
        T u_;    /**< Velocity. */
    };

    /** Compute the middle state of the centered_rarefaction.
     *  \f[\tilde{\rho}\left(0\right) = {\left(\frac{u_l + u_{\max}\rho_l^{\gamma}}{\left(\gamma+1\right)u_{\max}}\right)}^{\frac{1}{\gamma}}\f]
     *  \f[\tilde{u}\left(0\right)    = \frac{\gamma\left(u_l + u_{\max}\rho_l^\gamma\right)}{\gamma+1}\f]
     *  \param q_l The left state for this equation.
     *  \param u_max The maxiumum speed in this region.
     *  \param gamma The gamma for this region.
     *  \param inv_gamma 1.0/gamma for this region.
     *  \returns The centered rarefaction state.
     */
    static inline full_q centered_rarefaction(const full_q &__restrict__ q_l,
                                              const T                    u_max,
                                              const T                    gamma,
                                              const T                    inv_gamma);

    /** Compute q_m from a given left and right state.
     *  Note that this is only valid when \f$\rho_l^{\gamma} + \frac{u_l - u_r }{u_{\max} > 0\f$
     *  \f[\rho_m = {\left(\rho_l^{\gamma} + \frac{u_l - u_r }{u_{\max}}\right)}^{\frac{1}{\gamma}}\f]
     *  \param q_l The left state.
     *  \param q_l The right state.
     *  \param inv_u_max 1.0/u_max, the maximum velocity for the region.
     *  \param inv_gamma 1.0/gamma for the region.
     *  \return q_m as computed above.
     */
    static inline full_q rho_middle(const full_q &__restrict__ q_l,
                                    const full_q &__restrict__ q_r,
                                    const T                    inv_u_max,
                                    const T                    inv_gamma);

    /** Class to compute Riemann solutions for the ARZ equations.
     */
    struct riemann_solution
    {
        /** Compute the full riemann solution, store in this instance.
         *  \param q_l The left state.
         *  \param q_r The right state.
         *  \param u_max The maximum speed of the current region.
         *  \param inv_u_max 1.0/u_max.
         *  \param gamma The gamma for this region.
         *  \param inv_gamma 1.0/gamma.
         */
        inline void riemann(const full_q &__restrict__ q_l,
                            const full_q &__restrict__ q_r,
                            const T                    u_max,
                            const T                    inv_u_max,
                            const T                    gamma,
                            const T                    inv_gamma);


        /** Compute the full riemann solution with inhomogeneous speedlimits, store in this instance.
         *  \param q_l The left state.
         *  \param q_r The right state.
         *  \param u_max_l The maximum speed of the left cell
         *  \param u_max_r The maximum speed of the right cell.
         *  \param gamma The gamma for this region.
         *  \param inv_gamma 1.0/gamma.
         */
        inline void lebaque_inhomogeneous_riemann(const full_q &__restrict__ q_l,
                                                  const full_q &__restrict__ q_r,
                                                  float                      u_max_l,
                                                  float                      u_max_r,
                                                  float                      gamma,
                                                  float                      inv_gamma);

        /** Compute the one-sided riemann solution where there is no inflow, store in this instance.
         *  \param q_r The right state.
         *  \param u_max The maximum speed of the current region.
         *  \param inv_u_max 1.0/u_max.
         *  \param gamma The gamma for this region.
         *  \param inv_gamma 1.0/gamma.
         */
        inline void starvation_riemann(const full_q &__restrict__ q_r,
                                       const T                    u_max,
                                       const T                    inv_u_max,
                                       const T                    gamma,
                                       const T                    inv_gamma);

        /** Compute the one-sided riemann solution where there is no outflow, store in this instance.
         *  \param q_l The left state.
         *  \param u_max The maximum speed of the current region.
         *  \param inv_u_max 1.0/u_max.
         *  \param gamma The gamma for this region.
         *  \param inv_gamma 1.0/gamma.
         */
        inline void stop_riemann(const full_q &__restrict__ q_l,
                                 const T                    u_max,
                                 const T                    inv_u_max,
                                 const T                    gamma,
                                 const T                    inv_gamma);

        /** Zero-out data structure.
         */
        inline void clear();

        bool check() const;

        q waves[2];          /**< Storage for the two solution waves.*/
        q left_fluctuation;  /**< Storage for the left-going fluctuation.*/
        q right_fluctuation; /**< Storage for the right-going fluctuation.*/
        q q_0;               /**< Storage for the middle state of this solution.*/
        T speeds[2];         /**< Storage for the two speeds.*/
    };
};
/**@}*/

inline std::ostream &operator<<(std::ostream &os, const arz<float>::riemann_solution &rs)
{
    os << "Waves:\n"
       << 0   <<  " speed: " << rs.speeds[0] << "\n"
       << rs.waves[0] <<"\n"
       << "Left fluctuation:\n"
       << rs.left_fluctuation << "\n\n"
       << 1   <<  " speed: " << rs.speeds[1] << "\n"
       << rs.waves[1] <<"\n"
       << "Right fluctuation:\n"
       << rs.right_fluctuation << std::endl;
    return os;
}

#include "arz-eq.hpp"
#include "arz-impl.hpp"

#endif
