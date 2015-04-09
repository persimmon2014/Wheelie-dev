#ifndef __ARZ_EQ_HPP__
#define __ARZ_EQ_HPP__

#include <cmath>

template <typename T>
inline T arz<T>::eq::y(const T rho, const T u,
                       const T u_max,
                       const T gamma)
{
    return rho*(u - u_eq(rho, u_max, gamma));
}

template <typename T>
inline T arz<T>::eq::u(const T rho, const T y,
                       const T u_max,
                       const T gamma)
{
    if(rho < epsilon())
        return u_max;
    return std::max(y/rho + u_eq(rho, u_max, gamma), static_cast<T>(0)); // u is at least 0, no negative velocity allowed!
}

template <typename T>
inline T arz<T>::eq::u_eq(const T rho,
                          const T u_max,
                          const T gamma)
{
    assert(rho >= 0.0f);
    return u_max*(1.0 - std::pow(rho, gamma));
}

template <typename T>
inline T arz<T>::eq::inv_u_eq(const T u_eq,
                              const T inv_u_max,
                              const T inv_gamma)
{
    return std::pow(1 - u_eq*inv_u_max, inv_gamma);
}

template <typename T>
inline T arz<T>::eq::u_eq_prime(const T rho,
                                const T u_max,
                                const T gamma)
{
    if(rho < epsilon())
        return 0.0;
    return -u_max*gamma*std::pow(rho, gamma - 1); // take the derivative of u_eq
}


template <typename T, class f>
inline T secant(const T x0, const T x1, const T bottom, const T top, const T tol, const int maxiter, const f &fnc)
{
    T xn_1 = x0;
    T xn   = x1;
    T fn_1 = fnc(xn_1);
    T fn   = fnc(xn);
    T denom = fn-fn_1;
    for(int i = 0; std::abs(fn) > tol && std::abs(denom) > tol && i < maxiter; ++i)
    {
        T newxn = xn - (xn - xn_1)/denom * fn;
        while(newxn <= bottom && newxn != xn)
            newxn = (newxn + xn)*0.5;
        while(newxn >= top && newxn != xn)
            newxn = (newxn + xn)*0.5;
        xn_1 = xn;
        xn = newxn;
        fn_1 = fn;
        fn = fnc(xn);
        denom = (fn-fn_1);
    }
    return xn;
}

template <typename T>
inline T fundamental_diagram(const T rho, const T relv, const T u_max, const T gamma)
{
    return rho*(arz<T>::eq::u_eq(rho, u_max, gamma) + relv);
}

template <typename T>
inline T critical_density(const T relv, const T u_max, const T gamma)
{
    return std::pow((u_max + relv)/(u_max*(1.0f+gamma)), 1.0f/gamma);
}

template <typename T>
inline T max_flow(const T relv, const T u_max, const T gamma)
{
    return fundamental_diagram(critical_density(relv, u_max, gamma), relv, u_max, gamma);
}

template <typename T>
inline T demand(T rho, const T relv, const T u_max, const T gamma)
{
    const T crit = critical_density(relv, u_max, gamma);
    if(rho > crit)
        rho = crit;
    return fundamental_diagram(rho, relv, u_max, gamma);
}

template <typename T>
struct inv_fd
{
    inv_fd(const T flow, const T relv, const T u_max, const T gamma) :
        flow_(flow), relv_(relv), u_max_(u_max), gamma_(gamma)
    {}

    T operator()(const T rho) const
    {
        const T fd0 = fundamental_diagram(rho, relv_, u_max_, gamma_);
        return fd0-flow_;
    }

    T flow_;
    T relv_;
    T u_max_;
    T gamma_;
};

template <typename T>
inline T inv_demand(const T flow, const T relv, const T u_max, const T gamma)
{
     const T crit = critical_density(relv, u_max, gamma);
     const T max_flow = fundamental_diagram(crit, relv, u_max, gamma);

    if(flow + 1e-3f >= max_flow)
        return crit;

    const inv_fd<T> solver(flow, relv, u_max, gamma);
    return secant<T, inv_fd<T> >(crit*0.3f, crit, 1e-4, crit, 5e-8, 500, solver);
}

template <typename T>
inline T inv_supply(const T flow, const T relv, const T u_max, const T gamma)
{
    const T crit = critical_density(relv, u_max, gamma);
    const T max_flow = fundamental_diagram(crit, relv, u_max, gamma);

    if(flow + 1e-3f >= max_flow)
        return crit;

    const inv_fd<T> solver(flow, relv, u_max, gamma);
    return secant<T, inv_fd<T> >(crit*1.3f, 1.0, crit, 1.0, 5e-8, 500, solver);
}

template <typename T>
inline T supply(T rho, const T relv, const T u_max, const T gamma)
{
    const T crit = critical_density(relv, u_max, gamma);
    if(rho <= crit)
        rho = crit;
    return fundamental_diagram(rho, relv, u_max, gamma);
}
#endif
