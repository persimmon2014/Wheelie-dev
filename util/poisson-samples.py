#!/usr/bin/python

import math
import random
import itertools as it
import bisect
import numpy
import pylab
import matplotlib

def exp_pdf(x, lam):
    return lam*math.exp(-lam*x)

def exp_cdf(x, lam):
    return 1 - math.exp(-lam*x)

def exp_quantile(p, lam):
    return -math.log(1 - p)/lam

def exp_rvar(p):
    return -math.log(p)

def lam_f(t):
    return 1.0

def cap_lam_f(t):
    return 1.0*t

def inv_cap_lam_f(t):
    return t/1.0

def inhomo_poisson_inv(start, cap_obj):
    t = start
    k = 0
    while True:
        E = exp_rvar(random.random())
#        print E
        k += 1
        t = cap_obj.inv_integral(E + cap_obj.integral(t))
        yield t

# class cap(object):
#     def value(self, t):
#         pass
#     def integral(self, t):
#         pass
#     def inv_integral(self, t):
#         pass

class cap(object):
    def value(self, t):
        return 1
    def integral(self, t):
        return t
    def inv_integral(self, t):
        return t

class ecap(object):
    def value(self, t):
        return math.exp(t)
    def integral(self, t):
        return math.exp(t)
    def inv_integral(self, t):
        return math.log(t)

def lam(x):
    vals      = [0.5]
    intervals = [50.0]

    p = bisect.bisect_left(intervals, x)
    if(p < len(intervals)):
        return vals[p]
    raise Exception("Undefined lambda value")

def thinning_poisson_car(limit, lam, cap_obj):
    t = 0.25
    k = 0
    while True:
        Z = inhomo_poisson_inv(t, cap_obj).next()
        U = random.random()
        t = Z
        if t > limit:
            return
        if U <= lam(Z)/cap_obj.value(Z):
            k += 1
            t += 0.75
            yield Z

def test():
    gen = thinning_poisson_car(50, lam, cap())
    try:
        while True:
            print gen.next()
    except StopIteration:
        pass

def show_poisson_proc(ax, start_end, lam, cap_obj):
    gen = thinning_poisson(start_end[0], start_end[1], lam, cap_obj)
    res = []
    try:
        while True:
            res.append(gen.next())
    except StopIteration:
        pass

    x = numpy.linspace(start_end[0], start_end[1])
    y = numpy.array([lam(i) for i in x])
    ax.plot(x, y)
    maxy = max(y)
    for i in res:
        l0 = matplotlib.lines.Line2D((i, i), (0, maxy), color='black', linewidth=1.0)
        ax.add_line(l0)

    return ax

def thinning_poisson(start, limit, lam, cap_obj):
    t = start
    k = 0
    while True:
        Z = inhomo_poisson_inv(t, cap_obj).next()
        U = random.random()
        t = Z
        if t > limit:
            return
        if U <= lam(Z)/cap_obj.value(Z):
            k += 1
            yield Z

if __name__ == '__main__':
    pylab.clf()
    show_poisson_proc(pylab.axes(), (0, 2), lambda x: math.exp(x), ecap())
    pylab.show()




