#!/usr/bin/python

from matplotlib import rc
import matplotlib
import numpy as np
import pylab
import itertools as it
import operator as op
import sys
import re
import math

def to_u(rho, y, umax, gamma):
    return y/rho + ueq(rho, umax, gamma)

def to_y(rho, u, umax, gamma):
    return rho*(u - ueq(rho, umax, gamma))

def lambda0(rho, y, umax, gamma):
    u = to_u(rho, y, umax, gamma)
    return u + rho*ueq_prime(rho, umax, gamma)

def lambda1(rho, y, umax, gamma):
    u = to_u(rho, y, umax, gamma)
    return u

def ueq(rho, umax, gamma):
    return umax*(1.0-rho**gamma)

def inv_ueq(u, umax, gamma):
    return math.pow(1 - u/umax, 1.0/gamma)

def ueq_prime(rho, umax, gamma):
    return -umax*gamma*rho**(gamma-1)

def invariant0(rho, y, umax, gamma):
    u = to_u(rho, y, umax, gamma)
    return u - ueq(rho, umax, gamma)

def invariant1(rho, y, umax, gamma):
    u = to_u(rho, y, umax, gamma)
    return u

def fundiag(x, umax, gamma):
    return x*(ueq(x, umax, gamma))

def critval(wl, umax, gamma):
    return ((wl + umax)/(umax*(gamma+1.0)))**(1.0/gamma)

class fundamental_diagram(object):
    def __init__(self, umax, gamma, irel):
        self.umax = umax
        self.gamma = gamma
        self.irel = irel
        self.critical_rho = critval(self.irel, self.umax, self.gamma)
        self.critical_flux = self[self.critical_rho]
    def plot(self, ax, n, **kwargs):
        x = []
        y = []
        dx = 1/float(n)
        current_x = 0
        current_y = self[current_x]
        while current_x < 1:
            if current_y >= 0:
                x.append(current_x)
                y.append(current_y)
            else:
                last_x = current_x-dx
                last_y = self[current_x-dx]
                if current_y * last_y < 0:
                    m = (current_y - last_y)/dx
                    b = current_y - m*current_x
                    x.append(-b/m)
                    y.append(0)
            current_x += dx
            current_y = self[current_x]
        return ax.plot(np.array(x), np.array(y), **kwargs)

class supply(fundamental_diagram):
    def __init__(self, umax, gamma, irel):
        fundamental_diagram.__init__(self, umax, gamma, irel)
    def __getitem__(self, x):
        if(x < self.critical_rho):
            return self.critical_flux
        else:
            return fundiag(x, self.umax, self.gamma) + x*self.irel

class demand(fundamental_diagram):
    def __init__(self, umax, gamma, irel):
        fundamental_diagram.__init__(self, umax, gamma, irel)
    def __getitem__(self, x):
        if(x > self.critical_rho):
            return self.critical_flux
        else:
            return fundiag(x, self.umax, self.gamma) + x*self.irel

def maxflux(gamma):
    return gamma*math.pow(1.0/(gamma+1.0), (gamma+1.0)/gamma)

if __name__ == '__main__':
    # gamma = 0.5

    # u_max_l = 4.3
    # u_max_r = 2.3

    # rho_l = 0.00158
    # u_l = 2.3

    # rho_r = 0.0
    # u_r = u_max_r

    # irel = u_l - ueq(rho_l, u_max_l, gamma)
    # rho_m = inv_ueq(u_r - irel, u_max_r, gamma)

    # print irel, rho_m

    # d = demand(u_max_l, gamma, irel)
    # s = supply(u_max_r, gamma, irel)

    # fsize = 20
    # rc('text', usetex=True)

    # x = np.linspace(0, 1, 50)

    # ax = pylab.axes()
    # ax.add_line(matplotlib.lines.Line2D((rho_l, rho_l), (0, max(0, d[rho_l])), color='orange'))
    # ax.add_line(matplotlib.lines.Line2D((rho_m, rho_m), (0, max(0, s[rho_m])), color='blue'))

    # d.plot(ax, 100, color='orange', label="demand (right)")
    # s.plot(ax, 100, color='blue', label="suppy (left)")
    # ax.legend()

    x = np.linspace(0,1, 1000)
    y = [fundiag(i,1, 0.01) for i in x]
    pylab.plot(x, y)

    if(len(sys.argv) == 2):
        pylab.savefig(sys.argv[1])
    else:
        pylab.show()
    pylab.clf()
