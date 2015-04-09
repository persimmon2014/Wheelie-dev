#!/usr/bin/python

import math

epsilon = 1e-3

def y(rho, u, u_max, gamma):
    return rho*(u - u_eq(rho, u_max, gamma))

def u(rho, y, u_max, gamma):
    if(rho < epsilon):
        return u_max;
    return max(y/rho + u_eq(rho, u_max, gamma), 0)

def u_eq(rho, u_max, gamma):
    assert rho >= 0.0;
    return u_max*(1.0 - math.pow(rho, gamma))

def inv_u_eq(u_eq, inv_u_max, inv_gamma):
    return math.pow(1.0 - u_eq*inv_u_max, inv_gamma)

def u_eq_prime(rho, u_max, gamma):
    if(rho < epsilon):
        return 0.0
    return -u_max*gamma*math.pow(rho, gamma - 1)

def fundamental_diagram(rho, relv, u_max, gamma):
    return rho*(u_eq(rho, u_max, gamma) + relv);

def critical_density(relv, u_max, gamma):
    return math.pow((u_max + relv)/(u_max*(1.0+gamma)), 1.0/gamma)

class full_q(object):
    def __init__(self, rho, u, u_max, gamma):
        self.rho  = rho
        self.u_eq = u_eq(self.rho, u_max, gamma)
        self.u    = min(self.u_eq, u)
        self.y    = self.rho*(self.u - self.u_eq)
    def lambda0(self, u_max, gamma):
        return self.u + self.rho*u_eq_prime(self.rho, u_max, gamma)
    def lambda1(self):
        return self.u
    def centered_rarefaction_rho(self, eta, u_max, gamma):
        return math.pow((self.u + u_max*math.pow(self.rho, gamma) - eta)/((gamma+1)*u_max), 1.0/gamma)
    def centered_rarefaction_u(self, eta, u_max, gamma):
        return (self.gamma*(self.u + u_max*math.pow(self.rho, gamma)) + eta)/(self.gamma + 1)

def rho_m(q_l, q_r, inv_u_max, inv_gamma):
    q_m      = full_q(0, 0, inv_u_max, inv_gamma)
    q_m.u_eq = q_r.u - q_l.u + q_l.u_eq
    q_m.rho  = math.pow(1.0 - q_m.u_eq*inv_u_max, inv_gamma)
    q_m.u    = q_r.u
    q_m.y    = q_m.rho*(q_m.u - q_m.u_eq)
    return q_m



