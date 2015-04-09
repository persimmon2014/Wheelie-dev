import math
import numpy
import matplotlib
import matplotlib.pyplot as pyplot
from matplotlib import rc
import bisect
import random
import sys

car_length = 4.5

class pc_data(object):
    def __init__(self, dx, data, inf):
        self.dx = dx
        self.inf = inf
        self.n = len(data)
        self.build(data)
    def __iter__(self):
        def it(start, obj):
            i = start
            while i < obj.n:
                yield obj[i]
                i += 1
        return it(0, self)
    def end(self):
        return self.n*self.dx
    def build(self, data):
        self.integration = numpy.zeros((self.n+1))
        for i in xrange(1, len(self.integration)):
            self.integration[i] = self.integration[i-1] + data[i-1]*self.dx
    def __getitem__(self, idx):
        if(idx >= self.n):
            return self.inf
        return (self.integration[idx+1] - self.integration[idx])/self.dx
    def integrate(self, t):
        if t >= self.n*self.dx:
            local = t - self.n*self.dx
            return self.integration[-1] + local*self.inf
        else:
            scaled = t/self.dx
            cell = math.floor(scaled)
            local = scaled - cell
            return (1.0-local)*self.integration[cell] + local*self.integration[cell+1]
    def inv_integrate(self, v, start=0):
        if v < self.integration[0]:
            raise Exception("Value is beyond range of integral!")
        if v > self.integration[-1]:
            last_v = self.integration[-1]
            t = (v-last_v)/self.inf + self.n*self.dx
            next_start = len(self.integration)-1
        else:
            assert(start + 1 < len(self.integration))
            idx = bisect.bisect_right(self.integration, v, start)
            last_v = self.integration[idx-1]
            t = (v-last_v)/self[idx-1] + (idx-1) * self.dx
            next_start = idx-1
        return t, next_start
    def plot(self, ax, data, integrate, extra):
        assert extra >= self.n*self.dx
        if data:
            x = numpy.linspace(0, self.dx*(self.n-1), self.n)
            pl = numpy.zeros(self.n)
            for i in xrange(0, self.n):
                pl[i] = self[i]

            b = ax.bar(x, pl, width=self.dx, color='#dcad2d', ec='none', label=r'$\lambda(x)=\frac{1}{l}\rho(x)$')
        if integrate:
            x = numpy.linspace(0, self.dx*self.n, self.n+1)
            p = ax.plot(list(x)+[extra], list(self.integration)+[self.integration[-1] + self.inf*(extra-x[-1])], label='$\Lambda(x) = \int_0^x \lambda(t)d\,t$', lw=2, color='#2c6b26')
            for i in xrange(1, len(x)):
                ax.add_line(matplotlib.lines.Line2D((x[i], x[i], 0),(0, self.integration[i], self.integration[i]),
                                                    color='black', ls=':'))

        return ax

def pc_string(str):
    lst = str.split()
    n = int(lst[0])
    dx = float(lst[1])
    inf = float(lst[-1])
    data = [float(x) for x in lst[2:-1]]
    assert len(data) == n
    return pc_data(dx, data, inf)

def pc_func(func, dx, n):
    data = numpy.zeros((n))
    x = 0
    for i in xrange(0, n):
        data[i] = 0.5*(func(x) + func(x+dx))
        x += dx
    return pc_data(dx, data, func(x))

def pc_avg(places, dx, n):
    data = numpy.zeros((n))
    inv_dx = 1.0/dx
    for p in places:
        data[int(math.floor(p*inv_dx))] += 1
    for i in xrange(len(data)):
        data[i] *= inv_dx
    return pc_data(dx, data, 0)

def plot_inv_web(ax, pc, y):
    x,ignore = pc.inv_integrate(y)
    l0 = matplotlib.lines.Line2D((0, x), (y, y), color='black', linewidth=1.0)
    l1 = matplotlib.lines.Line2D((x, x), (0, y), color='black', linewidth=1.0)
    ax.add_line(l0)
    ax.add_line(l1)
    return ax

def plot_web(ax, pc, x):
    y = pc.integrate(x)
    l0 = matplotlib.lines.Line2D((0, x), (y, y), color='black', linewidth=1.0)
    l1 = matplotlib.lines.Line2D((x, x), (0, y), color='black', linewidth=1.0)
    ax.add_line(l0)
    ax.add_line(l1)
    return ax

def exp_rvar(p):
    return -math.log(p)

def ih_poisson(start, end, pc, limit=0):
    t = start
    arg = pc.integrate(t)
    last_start = 0
    count = 0
    while True:
        U = random.random()
        E = exp_rvar(U)
        # t = pc.inv_integrate(E + pc.integrate(t))
        arg += E
        t, last_start = pc.inv_integrate(arg, last_start)
        if t >= end or (limit > 0 and count > limit):
            return
        yield t
        count += 1

def show_poisson_proc(ax, start_end, pc):
    ax = pc.plot(ax, True, False, max(start_end[1], pc.end()))

    gen = ih_poisson(start_end[0], start_end[1], pc)
    res = list(gen)
    maxy = max(max(pc), pc.inf)
    ax = plot_events(ax, res, maxy)

    return ax

def plot_events(ax, res, height):
    l0 = matplotlib.lines.Line2D((res[0], res[0]), (0, height), color='green', linewidth=1.0, label='vehicle location')
    ax.add_line(l0)
    for i in res[1:]:
        l0 = matplotlib.lines.Line2D((i, i), (0, height), color='green', linewidth=1.0)
        ax.add_line(l0)
    return ax

if __name__ == '__main__':
    rc('text', usetex=True)
    getn = open("pc.txt")
    i = pc_string(getn.readline().rstrip())
    gen = [float(x) for x in getn.readline().rstrip().split()]

    pyplot.figure(figsize=(8, 4))
    ax = pyplot.axes([0.065, 0.1, 0.9, 0.8])

    ax = i.plot(ax, True, False, i.end())

    ax = plot_events(ax, gen, 1.0)

    pyplot.title("Lane density and instantiated cars")
    pyplot.xlabel("$x$ (m)")
    pyplot.ylabel("Cars per meter")
    ax.axis([0, 2000, 0, 1])

    pyplot.legend(loc=0)

    if len(sys.argv) > 1:
        pyplot.savefig(sys.argv[1])
    else:
        pyplot.show()
