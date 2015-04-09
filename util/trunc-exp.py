import math
import numpy
import scipy
import matplotlib
import pylab

def exp_pdf(x, lam):
    return lam*math.exp(-x*lam)

def exp_rvar(x):
    return -math.log(x)

def texp_rvar(x, lam, s, e):
    return -math.log(1-(1-math.exp(-lam*(e-s)))*x)/lam+s

if __name__ == '__main__':
    lam = 1
    pts = [texp_rvar(x,lam, 1, 10000) for x in scipy.random.rand(1000000)]
    x = numpy.linspace(0, max(pts), 100)
    pylab.hist(pts,150, normed=True)
#    pylab.plot(x, [exp_pdf(i, lam) for i in x], color='#FF0000', linewidth=2)
    pylab.show()
