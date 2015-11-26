# coding: utf-8
import numpy
from numba import double
from numba.decorators import jit
import time

@jit
def pairwise_numba(X, D):
    M = X.shape[0]
    N = X.shape[1]
    for i in range(M):
        for j in range(M):
            d = 0.0
            for k in range(N):
                tmp = X[i, k] - X[j, k]
                d += tmp * tmp
            D[i, j] = numpy.sqrt(d)


def pairwise_python(X, D):
    M = X.shape[0]
    N = X.shape[1]
    for i in range(M):
        for j in range(M):
            d = 0.0
            for k in range(N):
                tmp = X[i, k] - X[j, k]
                d += tmp * tmp
            D[i, j] = numpy.sqrt(d)


if __name__ == '__main__':
    t = time.time()
    X = numpy.random.random((1000, 3))
    D = numpy.empty((1000, 1000))
    pairwise_python(X, D)
    print "python:", time.time() - t

    t = time.time()
    X = numpy.random.random((1000, 3))
    D = numpy.empty((1000, 1000))
    pairwise_numba(X, D)
    print "numba:", time.time() - t