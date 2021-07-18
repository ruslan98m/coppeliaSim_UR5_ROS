
from math import *
import numpy as np


class MPC_controller():
    def __init__(self, p, m, n, dt):
        self.p = p
        self.m = m
        self.dt = dt
        self.n = n
        self.freq = 1/dt
        self.I = np.eye(self.n)
        self.O = np.zeros((self.n, self.n), dtype=float, order='C' )
        

        #Ie calculate
        self.Ie = self.I
        for i in range(1, m):
            self.Ie = np.concatenate((self.Ie, self.O), axis=1)

        #Ia and Ib calculate
        self.Ia = self.I
        self.Ib = self.I
        for i in range(2, p+1):
            self.Ia = np.concatenate((self.Ia, self.I), axis=1)
            self.Ib = np.concatenate((self.Ib, self.I.dot(i)), axis=1)
        self.Ib = self.Ib.dot(self.dt)

        self.Ia = np.transpose(self.Ia)
        self.Ib = np.transpose(self.Ib)
        #A calculate
        self.A = np.zeros((n*p, n*m), dtype=float, order='C')
        for i in range(self.p):
            for j in range(self.m):
                if(i<j):
                    self.A[i*n:i*n+n, j*n:j*n+n] = self.O
                else:
                    self.A[i*n:i*n+n, j*n:j*n+n] = self.I.dot((2*i-2*j+1)/2)
 
        self.A = self.A.dot(dt**2)
        #
        one = np.ones((1, n))        
        self.Gq = np.eye(p*n)*np.concatenate((one*np.full(n, 20), one*np.full(n, 20), one*np.full(n, 30), one*np.full(n, 10), one*np.full(n, 10)), axis=1)
        self.Gw = np.eye(p*n)*np.concatenate((one*np.full(n, 0.2), one*np.full(n, 0.4), one*np.full(n, 0.3), one*np.full(n, 0.2), one*np.full(n, 0.1)), axis=1)
        self.Ki = np.eye(p*n)*0.1   #0.003
        #
        self.Gq_t = np.transpose(self.Gq)
        self.Gw_t = np.transpose(self.Gw)
        self.A_t = np.transpose(self.A)
        #
        self.H = self.A_t.dot(self.Gq_t).dot(self.Gq).dot(self.A) + self.Gw_t.dot(self.Gw)
        self.Kmpc = self.Ie.dot(np.linalg.inv(self.H)).dot(self.A_t).dot(self.Gq_t).dot(self.Gq)
        



