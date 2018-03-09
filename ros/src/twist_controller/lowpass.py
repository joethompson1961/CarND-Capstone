
# tau = LPF time constant
# ts = time step (seconds)
# when ts is increased the weight of previous value decreases while weight of new value increases
class LowPassFilter_naive(object):
    def __init__(self, tau, ts):
        self.a = 1. / ((tau / ts) + 1.0)
        self.b = tau / (tau + ts)

        self.last_val = 0.
        self.ready = False

    def get(self):
        return self.last_val

    def filt(self, val):
        if self.ready:
            val = self.a * val + self.b * self.last_val
        else:
            val = 0.0
            self.ready = True

        self.last_val = val
        return val

class LowPassFilter_smooth(object):
    def __init__(self, lamda):
        self.lamda = lamda
        
    def get(self):
        return self.last_val
    
    def filt(self, val):
        if self.ready:
            val = self.last_val / math.e^(self.lamda*dt) + val / (1 - 1/e^(self.lamda*dt)) 
        else:
            val = 0.0
            self.ready = True
            
        self.last_val = val
        return val
    