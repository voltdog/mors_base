from math import *

class SignalGenerator(object):
    def __init__(self):
        pass

    # type:
    #   horiz
    #   lin
    #   sin
    def set_params(self, data=[[2,0], [4,2], [1,4]], type='horiz', dt=0.01):
        self.data = data
        self.type = type
        self.cur_pt = 0
        self.out = 0

        if type == 'lin':
            if data[0][1] != 0:
                data.insert(0, [0,0])
            if data[0][1] == 0:
                self.out = self.data[0][0]
            else:
                self.out = 0
        elif type == 'sin':
            self.A = data[0]
            self.w = data[1]
            self.phi = data[2]
            self.bias = data[3]
        self.inc = 0
        self.dt = dt


    def step(self, i):
        
        if self.type == 'horiz':
            self.out = self.__step_horiz(i)
        elif self.type == 'lin':
            self.out = self.__step_lin(i)
        elif self.type == 'sin':
            self.out = self.__step_sin(i)
        return self.out

    def __step_horiz(self, i):
        if i < self.data[0][1]:
            return 0

        if self.cur_pt < (len(self.data)-1):
            if i < self.data[self.cur_pt+1][1]:
                return self.data[self.cur_pt][0]
            else:
                self.cur_pt += 1
                return self.data[self.cur_pt][0]
        else:
            return self.data[self.cur_pt][0]

    def __step_lin(self, i):
        if i == 0:
            return self.out
        
        elif self.cur_pt < (len(self.data)-1):
            if i > self.data[self.cur_pt+1][1]:
                self.cur_pt += 1
            if self.cur_pt >= (len(self.data)-1):
                self.inc = 0
            else:
                dt_int = (self.data[self.cur_pt+1][1] - self.data[self.cur_pt][1])/self.dt
                if dt_int != 0:
                    self.inc = (self.data[self.cur_pt+1][0] - self.data[self.cur_pt][0])/(dt_int)
                else:
                    self.inc = 0
        else:
            self.inc = 0
        
        self.out += self.inc
        return self.out

    def __step_sin(self, i):
        self.out = self.A*sin(self.w*i + self.phi) + self.bias
        return self.out