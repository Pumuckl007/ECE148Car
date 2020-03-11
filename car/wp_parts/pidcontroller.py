class PIDController():
    def __init__(self, p=0, i=0, d=0, f=0):
        self.p = p
        self.i = i
        self.d = d
        self.f = f
        self.acc = 0
        self.output = 0
        self.lastData = 0
        self.setting = 0
        self.delta = 0
        self.dt = 1

    def sample(self, data, dt=1):
        self.dt = dt
        self.acc = self.acc * 0.9
        self.acc = self.acc + data*0.1
        self.delta = data - self.lastData
        self.lastData = data
        self.compute()

    def compute(self):
        fOutput = self.setting * self.f
        pOutput = (self.setting - self.lastData) * self.p
        iOutput = self.acc * self.i
        dOutput = self.delta * self.d / self.dt
        self.output = fOutput + pOutput + iOutput + dOutput

    def getOutput(self):
        return self.output

    def set(self, setting):
        self.setting = setting
        self.compute()
