class KalmanAngle:

    def __init__(self):

        self.QAngle = 0.001                     # Q (ANGLE) unknown uncertainty from the environment
        self.QBias = 0.003                      # Q (BIAS) unknown uncertainty from the environment. Here - covariance is degree of correlation between variances of the angle and its error/bias.     
        self.RMeasure = 0.1                     
        self.angle = 0.0
        self.bias = 0.0
        self.rate = 0.0
        self.P=[[0.0,0.0],[0.0,0.0]]

    def getAngle(self,newAngle, newRate,dt):

        #step 1: Predict new state (for our case - state is angle) from old state + known external influence

        self.rate = newRate - self.bias    #new_rate is the latest Gyro measurement
        self.angle += dt * self.rate

        #Step 2: Predict new uncertainty (or covariance) from old uncertainity and unknown uncertainty from the environment. 

        self.P[0][0] += dt * (dt*self.P[1][1] -self.P[0][1] - self.P[1][0] + self.QAngle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.QBias * dt

        #Step 3: Innovation i.e. predict th next measurement

        y = newAngle - self.angle

        #Step 4: Innovation covariance i.e. error in prediction 

        s = self.P[0][0] + self.RMeasure

        #Step 5:  Calculate Kalman Gain

        K=[0.0,0.0]
        K[0] = self.P[0][0]/s
        K[1] = self.P[1][0]/s

        #Step 6: Update the Angle

        self.angle += K[0] * y
        self.bias  += K[1] * y

        #Step 7: Calculate estimation error covariance - Update the error covariance

        P00Temp = self.P[0][0]
        P01Temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00Temp
        self.P[0][1] -= K[0] * P01Temp
        self.P[1][0] -= K[1] * P00Temp
        self.P[1][1] -= K[1] * P01Temp

        return self.angle


    def setAngle(self,angle):
        self.angle = angle

    def setQAngle(self,QAngle):
        self.QAngle = QAngle

    def setQBias(self,QBias):
        self.QBias = QBias

    def setRMeasure(self,RMeasure):
        self.RMeasure = RMeasure

    def getRate():
        return self.rate

    def getQAngle():
        return self.QAngle

    def getQBias():
        return self.QBias

    def  getRMeasure():   
        return self.RMeasure