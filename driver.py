'''
Created on Apr 4, 2012

@author: lanquarden
'''

import msgParser
import carState
import carControl
import math as m
from simple_pid import PID



class Driver(object):
    '''
    A driver object for the SCRC
    '''

    def __init__(self, stage):
        '''Constructor'''
        self.WARM_UP = 0
        self.QUALIFYING = 1
        self.RACE = 2
        self.UNKNOWN = 3
        self.stage = stage
        
        self.parser = msgParser.MsgParser()
        
        self.state = carState.CarState()
        
        self.control = carControl.CarControl()
        
        self.steer_lock = 0.785398
        self.max_speed = 170
        self.prev_rpm = 0
        self.RAD2DEG = m.pi / 180.0
        self.DEG2RAD = 1.0 / self.RAD2DEG
        self.PP_K = 0.50                      # bias - increase to reduce steering sensivity
        self.PP_L = 2.4                         # aprox vehicle wheelbase
        self.PP_2L = 2 * self.PP_L
        self.MAX_STEER_ANGLE = 21

        self.EDGE_MAX_POS = 0.85
        self.EDGE_STEER = 0.0075

        self.DEF_MIN_SPEED = 50
        self.DEF_MAX_SPEED = 250

        self.GEAR_MAX = 6
        self.RPM_MAX = 8000
        self.ACCEL_MAX = 1.0
        self.ACCEL_DELTA = 0.5          #maximum rate of change in acceleration signal, avoid spinning out
        self.BRAKE_MAX = -0.5           #braking signal <= BRAKING_MAX 
        self.BRAKE_DELTA = 0.05         #dampen braking to avoid lockup, max rate of chage in braking
        self.WSPIN_ACCEL_DELTA = 0.025
        self.WSPIN_MAX = 5.0            #greater than this value --> loss of control


    def init(self):
        '''Return init string with rangefinder angles'''
        self.angles = [0 for x in range(19)]
        
        for i in range(5):
            self.angles[i] = -90 + i * 15
            self.angles[18 - i] = 90 - i * 15
        
        for i in range(5, 9):
            self.angles[i] = -20 + (i-5) * 5
            self.angles[18 - i] = 20 - (i-5) * 5
        
        return self.parser.stringify({'init': self.angles})
    
    def drive(self, msg):
        self.state.setFromMsg(msg)
        
        self.steer()
        
        self.gear()
        
        self.speed()
        
        return self.control.toMsg()
    
    def steer(self):
        '''
        angle = self.state.angle
        dist = self.state.trackPos
        print(self.state.getTrack())
        
        self.control.setSteer((angle - dist*0.5)/self.steer_lock)
        '''

        target_angle = self.computeTargetAngle()
        # alpha (a) = angle of longest sensor (... -20, -10, 0, 10, 20, ...)
        rawSteeringAngle = - m.atan(self.PP_2L * m.sin(target_angle) / (self.PP_K * self.state.getSpeed()))
        # normalize between [-1,1]
        normalizedSteeringAngle = self.state.clamp(rawSteeringAngle/self.MAX_STEER_ANGLE,-1.0,1.0)
        #print(normalizedSteeringAngle)
        self.control.setSteer(normalizedSteeringAngle*100)

        # edge avoidance code
        edgeSteerCorrection = 0
        # too far left
        if self.state.trackPos > self.EDGE_MAX_POS and edgeSteerCorrection < 0.005:
            edgeSteerCorrection = -self.EDGE_STEER
        # too far right
        elif self.state.trackPos < -self.EDGE_MAX_POS and edgeSteerCorrection > -0.005:
            edgeSteerCorrection = self.EDGE_STEER

        self.control.setSteer(self.control.getSteer() + edgeSteerCorrection)        # correct steer


    def computeTargetAngle(self):
        targetAngle = self.state.getMaxDistanceAngle() * self.RAD2DEG
        return targetAngle

    def speed(self):
        '''
        speed = self.state.getSpeedX()
        accel = self.control.getAccel()
        
        if speed < self.max_speed:
            accel += 0.1
            if accel > 1:
                accel = 1.0
        else:
            accel -= 0.1
            if accel < 0:
                accel = 0.0
        
        self.control.setAccel(accel)
        '''

        brakingZone = self.state.getMaxDistance() < (self.state.speedX / 1.5)
        targetSpeed = 0
        hasWheelSpin = False

        if (brakingZone):
            targetSpeed = max(self.DEF_MIN_SPEED,self.state.getMaxDistance())
        else:
            targetSpeed = self.DEF_MAX_SPEED

        '''
        #wheel spin velocity
        wheelSpinVelocity = self.state.getWheelSpinVel()
        frontWheelAvgSpeed = (wheelSpinVelocity[0] + wheelSpinVelocity[1]) / 2.0
        rearWheelAvgSpeed = (wheelSpinVelocity[2] + wheelSpinVelocity[3]) / 2.0
        if rearWheelAvgSpeed != 0:
           slippagePercent = frontWheelAvgSpeed / rearWheelAvgSpeed * 100.0

        wheelSpinDelta = abs((frontWheelAvgSpeed / 2) - (rearWheelAvgSpeed/ 2))   
        hasWheelSpin = self.state.getSpeedX() > 5.0 and slippagePercent < 80.00
        if hasWheelSpin:
            accel = self.control.getCurrAccel() - self.WSPIN_ACCEL_DELTA
            self.control.setCurrAccel(accel)
          
        else:  
        '''    


        #
        # CONTROLO PID
        pid = PID(1,1,5,targetSpeed)        #kp, ki, kd, ref
        accel = pid(self.state.getSpeed())

        accel = self.state.clamp(accel,self.BRAKE_MAX,self.ACCEL_MAX)
        self.control.setCurrAccel(accel)

        if accel > 0.0:
            self.control.setAccel(accel)
            self.control.setBrake(0)
        elif accel < 0.0:
            self.control.setAccel(0)
            self.control.setBrake(abs(accel))
        else:
            self.control.setAccel(0.0)
            self.control.setBrake(0.0)



    def gear(self):
        
        rpm = self.state.getRpm()
        gear = self.state.getGear()
        accel = self.control.getAccel()
        speed = self.state.getSpeedX()

        if gear == 1:
            if rpm > 5000:
                gear+=1
            elif rpm < 1000:
                gear-=1
        elif gear == 2:
            if rpm > 7000:
                gear+=1
            elif rpm < 1000:
                gear-=1                
        elif gear == 3:
            if rpm > 7000:
                gear+=1
            elif rpm < 4000:
                gear-=1        
        elif gear == 4:
            if rpm > 7000:
                gear+=1
            elif rpm < 4000:
                gear-=1
        elif gear == 5:
            if rpm > 8500:
                gear+=1
            elif rpm < 4000:
                gear-=1
        elif gear == 6:
            if rpm < 4000:
                gear-=1

        self.control.setGear(gear)






        if accel > 0.7 and rpm > 8000:
            gear += 1
        elif accel < 0 and rpm < 4000:
            gear -= 1

        self.control.setGear(gear)


        '''
        if self.prev_rpm == 0:
            up = True
        else:
            if (self.prev_rpm - rpm) < 0:
                up = True
            else:
                up = False
        
        if up and rpm > 8000:
            gear += 1
        
        if not up and rpm < 4000:
            gear -= 1
        
        self.control.setGear(gear)

        '''
        '''
        accel = self.control.getAccel()
        rpm = self.state.getRpm()
        currAccel = self.control.getCurrAccel()
        gear = self.control.getGear()

        if accel > 0: #gas
            accel = self.state.clamp(accel,0.0, currAccel + self.ACCEL_DELTA)
            if gear == 0 or rpm > self.RPM_MAX:
                gear+=1
        elif accel < 0: #brake
            accel = self.state.clamp(accel,currAccel + self.BRAKE_DELTA,0.0)
            if rpm < self.RPM_MAX*0.75:
                gear-=1          


        gear = self.state.clamp(gear,1,self.GEAR_MAX)
        self.control.setGear(gear)        
        '''

        
    def onShutDown(self):
        pass
    
    def onRestart(self):
        pass
        