import msgParser
import carState
import carControl
import math
import matplotlib as plt

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
        self.maxSpeedDist  = 70
        self.prev_rpm = None


    
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
        angle = self.state.angle
        trackPos = self.state.trackPos
        rSensor = self.state.getTrack(10)   #+5 degrees
        cSensor = self.state.getTrack(9)
        lSensor = self.state.getTrack(8)    #-5 degrees

        self.accel = 0
        self.gear = self.state.gear
        brakingZone = self.state.m



        '''
        #self.control.setSteer(angle - trackPos*0.5)

        #print('LS: ' + str(lSensor) + ' CS: ' + str(cSensor) + ' RS: ' + str(rSensor))
        print(trackPos)


        
        
        if lSensor == rSensor:
            self.control.setSteer(angle - trackPos*0.5)

        if lSensor > (10+rSensor):
            if trackPos !=-1:
                self.control.setSteer(-0.02)
            else:   
                self.control.setSteer(0)
        elif rSensor > (10+lSensor):
            if trackPos !=1:
                self.control.setSteer(0.02)
            else:   
                self.control.setSteer(0)
        '''

                


        
    
    def gear(self):
        rpm = self.state.getRpm()
        gear = self.state.getGear()
        
        if self.prev_rpm == None:
            up = True
        else:
            if (self.prev_rpm - rpm) < 0:
                up = True
            else:
                up = False
        
        if up and rpm > 7000:
            gear += 1
        
        if not up and rpm < 3000:
            gear -= 1
        
        self.control.setGear(gear)
    
    def speed(self):
        """
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
        """

        trackPos = self.state.getTrackPos()
        #print('POS:' + str(trackPos))

        if trackPos < 1 and trackPos > -1:
            rSensor = self.state.getTrack(10)   #+5 degrees
            cSensor = self.state.getTrack(9)
            lSensor = self.state.getTrack(8)    #-5 degrees            

            if cSensor > self.maxSpeedDist or (cSensor >= rSensor and cSensor >=lSensor):
                targetSpeed = self.max_speed
            else:
                if (rSensor > lSensor):
                    h = cSensor*math.sin(5)
                    b = rSensor - cSensor*math.cos(5)
                    sinAngle = b*b/(h*h+b*b)            #compute angle of turn(???)
                    targetSpeed = self.max_speed*(cSensor*sinAngle/self.maxSpeedDist)    #estimate the target speed depending on turn and on how close it is
                else:
                    h = cSensor*math.sin(5)
                    b = lSensor - cSensor*math.cos(5)
                    sinAngle = b*b/(h*h+b*b)
                    targetSpeed = self.max_speed*(cSensor*sinAngle/self.maxSpeedDist)

            self.control.setAccel(2/(1+math.exp(self.state.getSpeedX() - targetSpeed)) - 1)
        else:
            self.control.setAccel(0.3)
   
    def onShutDown(self):
        pass
    
    def onRestart(self):
        pass
        