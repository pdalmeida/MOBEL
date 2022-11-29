'''
Created on Apr 5, 2012

@author: lanquarden
'''
import msgParser

class CarControl(object):
    '''
    An object holding all the control parameters of the car
    '''
    # TODO range check on set parameters

    def __init__(self, accel = 0.0, brake = 0.0, gear = 1, steer = 0.0, clutch = 0.0, focus = 0, meta = 0):
        '''Constructor'''
        self.parser = msgParser.MsgParser()
        
        self.actions = None
        
        self.accel = accel
        self.brake = brake
        self.gear = gear
        self.steer = steer
        self.clutch = clutch
        self.focus = focus
        self.meta = meta
        self.currAccel = 0
    
    def toMsg(self):
        self.actions = {}
        
        self.actions['accel'] = [self.accel]
        self.actions['brake'] = [self.brake]
        self.actions['gear'] = [self.gear]
        self.actions['steer'] = [self.steer]
        self.actions['clutch'] = [self.clutch]
        self.actions['focus'] = [self.focus]
        self.actions['meta'] = [self.meta]
        
        return self.parser.stringify(self.actions)
    
    def setAccel(self, accel):
        print(accel)
        self.accel = accel
    
    def getAccel(self):
        return self.accel

    def setCurrAccel(self, accel):
        self.currAccel = accel
    
    def getCurrAccel(self):
        return self.currAccel
    
    def setBrake(self, brake):
        self.brake = brake
    
    def getBrake(self):
        return self.brake
    
    def setGear(self, gear):
        self.gear = gear
    
    def getGear(self):
        return self.gear
    
    def setSteer(self, steer):
        self.steer = steer
    
    def getSteer(self):
        return self.steer
    
    def setClutch(self, clutch):
        self.clutch = clutch
    
    def getClutch(self):
        return self.clutch
    
    def setMeta(self, meta):
        self.meta = meta
    
    def getMeta(self):
        return self.meta
        
        