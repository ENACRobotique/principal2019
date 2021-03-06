import RPi.GPIO as GPIO

class lidarZone():
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        self.zone1 = 0
        self.zone2 = 0
        self.zone3 = 0
        
        
        self.pins_modes=[29,31,33,35,37]
        self.pins_zones=[11,13,15]
        
        for pin in self.pins_modes: 
            GPIO.setup(pin, GPIO.OUT)
        for pin in self.pins_zones: 
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        self.set_zones_forward_without_leeway_high()
        
    """Correspondances zones logiciel :
    areadesigner = n ==> pins = 31-n
    """
    
    def set_zones_forward_left_only(self):
        GPIO.output(self.pins_modes[0], GPIO.LOW)
        GPIO.output(self.pins_modes[1], GPIO.LOW)
        GPIO.output(self.pins_modes[2], GPIO.HIGH)
        GPIO.output(self.pins_modes[3], GPIO.LOW)
        GPIO.output(self.pins_modes[4], GPIO.HIGH)
        
    def set_zones_backward_left_only(self):
        GPIO.output(self.pins_modes[0], GPIO.HIGH)
        GPIO.output(self.pins_modes[1], GPIO.LOW)
        GPIO.output(self.pins_modes[2], GPIO.HIGH)
        GPIO.output(self.pins_modes[3], GPIO.LOW)
        GPIO.output(self.pins_modes[4], GPIO.HIGH)
        
    def set_zones_forward_right_only(self):
        GPIO.output(self.pins_modes[0], GPIO.LOW)
        GPIO.output(self.pins_modes[1], GPIO.HIGH)
        GPIO.output(self.pins_modes[2], GPIO.HIGH)
        GPIO.output(self.pins_modes[3], GPIO.LOW)
        GPIO.output(self.pins_modes[4], GPIO.HIGH)
        
    def set_zones_backward_right_only(self):
        GPIO.output(self.pins_modes[0], GPIO.HIGH)
        GPIO.output(self.pins_modes[1], GPIO.HIGH)
        GPIO.output(self.pins_modes[2], GPIO.LOW)
        GPIO.output(self.pins_modes[3], GPIO.LOW)
        GPIO.output(self.pins_modes[4], GPIO.HIGH)
        
    def set_zones_backward_with_leeway_high(self):
        GPIO.output(self.pins_modes[0], GPIO.HIGH)
        GPIO.output(self.pins_modes[1], GPIO.HIGH)
        GPIO.output(self.pins_modes[2], GPIO.LOW)
        GPIO.output(self.pins_modes[3], GPIO.HIGH)
        GPIO.output(self.pins_modes[4], GPIO.HIGH)
        
    def set_zones_backward_with_leeway_low(self):
        GPIO.output(self.pins_modes[0], GPIO.HIGH)
        GPIO.output(self.pins_modes[1], GPIO.LOW)
        GPIO.output(self.pins_modes[2], GPIO.HIGH)
        GPIO.output(self.pins_modes[3], GPIO.HIGH)
        GPIO.output(self.pins_modes[4], GPIO.HIGH)
        
    def set_zones_backward_without_leeway_low(self):
        GPIO.output(self.pins_modes[0], GPIO.HIGH)
        GPIO.output(self.pins_modes[1], GPIO.LOW)
        GPIO.output(self.pins_modes[2], GPIO.LOW)
        GPIO.output(self.pins_modes[3], GPIO.HIGH)
        GPIO.output(self.pins_modes[4], GPIO.HIGH)
        
    def set_zones_backward_without_leeway_high(self):
        GPIO.output(self.pins_modes[0], GPIO.HIGH)
        GPIO.output(self.pins_modes[1], GPIO.HIGH)
        GPIO.output(self.pins_modes[2], GPIO.HIGH)
        GPIO.output(self.pins_modes[3], GPIO.LOW)
        GPIO.output(self.pins_modes[4], GPIO.HIGH)
        
    def set_zones_forward_with_leeway_high(self):
        GPIO.output(self.pins_modes[0], GPIO.LOW)
        GPIO.output(self.pins_modes[1], GPIO.LOW)
        GPIO.output(self.pins_modes[2], GPIO.HIGH)
        GPIO.output(self.pins_modes[3], GPIO.HIGH)
        GPIO.output(self.pins_modes[4], GPIO.HIGH)
        
    def set_zones_forward_with_leeway_low(self):
        GPIO.output(self.pins_modes[0], GPIO.LOW)
        GPIO.output(self.pins_modes[1], GPIO.HIGH)
        GPIO.output(self.pins_modes[2], GPIO.HIGH)
        GPIO.output(self.pins_modes[3], GPIO.HIGH)
        GPIO.output(self.pins_modes[4], GPIO.HIGH)
        
    def set_zones_forward_without_leeway_low(self):
        GPIO.output(self.pins_modes[0], GPIO.LOW)
        GPIO.output(self.pins_modes[1], GPIO.HIGH)
        GPIO.output(self.pins_modes[2], GPIO.LOW)
        GPIO.output(self.pins_modes[3], GPIO.HIGH)
        GPIO.output(self.pins_modes[4], GPIO.HIGH)
        
    def set_zones_forward_without_leeway_high(self):
        GPIO.output(self.pins_modes[0], GPIO.LOW)
        GPIO.output(self.pins_modes[1], GPIO.LOW)
        GPIO.output(self.pins_modes[2], GPIO.LOW)
        GPIO.output(self.pins_modes[3], GPIO.HIGH)
        GPIO.output(self.pins_modes[4], GPIO.HIGH)
        
    def set_zones_null(self):
        GPIO.output(self.pins_modes[0], GPIO.LOW)
        GPIO.output(self.pins_modes[1], GPIO.LOW)
        GPIO.output(self.pins_modes[2], GPIO.LOW)
        GPIO.output(self.pins_modes[3], GPIO.LOW)
        GPIO.output(self.pins_modes[4], GPIO.LOW)
    
    def __repr__(self):
        return "zone 1 : {}; zone 2 = {}; zone 3 = {}".format(self.zone1,self.zone2,self.zone3)
    
    def update(self,zone1,zone2,zone3):
        self.zone1 = zone1
        self.zone2 = zone2
        self.zone3 = zone3
        
    def update_GPIO(self):
        self.zone1 = GPIO.input(self.pins_zones[0])
        self.zone2 = GPIO.input(self.pins_zones[1])
        self.zone3 = GPIO.input(self.pins_zones[2])
    
    def activated_zone1(self):
        return self.zone1 == 1
    
    def activated_zone2(self):
        return self.zone2 == 1
        
    def activated_zone3(self):
        return self.zone3 == 1
    