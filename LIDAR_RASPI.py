#Import GPIO library
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)



#Lidar pin definition
PINS_LIDAR_MODES=[29,31,33,35,37]
PINS_LIDAR_ZONES=[11,13,15]


#Set pins for lidar modes as output
for pin in PINS_LIDAR_MODES: GPIO.setup(pin, GPIO.OUT)

#Set pins for lidar zones as input
for pin in PINS_LIDAR_ZONES: GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)


#Set a lidar mode
for pin in PINS_LIDAR_MODES:
	GPIO.output(pin, GPIO.HIGH)

#Read Zones
for pin in PINS_LIDAR_ZONES:
	print(GPIO.input(pin))

