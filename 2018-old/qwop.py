import serial
import sys, pygame
pygame.init()

size = width, height = 320, 240
ser = serial.Serial("/dev/ttyACM2")

screen = pygame.display.set_mode(size)

while 1:
    while ser.in_waiting:
        print(ser.read())

    for event in pygame.event.get():
        if event.type == pygame.QUIT: sys.exit()
        elif event.type == pygame.KEYDOWN:
            if event.key == 97: #a
                ser.write(bytes([1,64]))
            elif event.key == 100: # d
                ser.write(bytes([1,192]))
            elif event.key == 119: # w
                ser.write(bytes([2,192]))
            elif event.key == 115: # s
                ser.write(bytes([2,64]))   
				
            elif event.key == 113: # q
                ser.write(bytes([3,192]))
            elif event.key == 101: #e
                ser.write(bytes([3,64]))
        elif event.type == pygame.KEYUP:
            if event.key == 97 or event.key == 100:
                ser.write(bytes([1,0]))
            elif event.key == 119 or event.key == 115:
                ser.write(bytes([2,0]))
            elif event.key == 113 or event.key == 101:
                ser.write(bytes([3,0]))
