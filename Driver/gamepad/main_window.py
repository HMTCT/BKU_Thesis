import pygame
import time
import serial
import threading

# Initialize pygame
pygame.init()

serialPort = 'COM9'
serialBaudrate = 9600

# Initialize the joystick
joystick_count = pygame.joystick.get_count()

serialObject = serial.Serial(
    port=serialPort, 
    baudrate=serialBaudrate, 
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE, )

# INitialize input values
buffer = [0, 0, 0, 0, 0, 0]
newValue = False

# create the event
event = threading.Event()

def readData():
    while not event.is_set():
        data = ""
        while serialObject.inWaiting() > 0:
            data = serialObject.readline().decode(encoding='utf-8')
            print("Receieve data: ", data)

def handle_button_release(button, buffer):
    if button == 0:
        print("Button A released")
        buffer[5] = 0
    elif button == 1:
        print("Button B released")
        buffer[4] = 0
    elif button == 2:
        print("Button X released")
        buffer[4] = 0
    elif button == 3:
        print("Button Y released")
        buffer[5] = 0
    elif button == 4:
        print("Left bumper released")
    elif button == 5:
        print("Right bumper released")
    elif button == 6:
        print("Back button released")
    elif button == 7:
        print("Start button released")
    elif button == 8:
        print("Left stick button released")
    elif button == 9:
        print("Right stick button released")
    elif button == 10:
        print("Left trigger (L2) released")
    elif button == 11:
        print("Right trigger (R2) released")
    else: 
        print("Helloooooo")

def handle_button_press(button, buffer):
    if button == 0:
        print("Button A pressed")
        buffer[5] = 1
    elif button == 1:
        print("Button B pressed")
        buffer[4] = 2
    elif button == 2:
        print("Button X pressed")
        buffer[4] = 1
    elif button == 3:
        print("Button Y pressed")
        buffer[5] = 2
    elif button == 4:
        print("Left bumper pressed")
    elif button == 5:
        print("Right bumper pressed")
        buffer[5] = 1
    elif button == 6:
        print("Back button pressed")
    elif button == 7:
        print("Start button pressed")
    elif button == 8:
        print("Left stick button pressed")
    elif button == 9:
        print("Right stick button pressed")
    elif button == 10:
        print("Left trigger (L2) pressed")
    elif button == 11:
        print("Right trigger (R2) pressed")
    else: 
        print("Helloooooo")
    

def handle_axis_motion(axis, value, buffer):
    if axis == 0:  # X-axis of the left stick
        print(f"Left stick X-axis moved to {value}")
        if value >= -1.1 and value < -0.5:
            buffer[0] = 1
        elif value > 0.5 and value <=1.1:
            buffer[0] = 2
        else: buffer[0] = 0
    elif axis == 1:  # Y-axis of the left stick
        print(f"Left stick Y-axis moved to {value}")
        if value >= -1.1 and value < -0.5:
            buffer[1] = 1
        elif value > 0.5 and value <=1.1:
            buffer[1] = 2
        else: buffer[1] = 0
    elif axis == 2:  # X-axis of the right stick
        print(f"Right stick X-axis moved to {value}")
        if value >= -1.1 and value < -0.5:
            buffer[3] = 1
        elif value > 0.5 and value <=1.1:
            buffer[3] = 2
        else: buffer[3] = 0
    elif axis == 3:  # Y-axis of the right stick
        print(f"Right stick Y-axis moved to {value}")
        if value >= -1.1 and value < -0.5:
            buffer[2] = 1
        elif value > 0.5 and value <=1.1:
            buffer[2] = 2
        else: buffer[2] = 0
    elif axis == 4:  # Left trigger (L2)
        print(f"Left trigger (L2) value: {value}")
    elif axis == 5:  # Right trigger (R2)
        print(f"Right trigger (R2) value: {value}")
    elif axis == 6:  # D-pad X-axis
        handle_dpad_x(value)
    elif axis == 7:  # D-pad Y-axis
        handle_dpad_y(value)

def handle_dpad_x(value):
    if value == 1.0:
        print("D-pad right pressed")
    elif value == -1.0:
        print("D-pad left pressed")
    else:
        print("D-pad X-axis released")

def handle_dpad_y(value):
    if value == 1.0:
        print("D-pad down pressed")
    elif value == -1.0:
        print("D-pad up pressed")
    else:
        print("D-pad Y-axis released")

thread1 = threading.Thread(target=readData)

thread1.start()

if joystick_count > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"Joystick found: {joystick.get_name()}")

    try:
        while True:
            pygame.event.pump()
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    handle_button_press(event.button, buffer)
                if event.type == pygame.JOYBUTTONUP:
                    handle_button_release(event.button, buffer)
                if event.type == pygame.JOYAXISMOTION:
                    handle_axis_motion(event.axis, event.value, buffer)
            newValue = False
            for b in buffer:
                if b != 0:
                    newValue = True
            if newValue == True:
                string = ''.join(str(x) for x in buffer)
                string = '!' + string + '#'
                print(string)
                serialObject.write(bytes(str(string), encoding='utf-8'))
            time.sleep(100/100)

    except KeyboardInterrupt:
        event.set()
        pass

    finally:
        # Quit pygame
        pygame.quit()

else:
    print("No joystick found.")
