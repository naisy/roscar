 #!/usr/bin/python
# coding: utf-8
# Logicool F310の裏面にあるX-D switchをDに設定するとTX2で認識する
# Logicool F710の場合は、X-D switchをDにして、正面の電源ボタン（Logicoolロゴのボタン）を押すとTX2で認識する
# roscore&
# python run_roscar.py
# python joycontrol.py
# https://www.pygame.org/docs/ref/joystick.html
import pygame
import curses
from lib.rostopic import RosTopicSetter

def map(x, in_min, in_max, out_min, out_max):
    value = (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
    if value < out_min:
        value = out_min
    elif value > out_max:
        value = out_max
    return value


def main(win):

    win.nodelay(True)
    win.keypad(True)
    key=-1

    ros_topic_setter = RosTopicSetter()
    speed = 0
    steering_angle = 0
    MAX_STEERING_ANGLE = 45

    pygame.init()

    #Loop until the user clicks the close button.
    done = False

    # Used to manage how fast the screen updates
    clock = pygame.time.Clock()

    # Initialize the joysticks
    pygame.joystick.init()


    # -------- Main Program Loop -----------
    while done == False:
        msg = ""
        key = win.getch()
        if key == ord('q') or key == 27: # push q or ESC key
            done = True
            pygame.quit()
            break

        # EVENT PROCESSING STEP
        for event in pygame.event.get(): # User did something
            if event.type == pygame.QUIT: # If user clicked close
                done = True # Flag that we are done so we exit this loop
                break
            # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
            if event.type == pygame.JOYBUTTONDOWN:
                msg += "Joystick button pressed.\n"
            if event.type == pygame.JOYBUTTONUP:
                msg += "Joystick button released.\n"

        if done:
            break
        # Get count of joysticks
        joystick_count = pygame.joystick.get_count()

        # For each joystick:
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()

            # Get the name from the OS for the controller/joystick
            name = joystick.get_name()
            msg += "Joystick name: {}\n".format(name)

            # Usually axis run in pairs, up/down for one, and left/right for
            # the other.
            axes = joystick.get_numaxes()
            msg += "Number of axes: {}\n".format(axes)

            for i in range( axes ):
                axis = joystick.get_axis( i )
                msg += "Axis {} value: {:>6.3f}\n".format(i, axis)

            """
            Steering
            axis(0): LEFT: <0, RIGHT: >0
            STEERING: LEFT: <0, RIGHT: >0
            """
            axis = joystick.get_axis(3)
            steering_angle = map(axis, -1.0, 0.99, -1*MAX_STEERING_ANGLE, MAX_STEERING_ANGLE)
            msg += "=============================\n"
            msg += "steering_angle:{}\n".format(steering_angle)
            ros_topic_setter.angle(int(steering_angle))

            """
            Speed
            axis(3): UP: <0, DOWN: >0
            """
            axis = -1.0*joystick.get_axis(1)
            speed = int(map(axis, -1.0, 0.99, -100, 100))
            msg += "speed:{}\n".format(speed)
            ros_topic_setter.motor(int(speed))

            buttons = joystick.get_numbuttons()
            msg += "Number of buttons: {}\n".format(buttons)

            for i in range( buttons ):
                button = joystick.get_button( i )
                msg += "Button {:>2} value: {}\n".format(i,button)

            # Hat switch. All or nothing for direction, not like joysticks.
            # Value comes back in an array.
            hats = joystick.get_numhats()
            msg += "Number of hats: {}\n".format(int(hats))

            for i in range( hats ):
                hat = joystick.get_hat( i )
                msg += "Hat {} value: {}\n".format(i, str(hat))

        msg += "Press q or ESC to quit."
        win.clear()
        win.addstr(msg)
        win.refresh()
        # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT

        # Limit to 20 frames per second
        clock.tick(20)

    # Close the window and quit.
    # If you forget this line, the program will 'hang'
    # on exit if running from IDLE.
    pygame.quit()

curses.wrapper(main)
