#!/usr/bin/python
# coding: utf-8

# run
# ssh -CY user@jetson_ip_address
# export DISPLAY=:0
# python3 gamepad_test.py

# Tested Gamepad: Logicool F710
# F710 mode: X-D switch: X and MODE LED: off

# sudo apt-get update
# sudo apt-get install -y libsdl2-dev libsdl2-image-dev libsdl2-mixer-dev libsdl2-net-dev libsdl2-ttf-dev libsdl2-gfx-dev
# sudo apt-get install -y libfreetype6-dev libportmidi-dev
# sudo -H pip3 install wheel
# sudo -H pip3 install pygame

# https://www.pygame.org/docs/ref/joystick.html
import pygame
import curses

def main(win):

    win.nodelay(True)
    win.keypad(True)
    key=-1

    pygame.init()

    #Loop until the user clicks the close button.
    done = False

    # Used to manage how fast the screen updates
    clock = pygame.time.Clock()

    # Initialize the joysticks
    pygame.joystick.init()

    # Get count of joysticks
    joystick_count = pygame.joystick.get_count()
    joystick = pygame.joystick.Joystick(0)

    # -------- Main Program Loop -----------
    while done == False:
        msg = ""
        key = win.getch()
        if key == ord('q') or key == 27: # push q or ESC key
            done = True
            pygame.quit()
            break

        # EVENT PROCESSING STEP
        # The event queue needs to be pumped frequently for some of the methods to work. So call one of pygame.event.get, pygame.event.wait, or pygame.event.pump regularly.
        for event in pygame.event.get(): # User did something
            if event.type == pygame.QUIT: # If user clicked close
                done = True # Flag that we are done so we exit this loop
                break
            # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
            # And in pygame 2, supports hotplugging: JOYDEVICEADDED JOYDEVICEREMOVED
            elif event.type == pygame.JOYBUTTONDOWN:
                #msg += "Joystick button pressed.\n"
                pass
            elif event.type == pygame.JOYBUTTONUP:
                #msg += "Joystick button released.\n"
                pass
            else:
                pass

        if done:
            break

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
        steering_value = joystick.get_axis(0)
        msg += "=============================\n"
        msg += "steering_value:{:>6.3f}\n".format(steering_value)

        """
        Throttle
        axis(4): UP: <0, DOWN: >0
        """
        throttle_value = joystick.get_axis(4)
        msg += "throttle_value:{:>6.3f}\n".format(throttle_value)
        msg += "=============================\n"

        trackballs = joystick.get_numballs()
        msg += "Number of trackballs: {}\n".format(trackballs)

        for i in range(trackballs):
            trackball = joystick.get_ball(i)
            msg += "TrackBall {:>2} value: {}\n".format(i,trackball)

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

