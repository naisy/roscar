#!/usr/bin/python
# coding: utf-8
# Logcool F310の裏面にあるX-D switchをDに設定するとTX2で認識する
# roscore&
# python run_roscar.py
# python joycontrol.py
# https://www.pygame.org/docs/ref/joystick.html
import pygame
from lib.rostopic import RosTopicSetter

def map(x, in_min, in_max, out_min, out_max):
    value = (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
    if value < out_min:
        value = out_min
    elif value > out_max:
        value = out_max
    return value
ros_topic_setter = RosTopicSetter()
speed = 0
steering_angle = 0
MAX_STEERING_ANGLE = 45

# Define some colors
BLACK    = (   0,   0,   0)
WHITE    = ( 255, 255, 255)

# This is a simple class that will help us print to the screen
# It has nothing to do with the joysticks, just outputting the
# information.
class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def pprint(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10


pygame.init()

# Set the width and height of the screen [width,height]
size = [500, 700]
screen = pygame.display.set_mode(size)

pygame.display.set_caption("My Game")

#Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

# Initialize the joysticks
pygame.joystick.init()

# Get ready to print
textPrint = TextPrint()

# -------- Main Program Loop -----------
while done==False:
    # EVENT PROCESSING STEP
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done=True # Flag that we are done so we exit this loop

        # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
        if event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
        if event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")


    # DRAWING STEP
    # First, clear the screen to white. Don't put other drawing commands
    # above this, or they will be erased with this command.
    screen.fill(WHITE)
    textPrint.reset()

    # Get count of joysticks
    joystick_count = pygame.joystick.get_count()

    textPrint.pprint(screen, "Number of joysticks: {}".format(joystick_count) )
    textPrint.indent()

    # For each joystick:
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()

        textPrint.pprint(screen, "Joystick {}".format(i) )
        textPrint.indent()

        # Get the name from the OS for the controller/joystick
        name = joystick.get_name()
        textPrint.pprint(screen, "Joystick name: {}".format(name) )

        # Usually axis run in pairs, up/down for one, and left/right for
        # the other.
        axes = joystick.get_numaxes()
        textPrint.pprint(screen, "Number of axes: {}".format(axes) )
        textPrint.indent()

        for i in range( axes ):
            axis = joystick.get_axis( i )
            textPrint.pprint(screen, "Axis {} value: {:>6.3f}".format(i, axis) )
        textPrint.unindent()
        """
        Steering
        axis(0): LEFT: <0, RIGHT: >0
        STEERING: LEFT: <0, RIGHT: >0
        """
        axis = joystick.get_axis(0)
        steering_angle = map(axis, -1.0, 0.99, -1*MAX_STEERING_ANGLE, MAX_STEERING_ANGLE)
        print("=============================")
        print("steering_angle:{}".format(steering_angle))
        ros_topic_setter.angle(int(steering_angle))

        """
        Speed
        axis(3): UP: <0, DOWN: >0
        """
        axis = -1.0*joystick.get_axis(3)
        speed = int(map(axis, -1.0, 0.99, -100, 100))
        print("speed:{}".format(speed))
        ros_topic_setter.motor(int(speed))
        """
        Speed
        axis(5): BRAKE: >-1.0
        """
        """
        axis = joystick.get_axis(5)
        print("axis:{}".format(axis))
        if axis > -0.99:
            speed = 0
            ros_topic_setter.brake(True)
        else:
            ros_topic_setter.brake(False)
        """

        buttons = joystick.get_numbuttons()
        textPrint.pprint(screen, "Number of buttons: {}".format(buttons) )
        textPrint.indent()

        for i in range( buttons ):
            button = joystick.get_button( i )
            textPrint.pprint(screen, "Button {:>2} value: {}".format(i,button) )
        textPrint.unindent()

        # Hat switch. All or nothing for direction, not like joysticks.
        # Value comes back in an array.
        hats = joystick.get_numhats()
        textPrint.pprint(screen, "Number of hats: {}".format(hats) )
        textPrint.indent()

        for i in range( hats ):
            hat = joystick.get_hat( i )
            textPrint.pprint(screen, "Hat {} value: {}".format(i, str(hat)) )
        textPrint.unindent()

        textPrint.unindent()


    # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT

    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()

    # Limit to 20 frames per second
    clock.tick(20)

# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit()
