import pygame
import serial
import sys
import math

# Set the serial port and baud rate
try:
    ser = serial.Serial('COM6')
except Exception:
    print("Cannot open serial port")

# Initialize pygame
pygame.init()

# Set the width and height of the screen
width, height = 640, 480
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption('lobster robot arm')
#gameIcon = pygame.image.load('dalton.jpg')
#pygame.display.set_icon(gameIcon)

# Define colors
black = (0, 0, 0)
white = (255, 255, 255)

# Create a clock to control the frame rate
clock = pygame.time.Clock()

# Main game loop
while True:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    # Read the joint angles from the serial port
    line = ser.readline().decode()

    if line.startswith("SOLUTION"):
        # Clear the screen
        screen.fill(white)

        # Split the line into a list of values
        values = line.split(',')
        # Define list of joint angles and append the values to the list
        joint_angles = [float(values[i].split(':')[1]) for i in range(4, len(values))]
        joint_angles.append(0.0)
        joint_lengths = [147, 130, 58, 58]
        print(joint_angles)

        # Define the coordinates of the robot arm components
        j1_a, j2_a, j3_a, ee_a, ee_r = joint_angles
        j1_l, j2_l, j3_l, ee_l = joint_lengths

        base_x1 = 200
        base_y = 380
        base_x2 = base_x1 + 50
        j1_x = base_x1 + 25
        j1_y = base_y
        j2_x = j1_x + j1_l * math.cos(j1_a * (math.pi / 180))
        j2_y = j1_y - j1_l * math.sin(j1_a * (math.pi / 180))
        j3_x = j2_x + j2_l * math.sin((j1_a + j2_a) * (math.pi / 180))
        j3_y = j2_y + j2_l * math.cos((j1_a + j2_a) * (math.pi / 180))
        j4_x = j3_x - j3_l * math.cos((j1_a + j2_a + j3_a) * (math.pi / 180))
        j4_y = j3_y + j3_l * math.sin((j1_a + j2_a + j3_a) * (math.pi / 180))
        ee_x = j4_x + ee_l * math.sin((j1_a + j2_a + j3_a + ee_a) * (math.pi / 180))
        ee_y = j4_y - ee_l * math.cos((j1_a + j2_a + j3_a + ee_a) * (math.pi / 180))

        # Draw the base of the robot arm using the draw.lines() method
        pygame.draw.lines(screen, black, False, [(base_x1, base_y), (base_x2, base_y)], 5)

        # Draw the first joint using the draw.circle() and draw.lines() method
        pygame.draw.circle(screen, black, (j1_x, j1_y), 5)
        pygame.draw.lines(screen, black, False, [(j1_x, j1_y), (j2_x, j2_y)], 5)

        # Draw the second joint using the draw.circle() and draw.lines() method
        pygame.draw.circle(screen, black, (j2_x, j2_y), 5)
        pygame.draw.lines(screen, black, False, [(j2_x, j2_y), (j3_x, j3_y)], 5)

        # Draw the third joint using the draw.circle() and draw.lines() method
        pygame.draw.circle(screen, black, (j3_x, j3_y), 5)
        pygame.draw.lines(screen, black, False, [(j3_x, j3_y), (j4_x, j4_y)], 5)

        # Draw the end effector using the draw.circle(), draw.rect(), and draw.lines() method
        #pygame.draw.circle(screen, black, (j4_x, j4_y), 5)
        #pygame.draw.lines(screen, black, False, [(j4_x, j4_y), (ee_x, ee_y)], 15)

        # Update the screen
        pygame.display.flip()

        # Limit the frame rate to 60 FPS
        clock.tick(60)