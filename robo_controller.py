import os
import sys

# Ensure WEBOTS_HOME is set so we can import the Webots controller modules
if "WEBOTS_HOME" not in os.environ:
    os.environ["WEBOTS_HOME"] = "/snap/webots/current/usr/share/webots"

# Append the Webots Python controller path
webots_path = os.path.join(os.environ["WEBOTS_HOME"], "lib", "controller", "python")
sys.path.append(webots_path)

from controller import Robot

def main():
    # Attempt to initialize the robot
    # The Robot() call will automatically connect to Webots using the WEBOTS_CONTROLLER_URL
    robot = Robot()

    # Get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    
    print(f"[{robot.getName()}] Successfully connected to Webots as an extern controller!")

    # Example: you could initialize motors here if the robot has wheels.
    # left_motor = robot.getDevice('left wheel motor')
    # right_motor = robot.getDevice('right wheel motor')
    # if left_motor:
    #     left_motor.setPosition(float('inf'))
    #     left_motor.setVelocity(0.0)

    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Read the sensors:
        # val = ds.getValue()

        # Process sensor data here.

        # Enter here functions to send actuator commands, like:
        # left_motor.setVelocity(1.0)
        # right_motor.setVelocity(1.0)
        pass

    print(f"[{robot.getName()}] Disconnected or simulation stopped.")

if __name__ == "__main__":
    main()
