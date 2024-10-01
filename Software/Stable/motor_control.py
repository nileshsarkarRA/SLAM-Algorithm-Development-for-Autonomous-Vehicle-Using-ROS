import time
from gpiozero import Motor, RotaryEncoder

# Define motor connections (replace GPIO pins with your actual wiring)
motor_a = Motor(forward=17, backward=18)  # Motor A control pins
motor_b = Motor(forward=22, backward=23)  # Motor B control pins

# Define encoder connections (replace GPIO pins with your actual wiring)
encoder_a = RotaryEncoder(24, 25)  # Encoder A pins
encoder_b = RotaryEncoder(26, 27)  # Encoder B pins (if needed)

# Function to move motors forward at a specified speed
def move_forward(speed=1):
    if 0 <= speed <= 1:
        motor_a.forward(speed)
        motor_b.forward(speed)
    else:
        print("Speed must be between 0 and 1.")

# Function to move motors backward at a specified speed
def move_backward(speed=1):
    if 0 <= speed <= 1:
        motor_a.backward(speed)
        motor_b.backward(speed)
    else:
        print("Speed must be between 0 and 1.")

# Function to stop motors
def stop_motors():
    motor_a.stop()
    motor_b.stop()

# Main loop
try:
    print("Moving forward...")
    move_forward(0.5)  # Adjust speed as necessary

    # Run forward for a certain duration (e.g., 5 seconds)
    time.sleep(5)

    print("Moving backward...")
    move_backward(0.5)  # Adjust speed as necessary

    # Run backward for a certain duration (e.g., 5 seconds)
    time.sleep(5)

    # Stop motors
    stop_motors()

    # Read encoder values while stopped
    print("Encoder readings while stopped:")
    print(f"Encoder A position: {encoder_a.value}")  # Read encoder A value
    print(f"Encoder B position: {encoder_b.value}")  # Read encoder B value (if used)

except KeyboardInterrupt:
    print("Stopping motors...")
    stop_motors()

finally:
    # Cleanup code (if necessary)
    print("Cleaning up GPIO...")

   
