import matplotlib.pyplot as plt
import numpy as np

def simulate_pwm(angle, period_ms=20):
    # Clamp angle between -90 and 90
    angle = max(angle, -90)
    angle = min(angle, 90)

    # Convert angle to a range between 0-180 degrees (just for testing purposes)
    transformed_angle = angle + 90
    
    # Calculate pulse width in microseconds
    pulse_ms = (transformed_angle * 1800) / 180 + 600
    pulse_ms /= 1000 # Convert to miliseconds
    
    # Create time array
    t = np.linspace(0, period_ms, 1000)
    
    # Create PWM waveform
    pwm = np.where(t % period_ms < pulse_ms, 1, 0)
    
    plt.plot(t, pwm)
    plt.title(f"PWM waveform for angle = {angle}° (Pulse = {pulse_ms:.2f} ms)")
    plt.xlabel("Time (ms)")
    plt.ylabel("PWM Level")
    plt.ylim(-0.2, 1.2)
    plt.grid(True)
    plt.show()

simulate_pwm(-90)
simulate_pwm(0)
simulate_pwm(90)
