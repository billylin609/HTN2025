"""
Pure Python Madgwick Filter Implementation
No external dependencies (no numpy, no scipy)
For MPU6050 sensor fusion to estimate roll, pitch, yaw angles
"""

import math
import time
import json
import paho.mqtt.client as mqtt

class MadgwickFilter:
    def __init__(self, sample_frequency=100.0, beta=0.1):
        """
        Initialize Madgwick filter
        
        Args:
            sample_frequency: Sample rate in Hz
            beta: Algorithm gain (lower = more stable, higher = faster convergence)
        """
        self.sample_freq = sample_frequency
        self.beta = beta
        
        # Quaternion (w, x, y, z)
        self.q0 = 1.0  # w
        self.q1 = 0.0  # x  
        self.q2 = 0.0  # y
        self.q3 = 0.0  # z
    
    def fast_inverse_sqrt(self, x):
        """Fast inverse square root approximation"""
        if x <= 0:
            return 0.0
        return 1.0 / math.sqrt(x)
    
    def update_imu(self, gx, gy, gz, ax, ay, az):
        """
        Update filter with gyroscope and accelerometer data
        
        Args:
            gx, gy, gz: Gyroscope data in rad/s
            ax, ay, az: Accelerometer data in g
        """
        # Convert gyroscope degrees/sec to radians/sec if needed
        # gx, gy, gz should already be in rad/s
        
        # Short name local variable for readability
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        
        # Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0 * q0
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _4q0 = 4.0 * q0
        _4q1 = 4.0 * q1
        _4q2 = 4.0 * q2
        _8q1 = 8.0 * q1
        _8q2 = 8.0 * q2
        q0q0 = q0 * q0
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        
        # Normalize accelerometer measurement
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0.0:
            return  # Handle NaN
        
        norm = 1.0 / norm
        ax *= norm
        ay *= norm
        az *= norm
        
        # Gradient descent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
        s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
        s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay
        
        # Normalize step magnitude
        norm = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
        if norm != 0.0:
            norm = 1.0 / norm
            s0 *= norm
            s1 *= norm
            s2 *= norm
            s3 *= norm
        
        # Apply feedback step
        qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz) - self.beta * s0
        qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy) - self.beta * s1
        qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx) - self.beta * s2
        qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx) - self.beta * s3
        
        # Integrate rate of change of quaternion
        dt = 1.0 / self.sample_freq
        q0 += qDot1 * dt
        q1 += qDot2 * dt
        q2 += qDot3 * dt
        q3 += qDot4 * dt
        
        # Normalize quaternion
        norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        if norm != 0.0:
            norm = 1.0 / norm
            self.q0 = q0 * norm
            self.q1 = q1 * norm
            self.q2 = q2 * norm
            self.q3 = q3 * norm
    
    def get_quaternion(self):
        """Get current quaternion (w, x, y, z)"""
        return self.q0, self.q1, self.q2, self.q3
    
    def get_euler_angles(self):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw) in degrees
        
        Returns:
            tuple: (roll, pitch, yaw) in degrees
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (self.q0 * self.q1 + self.q2 * self.q3)
        cosr_cosp = 1.0 - 2.0 * (self.q1 * self.q1 + self.q2 * self.q2)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2.0 * (self.q0 * self.q2 - self.q3 * self.q1)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (self.q0 * self.q3 + self.q1 * self.q2)
        cosy_cosp = 1.0 - 2.0 * (self.q2 * self.q2 + self.q3 * self.q3)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Convert to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        return roll_deg, pitch_deg, yaw_deg



# Example usage with MPU6050
if __name__ == "__main__":
    from mpu6050 import mpu6050
    
    # Simple configuration
    BROKER = "broker.hivemq.com"
    TOPIC = "sensors/mpu6050"
    
    # Initialize sensor and filter
    sensor = mpu6050(0x68)
    madgwick = MadgwickFilter(sample_frequency=100.0, beta=0.1)
    
    # Create MQTT client
    client = mqtt.Client()
    
    print("Starting sensor fusion...")
    print("Press Ctrl+C to exit")
    
    # Connect to broker
    print("Connecting to MQTT broker...")
    client.connect(BROKER, 1883, 60)
    
    try:
        while True:
            start_time = time.time()
            
            # Read sensor data
            accel_data = sensor.get_accel_data()
            gyro_data = sensor.get_gyro_data()
            
            # Convert units
            ax, ay, az = accel_data['x'], accel_data['y'], accel_data['z']
            gx_deg, gy_deg, gz_deg = gyro_data['x'], gyro_data['y'], gyro_data['z']
            
            # Convert gyro to rad/s for Madgwick
            gx_rad = math.radians(gx_deg)
            gy_rad = math.radians(gy_deg) 
            gz_rad = math.radians(gz_deg)
            
            # Update filters
            madgwick.update_imu(gx_rad, gy_rad, gz_rad, ax, ay, az)
            
            # Get results
            mad_roll, mad_pitch, mad_yaw = madgwick.get_euler_angles()
            
            DEADZONE_SENSITIVITY = 10
            
            deadzone = lambda x: x if abs(x) > DEADZONE_SENSITIVITY else 0.0
            mad_pitch = deadzone(mad_pitch)
            mad_roll = deadzone(mad_roll)
            
            # Display results
            print(f"Madgwick - Roll: {mad_roll:6.1f}°  Pitch: {mad_pitch:6.1f}°  Yaw: {mad_yaw:6.1f}°")
            print("-" * 60)
            
            message = {
                'roll': round(mad_roll, 2),
                'pitch': round(mad_pitch, 2),
                'yaw': round(mad_yaw, 2),
            }
            
            # Publish to MQTT
            client.publish(TOPIC, json.dumps(message))
            
            # Maintain sample rate
            elapsed = time.time() - start_time
            sleep_time = max(0, 0.01 - elapsed)  # 100Hz target
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("\nStopping sensor fusion...")
        client.disconnect()