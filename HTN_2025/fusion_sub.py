import paho.mqtt.client as mqtt
import json
from utils.utils_pwm import MotorBase

def on_message(client, userdata, msg):
    # Decode the message
    data = json.loads(msg.payload.decode())
    
    # Print the sensor data
    roll = data['roll']
    pitch = data['pitch']
    yaw = data['yaw']
    if pitch > 0:
        motor.set_left_speed(10.0)
    elif pitch == 0:
        motor.set_left_speed(7.6)
    else:
        motor.set_left_speed(5.0)
    
    print(f"roll: {roll}  pitch: {pitch}  yaw: {yaw}")
    print("-" * 40)

def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker!")
    client.subscribe("sensors/mpu6050")

# Create client and connect
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

print("Connecting to MQTT broker...")
client.connect("broker.hivemq.com", 1883, 60)

print("Listening for MPU6050 data...")
print("Press Ctrl+C to stop")

motor = MotorBase()

try:
    client.loop_forever()
except KeyboardInterrupt:
    print("\nStopping...")
    client.disconnect()
