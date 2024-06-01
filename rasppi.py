import time
from bluepy.btle import Peripheral, BTLEException, UUID
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import tkinter as tk
from threading import Thread, Event

# GPIO setup for servo motors
servo_pin_1 = 18  # GPIO 18 (Physical pin 12) for servo motor 1
servo_pin_2 = 23  # GPIO 23 (Physical pin 16) for servo motor 2
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)  # Disable GPIO warnings
GPIO.setup(servo_pin_1, GPIO.OUT)
GPIO.setup(servo_pin_2, GPIO.OUT)

# Set PWM frequency to 50 Hz
pwm1 = GPIO.PWM(servo_pin_1, 50)
pwm2 = GPIO.PWM(servo_pin_2, 50)
pwm1.start(0)
pwm2.start(0)

# Global variables to control the watering system and selected greenhouse
pause_watering = False
selected_greenhouse = "Greenhouse 1"  # Default to the single greenhouse
bluetooth_failures = 0  # Count Bluetooth failures

# Dictionary to store greenhouses and their Bluetooth addresses
greenhouses = {
    "Greenhouse 1": "78:21:84:ad:32:c2"
    # Future greenhouses can be added here
}

# MQTT Broker settings
mqtt_broker = "broker.emqx.io"
mqtt_port = 1883
mqtt_topic_soil = "greenhouse/soil"
mqtt_topic_water = "greenhouse/water"
mqtt_client = mqtt.Client(client_id="Greenhouse_Client", protocol=mqtt.MQTTv311)

# Heartbeat events for watchdog
main_bluetooth_heartbeat = Event()
main_gui_heartbeat = Event()

def set_angle(pwm, angle):
    duty = 2 + (angle / 18)
    pwm.ChangeDutyCycle(duty)
    time.sleep(1)
    pwm.ChangeDutyCycle(0)

def move_servo(pwm, start_angle, end_angle):
    set_angle(pwm, end_angle)
    time.sleep(1)
    set_angle(pwm, start_angle)

def connect_to_device(address, addr_type):
    global bluetooth_failures
    while True:
        try:
            print(f"Connecting to {address} with address type {addr_type}")
            peripheral = Peripheral(address, addr_type)
            print(f"Successfully connected to {address} with address type {addr_type}")
            bluetooth_failures = 0  # Reset failure count on success
            return peripheral
        except BTLEException as e:
            print(f"Connection failed: {e}. Retrying in 5 seconds...")
            bluetooth_failures += 1
            if bluetooth_failures >= 10:
                print("Bluetooth failed 10 times. Switching to MQTT.")
                return None
            time.sleep(5)

def on_mqtt_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        mqtt_client.subscribe(mqtt_topic_soil)
        mqtt_client.subscribe(mqtt_topic_water)
    else:
        print("Failed to connect, return code %d\n", rc)

def on_mqtt_message(client, userdata, msg):
    global pause_watering

    if pause_watering:
        return

    topic = msg.topic
    payload = msg.payload.decode()
    value = int(payload)

    if topic == mqtt_topic_soil:
        if value < 500:
            print("Soil moisture below threshold, moving servo 2 to 45 degrees")
            set_angle(pwm2, 45)
        else:
            print("Soil moisture above threshold, moving servo 2 to 0 degrees")
            set_angle(pwm2, 0)
    elif topic == mqtt_topic_water:
        if value > 500:
            print("Water sensor above threshold, moving servo 1 to 180 degrees and back to 0 degrees")
            move_servo(pwm1, 0, 180)
        else:
            print("Water sensor below threshold, ensuring servo 1 is at 0 degrees")
            set_angle(pwm1, 0)

def main_bluetooth():
    global pause_watering
    global selected_greenhouse
    global bluetooth_failures

    arduino_address = greenhouses[selected_greenhouse]
    addr_type = "public"  # Trying with "public" address type first
    peripheral = connect_to_device(arduino_address, addr_type)

    if not peripheral:
        # If Bluetooth fails, switch to MQTT
        mqtt_client.on_connect = on_mqtt_connect
        mqtt_client.on_message = on_mqtt_message
        mqtt_client.connect(mqtt_broker, mqtt_port, 60)
        mqtt_client.loop_start()
        use_mqtt = True
    else:
        use_mqtt = False

    service_uuid = UUID("180F")
    soil_char_uuid = UUID("2A19")
    water_char_uuid = UUID("2A6E")

    if not use_mqtt:
        service = peripheral.getServiceByUUID(service_uuid)
        soil_char = service.getCharacteristics(soil_char_uuid)[0]
        water_char = service.getCharacteristics(water_char_uuid)[0]

    previous_soil_value = None
    previous_water_value = None

    try:
        while True:
            try:
                main_bluetooth_heartbeat.set()  # Send heartbeat signal

                if not pause_watering:
                    if use_mqtt:
                        # MQTT messages are handled by the on_mqtt_message callback
                        pass
                    else:
                        soil_value = soil_char.read()
                        water_value = water_char.read()

                        if soil_value != previous_soil_value or water_value != previous_water_value:
                            print(f"Received soil moisture value: {soil_value}, water sensor value: {water_value}")

                            if soil_value == b'\x01':
                                print("Soil moisture below threshold, moving servo 2 to 45 degrees")
                                set_angle(pwm2, 45)
                            else:
                                print("Soil moisture above threshold, moving servo 2 to 0 degrees")
                                set_angle(pwm2, 0)

                            if water_value == b'\x01':
                                print("Water sensor above threshold, moving servo 1 to 180 degrees and back to 0 degrees")
                                move_servo(pwm1, 0, 180)
                            else:
                                print("Water sensor below threshold, ensuring servo 1 is at 0 degrees")
                                set_angle(pwm1, 0)

                        previous_soil_value = soil_value
                        previous_water_value = water_value

                time.sleep(5)  # Adjust the sleep time as needed
            except BTLEException as e:
                print(f'Error during communication: {e}')
                if not use_mqtt:
                    peripheral.disconnect()
                    peripheral = connect_to_device(arduino_address, addr_type)
                    if not peripheral:
                        # Switch to MQTT if Bluetooth fails
                        mqtt_client.on_connect = on_mqtt_connect
                        mqtt_client.on_message = on_mqtt_message
                        mqtt_client.connect(mqtt_broker, mqtt_port, 60)
                        mqtt_client.loop_start()
                        use_mqtt = True
                    else:
                        service = peripheral.getServiceByUUID(service_uuid)
                        soil_char = service.getCharacteristics(soil_char_uuid)[0]
                        water_char = service.getCharacteristics(soil_char_uuid)[0]
    except KeyboardInterrupt:
        GPIO.cleanup()
        if not use_mqtt:
            peripheral.disconnect()
        else:
            mqtt_client.loop_stop()
        print("Disconnected")

def toggle_watering_system():
    global pause_watering
    pause_watering = not pause_watering
    pause_status.set(pause_watering)

def select_greenhouse():
    global selected_greenhouse
    selected_greenhouse = greenhouse_var.get()

def start_bluetooth_thread():
    bt_thread = Thread(target=main_bluetooth)
    bt_thread.daemon = True
    bt_thread.start()

def create_gui():
    global pause_status
    global greenhouse_var

    root = tk.Tk()
    root.title("Greenhouse Control System")

    tk.Label(root, text="Select Greenhouse:").pack()

    greenhouse_var = tk.StringVar()
    greenhouse_var.set("Greenhouse 1")  # Default value

    for greenhouse in greenhouses.keys():
        tk.Radiobutton(root, text=greenhouse, variable=greenhouse_var, value=greenhouse, command=select_greenhouse).pack()

    pause_status = tk.BooleanVar()

    toggle_button = tk.Button(root, text="Pause/Resume Watering System", command=toggle_watering_system)
    toggle_button.pack()

    start_bluetooth_thread()

    root.mainloop()

if __name__ == "__main__":
    create_gui()
