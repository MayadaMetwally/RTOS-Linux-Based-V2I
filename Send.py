import serial
from time import sleep
import json
import paho.mqtt.client as mqtt_client
import threading

# MQTT Broker details
host = "10.145.12.10"  # Your local broker's IP address
port = 1883

# Client details for pub
client_id_pub = "pub"
username_pub = "pub"
password_pub = "pub"

# Topic for pub
topic_pub = "test/topic"

# UART communication setup
ser = serial.Serial("/dev/ttyS0", 9600)

def parse_nmea(nmea_message):
    start_idx = nmea_message.find('$')
    if start_idx == -1:
        raise ValueError('No valid NMEA message found')

    nmea_message = nmea_message[start_idx:]
    parts = nmea_message.split(',')

    if parts[0] == '$GPRMC':
        if len(parts) >= 10 and '*' in parts[-1]:  # Ensure there's enough parts and a checksum present
            lat = parts[3]
            lat_dir = parts[4]
            lon = parts[5]
            lon_dir = parts[6]

            latitude = float(lat[:2]) + float(lat[2:]) / 60.0
            if lat_dir == 'S':
                latitude = -latitude

            longitude = float(lon[:3]) + float(lon[3:]) / 60.0
            if lon_dir == 'W':
                longitude = -longitude

            return latitude, longitude
        else:
            raise ValueError('No valid GPS signal')
    else:
        raise ValueError('Unsupported NMEA message type')

def send_coordinates(latitude, longitude):
    message = json.dumps({
        "latitude": latitude,
        "longitude": longitude
    })

    try:
        client = mqtt_client.Client(client_id_pub)
        client.username_pw_set(username_pub, password_pub)
        client.connect(host, port, 60)
        client.loop_start()
        client.publish(topic_pub, payload=message, qos=1)
        client.loop_stop()
        client.disconnect()
        print(f"Coordinates sent to MQTT broker: Latitude={latitude}, Longitude={longitude}")
    except Exception as e:
        print(f"Failed to send coordinates: {e}")

def read_serial_data():
    buffer = ""
    while True:
        if ser.inWaiting() > 0:
            received_data = ser.read(ser.inWaiting()).decode('utf-8')
            buffer += received_data

            while '$' in buffer and '*' in buffer:
                start_idx = buffer.find('$')
                end_idx = buffer.find('*', start_idx)
                if end_idx == -1:
                    break

                end_idx = end_idx + 3

                nmea_message = buffer[start_idx:end_idx]
                buffer = buffer[end_idx + 1:]

                try:
                    latitude, longitude = parse_nmea(nmea_message)
                    if latitude is not None and longitude is not None:
                        print(f"Parsed coordinates: Latitude={latitude}, Longitude={longitude}")
                        send_coordinates(latitude, longitude)
                except ValueError as e:
                    print(f"Parsing error: {e}")

        sleep(0.1)

# Start the serial reading thread
serial_thread = threading.Thread(target=read_serial_data)
serial_thread.start()

# Keep the main thread alive
while True:
    sleep(1)
