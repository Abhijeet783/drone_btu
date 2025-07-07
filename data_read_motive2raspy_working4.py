import sys
import time
import json
import threading
from NatNetClient import NatNetClient
import paho.mqtt.client as mqtt

# --- Configuration ---
broker_address = "192.168.6.24"  # Update if needed
port = 1883
topic = "motive/data"

# --- Global ---
is_looping = True
mqtt_client_instance = None

# --- MQTT Callbacks ---
def on_connect(client, userdata, flags, rc):
    sys.stdout = original_stdout
    if rc == 0:
        print("Connected to MQTT Broker.")
    else:
        print(f"MQTT connection failed with code {rc}")
    sys.stdout = NullWriter()

# --- NatNet Callback ---
def receive_rigid_body_frame(id, position, rotation):
    global mqtt_client_instance

    motive_data = {
        "id": id,
        "position": {  # NEW: Wrap position data
            "x": position[0],
            "y": position[1],
            "z": position[2]
        },
        "rotation": {  # NEW: Wrap rotation data
            "x": rotation[0],
            "y": rotation[1],
            "z": rotation[2],
            "w": rotation[3]
        }
    }

    payload = json.dumps(motive_data)

    if mqtt_client_instance and mqtt_client_instance.is_connected():
        try:
            mqtt_client_instance.publish(topic, payload)
        except Exception as e:
            sys.stdout = original_stdout
            print(f"Error publishing MQTT message: {e}")
            sys.stdout = NullWriter()
    else:
        sys.stdout = original_stdout
        print("Warning: MQTT client not connected.")
        sys.stdout = NullWriter()

def receive_new_frame(data_dict):
    pass

# --- Suppress Output ---
class NullWriter:
    def write(self, s): pass
    def flush(self): pass

# --- Main ---
if __name__ == "__main__":
    original_stdout = sys.stdout
    original_stderr = sys.stderr
    sys.stdout = NullWriter()
    sys.stderr = NullWriter()

    # Setup MQTT
    mqtt_client_instance = mqtt.Client()
    mqtt_client_instance.on_connect = on_connect

    try:
        mqtt_client_instance.connect(broker_address, port, 60)
        mqtt_client_instance.loop_start()
    except Exception as e:
        sys.stdout = original_stdout
        print(f"ERROR: MQTT connect failed. {e}")
        sys.exit(1)
    sys.stdout = NullWriter()

    # Setup NatNet
    client = NatNetClient()
    client.rigid_body_listener = receive_rigid_body_frame
    client.new_frame_listener = receive_new_frame
    client.set_client_address("127.0.0.1")
    client.set_server_address("127.0.0.1")
    client.set_use_multicast(False)  # Unicast mode
    client.set_print_level(0)

    is_running = client.run('d')

    sys.stdout = original_stdout
    sys.stderr = original_stderr

    if not is_running:
        print("ERROR: NatNet client failed to start.")
        mqtt_client_instance.loop_stop()
        mqtt_client_instance.disconnect()
        client.shutdown()
        sys.exit(1)

    time.sleep(1)
    if not client.connected():
        print("ERROR: NatNet connection failed. Check firewall or IP settings.")
        mqtt_client_instance.loop_stop()
        mqtt_client_instance.disconnect()
        client.shutdown()
        sys.exit(2)

    print("\n--- NatNet Unicast to MQTT Publisher ---")
    print(f"Publishing to topic '{topic}' on {broker_address}:{port}")
    print("Press Ctrl+C to stop.\n")

    sys.stdout = NullWriter()
    sys.stderr = NullWriter()

    try:
        while is_looping:
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout = original_stdout
        sys.stderr = original_stderr
        print("\nShutting down...")

        if client:
            client.shutdown()
            print("NatNet client shut down.")
        if mqtt_client_instance:
            mqtt_client_instance.loop_stop()
            mqtt_client_instance.disconnect()
            print("MQTT client disconnected.")
        print("Done.")
        sys.exit(0)
