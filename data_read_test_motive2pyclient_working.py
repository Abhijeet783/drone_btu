#=============================================================================
# OptiTrack NatNet Python Client - Minimal Rigid Body Position Stream
# Prints position of any rigid body at 1-second intervals, stops on Ctrl+C.
#=============================================================================

import sys
import time

from NatNetClient import NatNetClient

# --- Global Variables ---
is_looping = True # Flag for graceful exit
last_print_time_per_rb = {} # Tracks last print time for each rigid body
PRINT_INTERVAL = 0.0166 # Print interval in seconds

# --- Callbacks ---
def receive_rigid_body_frame(id, position, rotation):
    global last_print_time_per_rb, PRINT_INTERVAL, original_stdout, NullWriter
    current_time = time.time()

    if (current_time - last_print_time_per_rb.get(id, 0.0)) >= PRINT_INTERVAL:
        # Temporarily restore stdout to print our message
        sys.stdout = original_stdout
        print(f"RB ID: {id}, Pos: x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}")
        sys.stdout = NullWriter() # Redirect stdout back
        last_print_time_per_rb[id] = current_time

def receive_new_frame(data_dict):
    pass # Not used for this minimal script



# --- Output Suppression Class ---
class NullWriter:
    def write(self, s):
        pass
    def flush(self):
        pass

# --- Main Execution ---
if __name__ == "__main__":
    # Store original stdout/stderr to restore later
    original_stdout = sys.stdout
    original_stderr = sys.stderr

    # Register Ctrl+C handler


    # Suppress all library output by default
    sys.stdout = NullWriter()
    sys.stderr = NullWriter()

    # Create and configure NatNet client
    client = NatNetClient()
    client.rigid_body_listener = receive_rigid_body_frame
    client.new_frame_listener = receive_new_frame
    client.set_client_address("127.0.0.1")
    client.set_server_address("127.0.0.1")
    client.set_use_multicast(True)
    # Set print level to 0 to suppress internal NatNetClient messages
    client.set_print_level(0)

    # Start client (will run in a separate thread)
    is_running = client.run('d')

    # Restore stdout/stderr for our own messages
    sys.stdout = original_stdout
    sys.stderr = original_stderr

    # Initial connection check and status
    if not is_running:
        print("ERROR: Failed to start NatNet client. Is Motive running and streaming?")
        client.uninitialize()
        sys.exit(1)

    time.sleep(1) # Give time for connection to establish
    if not client.connected():
        print("ERROR: Client failed to connect to Motive. Check firewall or IP settings.")
        client.shutdown()
        sys.exit(2)

    print("\n--- NatNet Client: Rigid Body Position Stream ---")
    print("Receiving data. Press Ctrl+C to stop.")

    # Re-suppress output during main loop
    sys.stdout = NullWriter()
    sys.stderr = NullWriter()

    # Keep script alive while client streams data
    try:
        while is_looping:
            time.sleep(0.01) # Small delay to avoid busy-waiting
    except KeyboardInterrupt:
        pass # Handled by signal_handler

    finally:
        # Ensure stdout/stderr are restored for final messages
        sys.stdout = original_stdout
        sys.stderr = original_stderr
        print("\nShutting down client.")
        client.shutdown()
        sys.exit(0)