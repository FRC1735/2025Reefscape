import usb.core
import usb.util
import usb.backend.libusb1
from usbmonitor import USBMonitor
from networktables import NetworkTables
import threading
import time
import logging

# XK-80 Vendor and Product ID
VENDOR_ID = 1523
PRODUCT_ID = 1089

# NetworkTables stuff
SERVER = '10.17.35.2'
KEYBOARD = 0
KEYS = 80

connected = False
# Set up logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger()

# Set up NetworkTables connection
def initialize_networktables():
    try:
        NetworkTables.initialize(server=SERVER)
    except Exception as e:
        logger.error(f"Failed to initialize NetworkTables with server {SERVER}: {e}")

keyboard_status_table = NetworkTables.getTable('AdvantageKit/DriverStation/Keyboard'+str(KEYBOARD))

def on_disconnect(device_id, device_info):
    global connected
    keyboard_status_table.putBoolean('isConnected', False)
    connected = False
    logger.info(f"Device disconnected: {device_id}")

def on_connect(device_id, device_info):
    global connected
    keyboard_status_table.putBoolean('isConnected', True)
    connected = True
    logger.info(f"Device connected: {device_id}")

monitor = USBMonitor()
monitor.start_monitoring(on_connect=on_connect, on_disconnect=on_disconnect, check_every_seconds=0.02)

def send_data(data):
    keyboard_status_table.putBoolean('isConnected', True)
    for key in data:
        keyboard_status_table.putBoolean(str(key), True)

def set_up_NT():
    for i in range(KEYS):
        keyboard_status_table.putBoolean(str(i), False)

set_up_NT()

def reconnect_device():
    global device, endpoint, connected
    # Try to find the device again
    device = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID, backend=backend)
    if device is None:
        logger.error("Device not found!")
        connected = False
        return None
    else:
        logger.info("Device reconnected!")
        device.set_configuration()
        cfg = device.get_active_configuration()
        interface = cfg[(0, 0)]
        endpoint = usb.util.find_descriptor(
            interface,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
        )
        connected = True
        return endpoint

def check_networktables_connection():
    try:
        # Just make a simple operation to verify connectivity
        NetworkTables.getTable('AdvantageKit/DriverStation/Keyboard' + str(KEYBOARD)).getBoolean('isConnected', False)
    except Exception as e:
        logger.error(f"NetworkTables connection lost: {e}. Reconnecting...")
        try:
            NetworkTables.initialize(server=SERVER)
        except Exception as e:
            logger.error(f"Failed to reconnect to NetworkTables: {e}")

# USB read function in a separate thread
def read_input():
    global connected, device, endpoint
    backend = usb.backend.libusb1.get_backend()

    while True:
        if not connected:
            logger.info("Waiting for device connection...")
            time.sleep(1)
            continue

        try:
            if device is None:
                logger.error("Device is not connected, attempting to reconnect...")
                endpoint = reconnect_device()
                if endpoint is None:
                    logger.error("Reconnection failed. Retrying in 2 seconds...")
                    time.sleep(2)
                    continue

            try:
                data = device.read(endpoint.bEndpointAddress, endpoint.wMaxPacketSize, timeout=5000)
                if data:
                    logger.debug(f"Raw Data: {list(data)}")
                    key_states = data[2:12]  # 10 bytes, representing 10 columns
                    pressed_keys = []

                    for col_index, byte in enumerate(key_states):
                        for row_index in range(8):
                            if (byte >> row_index) & 1:
                                key_number = (col_index) * 8 + row_index
                                pressed_keys.append(key_number)

                    set_up_NT()
                    if pressed_keys:
                        logger.info(f"Keys Pressed: {pressed_keys}")
                        send_data(pressed_keys)
                    else:
                        logger.info("No keys pressed.")
                else:
                    logger.info("No data received.")
                    keyboard_status_table.putBoolean('isConnected', False)

            except usb.core.USBTimeoutError:
                logger.warning("Timeout error occurred.")
            except usb.core.USBError as e:
                if e.errno == 5:  # Input/Output Error = Disconnected device
                    logger.error("Device disconnected!")
                    keyboard_status_table.putBoolean('isConnected', False)
                    connected = False
                    break

        except Exception as e:
            logger.error(f"Error during USB read: {e}")
            connected = False
            break

        check_networktables_connection()

def main():
    global connected, device, endpoint

    # Find the XK-80 device
    device = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID, backend=usb.backend.libusb1.get_backend())

    if device is None:
        logger.error("XK-80 not found! Check connections.")
        exit(1)

    logger.info("XK-80 found and ready!")

    # Detach kernel driver if necessary (Linux only)
    # if device.is_kernel_driver_active(0):
    #     device.detach_kernel_driver(0)

    # Set device configuration
    device.set_configuration()

    # Find the HID endpoint for reading input
    cfg = device.get_active_configuration()
    interface = cfg[(0, 0)]
    endpoint = usb.util.find_descriptor(
        interface,
        custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
    )

    if endpoint is None:
        logger.error("No IN endpoint found on XK-80")
        keyboard_status_table.putBoolean('isConnected', False)
        exit(1)

    logger.info("Listening for XK-80 key presses...")

    # Start the input reading thread
    read_thread = threading.Thread(target=read_input)
    read_thread.start()

    # Main loop
    try:
        while True:
            # Main thread can handle other tasks here
            time.sleep(1)

    except KeyboardInterrupt:
        logger.info("Stopping script.")
        keyboard_status_table.putBoolean('isConnected', False)

    finally:
        logger.info("Cleaning up...")
        monitor.stop_monitoring()
        keyboard_status_table.putBoolean('isConnected', False)
        NetworkTables.stopClient()

        try:
            usb.util.release_interface(device, 0)
            device.attach_kernel_driver(0)  # Only needed on Linux
        except:
            pass  # Ignore errors if device is already gone

if __name__ == "__main__":
    main()
