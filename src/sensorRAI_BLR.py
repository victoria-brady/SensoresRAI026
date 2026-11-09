import asyncio
from bleak import BleakScanner, BleakClient

# These are the name and UUIDs configured in your Arduino code
DEVICE_NAME = "ArduinoGigaR1"
CHARACTERISTIC_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"

# This function is triggered automatically every time the Arduino sends new data
def notification_handler(sender, data):
    # Decode the received bytes into text
    received_text = data.decode('utf-8')

    # Split the text using the comma as a separator
    try:
        vl1, vl2, us = received_text.split(',')
        print(f"📥 Data received -> Optical 1: {vl1} mm |  Ultrasonic: {us} mm | Optical 2: {vl2} mm")
    except ValueError:
        print(f"Unexpected format received: {received_text}")

async def main():
    print("Scanning for Bluetooth devices...")
    devices = await BleakScanner.discover()

    # Look for our Arduino by name
    arduino_device = None
    for d in devices:
        if d.name == DEVICE_NAME:
            arduino_device = d
            break

    if not arduino_device:
        print(f"No device named {DEVICE_NAME} was found")
        return

    print(f"Found {DEVICE_NAME} at address {arduino_device.address}")
    print("Connecting...")

    # Connect and subscribe to notifications
    async with BleakClient(arduino_device.address) as client:
        print("Connected! Waiting for data (press Ctrl+C to exit)...")
        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)

        # Keep the program running indefinitely
        while True:
            await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())
