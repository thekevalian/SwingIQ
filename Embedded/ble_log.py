# import asyncio
# import struct
# from bleak import BleakScanner, BleakClient

# async def list_characteristics(address: str):
#     # Connect to the device
#     async with BleakClient(address) as client:
#         # Get the services and characteristics for the device
#         print(client)
#         services = await client.get_services()

#         for i, service in enumerate(services):
#             print(f"Service: {service.uuid}")
#             for characteristic in service.characteristics:
#                 if characteristic.uuid == "d65d0396-0001-4381-9985-653653ce831f":
#                     val = struct.pack('B', 3) 
#                     await client.write_gatt_char(characteristic.uuid, val, response=True)
#                     print("wrote to device")
#                 try:
#                     print(f"  Characteristic: {characteristic.uuid}")
#                     print(f"    Properties: {characteristic.properties}")
#                     print(f"    Handle: {characteristic.handle}")
#                     print(f"    Value: {await client.read_gatt_char(characteristic.uuid)}")
#                 except:
#                     print(f"Error with {characteristic.uuid}")
#                     pass

# async def discover_device():
#     # Scan for devices (you can also specify a specific device address)
#     devices = await BleakScanner.discover()
#     for device in devices:
#         if device.name == "BLE_events":
#             await list_characteristics(device.address)

# # Run the discover_device function
# asyncio.run(discover_device())


import asyncio
from bleak import BleakScanner, BleakClient

out_file = "file.txt"
async def notification_callback(sender: int, data: bytearray):
    """
    Callback function that will be called when a notification is received.
    """
    print(data.decode())

async def list_characteristics(address: str):
    """
    List characteristics and subscribe to notifications on a specific characteristic.
    """
    async with BleakClient(address) as client:
        services = await client.get_services()

        for service in services:
            for characteristic in service.characteristics:
                
                print(f"  Characteristic: {characteristic.uuid}")
                print(f"    Properties: {characteristic.properties}")
                
                # Check if the characteristic supports notifications (i.e., it has 'notify' property)
                if characteristic.uuid in "a3aa43cc-e0fd-4e9e-81c0-dcf57d2d06ff":
                    print(f"  Subscribing to notifications for {characteristic.uuid}")
                    await client.start_notify(characteristic.uuid, notification_callback)
                    print("Notifications started...")

                    # Optionally, wait for some time or stop after certain conditions
                    await asyncio.sleep(500)  # For example, wait for 10 seconds
                    await client.stop_notify(characteristic.uuid)
                    print(f"Stopped notifications for {characteristic.uuid}")

async def discover_device():
    """
    Scan for devices and call list_characteristics when the target device is found.
    """
    devices = await BleakScanner.discover()
    for device in devices:
        if device.name == "SwingIQ":  # Replace with your device's name or criteria
            await list_characteristics(device.address)

# Run the discover_device function to start scanning and enable notifications
asyncio.run(discover_device())
