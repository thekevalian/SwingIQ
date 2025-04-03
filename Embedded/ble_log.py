# ALTERNATIVE
# USE LIGHTBLUE App
import asyncio
from bleak import BleakScanner, BleakClient

out_file = "file.txt"
async def notification_callback(sender: int, data: bytearray):
    print(data.decode())

async def list_characteristics(address: str):
    async with BleakClient(address) as client:
        services = await client.get_services()

        for service in services:
            for characteristic in service.characteristics:
                
                print(f"  Characteristic: {characteristic.uuid}")
                print(f"    Properties: {characteristic.properties}")
                
                if characteristic.uuid in "a3aa43cc-e0fd-4e9e-81c0-dcf57d2d06ff":
                    print(f"  Subscribing to notifications for {characteristic.uuid}")
                    await client.start_notify(characteristic.uuid, notification_callback)
                    print("Notifications started...")

                    await asyncio.sleep(500)
                    await client.stop_notify(characteristic.uuid)
                    print(f"Stopped notifications for {characteristic.uuid}")

async def discover_device():
    devices = await BleakScanner.discover()
    for device in devices:
        if device.name == "SwingIQ":
            await list_characteristics(device.address)

asyncio.run(discover_device())
