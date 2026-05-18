import asyncio
from bleak import BleakScanner, BleakClient

# Estos son los nombres y UUIDs que configuraste en tu código de Arduino
DEVICE_NAME = "ArduinoGigaR1"
CHARACTERISTIC_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"

# Esta función se activa automáticamente cada vez que Arduino envía un dato nuevo
def notification_handler(sender, data):
    # Decodificamos los bytes recibidos a texto
    texto_recibido = data.decode('utf-8')
    
    # Separamos el texto usando la coma como divisor
    try:
        vl1, vl2, us = texto_recibido.split(',')
        print(f"📥 Datos recibidos -> Óptico 1: {vl1} mm |  Ultrasónico: {us} mm |Óptico 2: {vl2} mm")
    except ValueError:
        print(f"Formato inesperado recibido: {texto_recibido}")

async def main():
    print("Buscando dispositivos Bluetooth...")
    devices = await BleakScanner.discover()
    
    # Buscamos nuestro Arduino por el nombre
    arduino_device = None
    for d in devices:
        if d.name == DEVICE_NAME:
            arduino_device = d
            break
            
    if not arduino_device:
        print(f"No se encontró ningún dispositivo llamado {DEVICE_NAME}")
        return

    print(f"Encontrado {DEVICE_NAME} con dirección {arduino_device.address}")
    print("Conectando...")

    # Nos conectamos y nos suscribimos a las notificaciones
    async with BleakClient(arduino_device.address) as client:
        print("¡Conectado! Esperando datos (Presiona Ctrl+C para salir)...")
        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
        
        # Mantenemos el programa corriendo indefinidamente
        while True:
            await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())