import gpiozero

sensor = gpiozero.DistanceSensor(3, 5)

# while True:
#     if (sensor.pin_factory):
#         print("yay")

sensor2 = gpiozero.Device(sensor.pin_factory)
while True:
    print(sensor2.value)
