import paho.mqtt.client as mqtt


def on_is_dispensing(client, userdata, msg):
    payload = str(msg.payload.decode("utf-8"))
    device_name, state = payload.split("\t")
    is_dispensing = state == "1"
    print(f"{device_name} is {'' if is_dispensing else 'not '}dispensing")

def on_has_drink(client, userdata, msg):
    payload = str(msg.payload.decode("utf-8"))
    device_name, state = payload.split("\t")
    has_drink = state == "1"
    print(f"{device_name} does {'' if has_drink else 'not '}have a drink")


def start_dispense(device_name: str):
    info = client.publish("start_dispense", device_name.encode("utf-8"), qos=0)
    info.wait_for_publish()
    if not info.is_published():
        print("Dispense message failed to publish!")


# Give a name to this MQTT client
client = mqtt.Client("dispenserA0")
client.message_callback_add("is_dispensing", on_is_dispensing)
client.message_callback_add("has_drink", on_has_drink)

# IP address of your MQTT broker, using ipconfig to look up it  
client.connect("192.168.0.16", 1883)

client.loop_start()
client.subscribe("is_dispensing/#")
client.subscribe("has_drink/#")

# client.loop_forever()
try:
    while True:
        input("Press enter to dispense> ")
        start_dispense("dispenser1")
finally:
    client.loop_stop()
