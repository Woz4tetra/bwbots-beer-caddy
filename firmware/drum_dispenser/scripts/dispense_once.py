import paho.mqtt.client as mqtt


def on_is_dispensing(client, userdata, msg):
    payload = str(msg.payload.decode("utf-8"))
    device_name, state = payload.split("\t")
    is_dispensing = state == "1"
    if is_dispensing:
        print(f"{device_name} is dispensing")


def start_dispense(device_name: str):
    info = client.publish("start_dispense", device_name.encode("utf-8"), qos=0)
    info.wait_for_publish()
    if not info.is_published():
        print("Dispense message failed to publish!")


# Give a name to this MQTT client
client = mqtt.Client("drum_dispenser")
client.message_callback_add("is_dispensing", on_is_dispensing)

# IP address of your MQTT broker
client.connect("0.0.0.0", 1883)

client.loop_start()
client.subscribe("is_dispensing/#")

try:
    while True:
        input("Press enter to dispense> ")
        start_dispense("drum_dispenser")
finally:
    client.loop_stop()
