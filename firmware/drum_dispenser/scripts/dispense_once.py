import paho.mqtt.client as mqtt

was_dispensing = False
did_connect = False


def on_is_dispensing(client, userdata, msg):
    global was_dispensing, did_connect
    if not did_connect:
        print("Dispenser connected!")
        did_connect = True
    payload = str(msg.payload.decode("utf-8"))
    device_name, state = payload.split("\t")
    is_dispensing = state == "1"
    if is_dispensing != was_dispensing:
        print(f"{device_name} is {'' if is_dispensing else 'not '}dispensing")
    was_dispensing = is_dispensing


def set_speed(speed: int):
    info = client.publish("dispense_speed", str(speed).encode("utf-8"), qos=0)
    info.wait_for_publish()
    if not info.is_published():
        print("Speed message failed to publish!")


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
        set_speed(100)
        start_dispense("drum_dispenser")
finally:
    client.loop_stop()
