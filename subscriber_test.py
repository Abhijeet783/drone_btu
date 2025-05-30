import paho.mqtt.client as mqtt

broker = "192.168.6.27"
port = 1883
topic ="test/topic"
def on_message(client, userdata, message):
	print(f"Recieived: {message.payload.decode()}")

client = mqtt.Client()
client.connect(broker, port)
client.subscriber(topic)
client.on_message = on_message

client.loop_forever()


