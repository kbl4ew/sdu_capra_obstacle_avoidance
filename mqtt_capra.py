import paho.mqtt.client as mqtt

# def show_camera():
#         # The Video<Capture command opnes a pipeline to the upd camera feed, the cap variable then become the access to the feed and is simply displayed here. Then the OpenCV library can be used to save to files. 
#     cap = cv2.VideoCapture("udpsrc port=8001 ! application/x-rtp,payload=96,encoding-name=H264 ! rtpjitterbuffer mode=1 ! rtph264depay ! h264parse ! decodebin ! videoconvert ! appsink", cv2.CAP_GSTREAMER)
#     if cap.isOpened():
#         window_handle = cv2.namedWindow("CSI Camera", cv2.WINDOW_AUTOSIZE)
#             # Window
#         while cv2.getWindowProperty("CSI Camera", 0) >= 0:
#         #while (True):
#             ret_val, img = cap.read()
#             cv2.imshow("CSI Camera", img)
#             # This also acts as
#             keyCode = cv2.waitKey(30) & 0xFF
#             # Stop the program on the ESC key
#             if keyCode == 27:
#                 break
#         cap.release()
#         cv2.destroyAllWindows()
#     else: 
#         print("could not open\n")

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("capra/odometry")

def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    publish_velocity(client, 0.0, 0.0)

def publish_velocity(client, linear_velocity, angular_velocity):
    payload = {"header":{"frame_id":"frame_id"},"twist":{"linear":{"x":linear_velocity,"y":0.0,"z":0.0},"angular":{"x":0.0,"y":0.0,"z":angular_velocity}}}
    client.publish("capra/direct_velocity", payload)


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("10.46.28.1", 1883, 60)

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_forever()


# show_camera()

