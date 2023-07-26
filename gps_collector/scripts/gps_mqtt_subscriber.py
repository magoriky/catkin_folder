#!/usr/bin/env python
import rospy
import paho.mqtt.client as mqtt
import numpy as np
from sensor_msgs.msg import NavSatFix




def get_info_from_string(string_message):
    string_message = string_message.splitlines()
    for line in string_message:
       if line.startswith("$GNRMC"):
           line = line.split(",") 
           return line[3], line[5] #? latitude, longitud
    

class async_client:
    def __init__(self, serverAddress,clientId,topicAddress):
        self.address = serverAddress
        self.clientId = clientId
        self.topic = topicAddress
        self.pub = rospy.Publisher('gps/fix', NavSatFix,queue_size=1)
    
    def on_connect(self,client,userdata,flags,rc):
        if rc == 0:
            print("connected to {}".format(self.address))
            client.subscribe(self.topic, qos=2)
            print("subscribed to {}".format(self.topic))
        else:
            print("Bad connection Returned code=", rc)

    
    
    def on_message(self, client, userdata, message):
        try:
            print("Message received")
            msg = str(message.payload.decode("utf-8"))
            latitude, longitude = get_info_from_string(msg)
            lat_dot = latitude.find(".")
            lon_dot = longitude.find(".")
            latitude = float(latitude[0:lat_dot-2]) + float(latitude[lat_dot-2:])/60      #? in degrees
            longitude = float(longitude[0:lon_dot-2]) + float(longitude[lon_dot-2:])/60   #? in degrees

            navsat_msg = NavSatFix()
            navsat_msg.header.frame_id = "sensor"
            navsat_msg.latitude = latitude
            navsat_msg.longitude = longitude
            self.pub.publish(navsat_msg)

        except:
            print("error on_message")
    
    



server = "rtk.aktgeo.net"
Id = "suny"
topic ="/agip2/gcm/gnss/12-12-12-12-12-12"
client = async_client(server,Id,topic)



if __name__ == '__main__':
    try:
        rospy.init_node('mqtt_subscriber', anonymous = True)
        while not rospy.is_shutdown():
            client_initializer = mqtt.Client(client_id="suny", clean_session = True)
            client_initializer.on_connect = client.on_connect
            client_initializer.on_message = client.on_message
            rospy.loginfo("connecting_talker")
            client_initializer.connect(client.address, 1883,60)
            client_initializer.loop_forever()

    except rospy.ROSInterruptException:
        pass
