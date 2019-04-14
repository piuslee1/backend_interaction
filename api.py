import falcon
import json
import base64
import numpy as np
import rospy
from std_msgs.msg import String


""" 
API GUIDE

Uses https://falconframework.org/ as the api handler

PATHS:

POST /drive_train
POST /arm

"""



#Gives direction to the drive train
class DriveTrain(object):
    def __init__(self):
        self.pub = rospy.Publisher('drive_train', String, queue_size=10)
        rospy.init_node('drive_train_api', anonymous=True)


    def on_post(self, req, resp):
        query = falcon.uri.decode(req.query_string)
        queries = query.split("&")
        
        #what all the data for the return package will go into 
        package = {}
        
        try:
            input_data = {
                "angle" : 0, 
                "force" : 0
            }

            #getting all the query parameters out of the POST url
            for each in queries:
                name, data = each.split("=")
                if(name in input_data):
                    input_data[name] = float(data)


            message = json.dumps(input_data)

            rospy.loginfo(message)
            self.pub.publish(message)

            resp.status = falcon.HTTP_200  # This is the default status
            resp.body = (json.dumps({"message":"successfully published"}))
        
        
        except Exception as e:
            resp.status = falcon.HTTP_500  # Error
            resp.body = (json.dumps({"message":"something failed, check error", "error":e.strerror}))



#Gives endpoints for the arm
class Arm(object):
    def __init__(self): 
        self.pub = rospy.Publisher('arm', String, queue_size=10)
        rospy.init_node('arm_api', anonymous=True)


    def on_post(self, req, resp):
        query = falcon.uri.decode(req.query_string)
        queries = query.split("&")
        
        #what all the data for the return package will go into 
        package = {}
        
        try:
            input_data = {
                "x" : 0, 
                "y" : 0, #up down
                "z" : 0,
                "x_angle" : 0, #side by side
                "y_angle" : 0, #up down
                "rotation" : 0 #spinning back and forth
            }

            #getting all the query parameters out of the POST url
            for each in queries:
                name, data = each.split("=")
                if(name in input_data):
                    input_data[name] = float(data)


            message = json.dumps(input_data)

            rospy.loginfo(message)
            self.pub.publish(message)

            resp.status = falcon.HTTP_200  # This is the default status
            resp.body = (json.dumps({"message":"successfully published"}))
        
        
        except Exception as e:
            resp.status = falcon.HTTP_500  # Error
            resp.body = (json.dumps({"message":"something failed, check error", "error":e.strerror}))




# falcon.API instances are callable WSGI apps
app = falcon.API()

drive_train = DriveTrain()
arm = Arm()

app.add_route('/drive_train', drive_train)
app.add_route('/arm', arm)
