import falcon
import json
import base64
import numpy as np
import rospy
from std_msgs.msg import String, Float64MultiArray
import threading


""" 
API GUIDE

Uses https://falconframework.org/ as the api handler

To run 
pip install gunicorn
gunicorn api:app

PATHS:

POST /drive_train
POST /arm

"""

production = False


drive_pub = rospy.Publisher('drive_train', Float64MultiArray, queue_size=10)
arm_pub = rospy.Publisher('arm', Float64MultiArray, queue_size=10)
threading.Thread(target=lambda: rospy.init_node('movement', anonymous=True, disable_signals=True)).start()


#Gives direction to the drive train
class DriveTrain(object):
    def __init__(self, pub):
        self.pub = pub

    def on_post(self, req, resp):
        query = falcon.uri.decode(req.query_string)
        queries = query.split("&")
        
        #what all the data for the return package will go into 
        package = {}
        
        try:
            drive_train = {
                "x" : 0, 
                "y" : 0
            }

            #getting all the query parameters out of the POST url
            for each in queries:
                if("=" in each):
                    name, data = each.split("=")
                    if(name in drive_train):
                        drive_train[name] = float(data)


            message = Float64MultiArray()
            message.data = [drive_train["x"], drive_train["y"]]

            self.pub.publish(message)

            resp.status = falcon.HTTP_200  # This is the default status
            resp.body = (json.dumps({"message":"successfully published"}))
        
        
        except Exception as e:
            resp.status = falcon.HTTP_500  # Error
            resp.body = (json.dumps({"message":"something failed, check error", "error":e}))



#Gives endpoints for the arm
class Arm(object):
    def __init__(self, pub): 
        self.pub = pub 

    def on_post(self, req, resp):
        query = falcon.uri.decode(req.query_string)
        queries = query.split("&")
        
        #what all the data for the return package will go into 
        package = {}
        
        try:

            arm = {
                "x" : 0, 
                "y" : 0, #up down
                "z" : 0,
                "x_angle" : 0, #side by side
                "y_angle" : 0, #up down
                "rotation" : 0 #spinning back and forth
            }

            #getting all the query parameters out of the POST url
            for each in queries:
                if("=" in each):
                    name, data = each.split("=")
                    if(name in arm):
                        arm[name] = float(data)

            
            messsage = Float64MultiArray()
            message.data = [arm["x"], arm["y"], arm["z"], arm["x_angle"], arm["y_angle"], arm["rotation"]]

            self.pub.publish(message)

            resp.status = falcon.HTTP_200  # This is the default status
            resp.body = (json.dumps({"message":"successfully published"}))
        
        
        except Exception as e:
            resp.status = falcon.HTTP_500  # Error
            resp.body = (json.dumps({"message":"something failed, check error", "error":e.error}))




#Gives endpoints for the arm
class Imu(object):
    def __init__(self): 
        self.pub = rospy.Subscriber('imu', String, self.update_data)
        self.imu_data = "data"

    def update_data(self, imu_data):
        self.imu_data = imu_data.data

    def on_get(self, req, resp):
        query = falcon.uri.decode(req.query_string)

        resp.status = falcon.HTTP_200  # This is the default status
        resp.body = self.imu_data


if(not production):
    from falcon_cors import CORS
    public_cors = CORS(allow_all_origins=True, allow_methods_list=["GET","PATCH","POST","DELETE"])
    app = falcon.API(middleware=[public_cors.middleware])
else:
    app = falcon.API()

drive_train = DriveTrain(drive_pub)
arm = Arm(arm_pub)
imu = Imu()

app.add_route('/drive_train', drive_train)
app.add_route('/arm', arm)
app.add_route('/imu_data', imu)
