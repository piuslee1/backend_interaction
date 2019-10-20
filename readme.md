# Backend Interaction

This is a http server that takes messages from the frontend and pushed them to ros.


## ROS

publishes a string in json to 'motor_control'

There are two different message posibilities, arm positions:
```json
{
    "arm":{
        "x" : 0,
        "y" : 0, #up down
        "z" : 0,
        "x_angle" : 0, #side by side
        "y_angle" : 0, #up down
        "rotation" : 0 #spinning back and forth
    }  
}
```

and drive_train position
```json
{
    "drive_train":{
        "x" : 0,
        "y" : 0
    }
}
```