#!/usr/bin/env python3

import rospy
import time
import requests
import threading
# from std_msgs.msg import String, Bool
from alitrak_lift.srv import Lift, LiftResponse

class LiftControl():
    def __init__(self):

        #self.api_url = "http://192.168.127.254" # Default MOXA
        self.api_url: str = rospy.get_param("~api_url", "http://10.10.1.45") # DTI MOXA IP

        self.lift_motor_raise: int = rospy.get_param("~lift_roller_speed") #port number
        self.lift_motor_lower: int = rospy.get_param("~lift_roller_dir") #port number

        # self.sensor_front_power = rospy.get_param("~sensor_front_power") #port number
        # self.sensor_front_input = rospy.get_param("~sensor_front_input") #port number
        # self.sensor_front_state = False

        # self.sensor_rear_power = rospy.get_param("~sensor_rear_power") #port number
        # self.sensor_rear_input = rospy.get_param("~sensor_rear_input") #port number
        # self.sensor_rear_state = False

        self.max_raise_time: int = rospy.get_param("~max_raise_time") # seconds
        self.max_lower_time: int = rospy.get_param("~max_lower_time") # seconds
        
        # self.sensor_data_rate = rospy.get_param("~sensor_data_rate") # Hz
        
        self.raise_direction_val: bool = rospy.get_param("~load_direction_val")
        self.lower_direction_val: bool = rospy.get_param("~unload_direction_val")

        _serv_lift = rospy.Service('lift_activation', Lift, self.handle_lift)
        # self.pub_sensor_front = rospy.Publisher("lift/sensor_front", Bool, queue_size=1)
        # self.pub_sensor_rear = rospy.Publisher("lift/sensor_rear", Bool, queue_size=1)
        rospy.loginfo("lift node has a service /lift_activation with string argument options: 'raise' or 'lower'")
        rospy.loginfo("Establishing a connection..")
        
        # Start thread that checks IO LogicBox state
        watchdog = threading.Thread(target=self.logicbox_watcher) 
        watchdog.start()

#region
        # Start threads that reads box sensors state
        # self.set_do_port(self.sensor_front_power,1) # Power on Box sensor rear
        # self.set_do_port(self.sensor_rear_power,1) # Power on Box sensor front
        # sensor = threading.Thread(target=self.box_sensor_thread) 
        # sensor.start()

    # def box_sensor_thread(self): # Checking state of the box sensor reader
    #     rate = rospy.Rate(self.sensor_data_rate)
    #     while not rospy.is_shutdown():
    #         sensor_state = self.get_di_port(self.sensor_front_input) # Box sensor state reader
    #         if sensor_state == 0:
    #             self.sensor_front_state = True
    #             self.pub_sensor_front.publish(self.sensor_front_state) # Box detected
    #         elif sensor_state == 1:
    #             self.sensor_front_state = False 
    #             self.pub_sensor_front.publish(self.sensor_front_state) # No box detections
    #         else:
    #             rospy.logwarn("Digital input from box sensor is not responding as expected!") 
    #             # Is the logicbox powered on?
    #             # Is the ethernet cable connected?
    #             # Are you on the same IP range as the logicbox?

    #         sensor_state = self.get_di_port(self.sensor_rear_input) # Box sensor state reader
    #         if sensor_state == 0:
    #             self.sensor_rear_state = True
    #             self.pub_sensor_rear.publish(self.sensor_rear_state) # Box detected
    #         elif sensor_state == 1: 
    #             self.sensor_rear_state = False
    #             self.pub_sensor_rear.publish(self.sensor_rear_state) # No box detections
    #         else:
    #             rospy.logwarn("Digital input from box sensor is not responding as expected!") 
    #             # Is the logicbox powered on?
    #             # Is the ethernet cable connected?
    #             # Are you on the same IP range as the logicbox?
    #         rate.sleep()

#endregion
    def logicbox_watcher(self): # Pinging to check connection
        initial_check = True
        time.sleep(2) # Initially wait before checking connection
        rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            state = self.get_deviceInfo() # LogicBox state reader
            if state == 200 and initial_check:
                rospy.loginfo("Connection to lift logicbox established.")
                initial_check = False # State OK at initial check
            elif state == 200:
                pass # State OK
            else:
                rospy.logwarn("IO LogixBox is not responding as expected!")
            rate.sleep()

    def handle_lift(self, req: Lift)-> LiftResponse:
        rospy.wait_for_service('lift_activation')
        try:
            lift_activation = rospy.ServiceProxy('lift_activation', Lift)
            rospy.loginfo(f"Service invoked with action: {req.action}")
            response: LiftResponse = LiftResponse()
            if req.action.lower() == "raise":
                response.result = self.raise_lift()
            elif req.action.lower() == "lower":
                response.result = self.lower_lift()
            else:
                rospy.logwarn("Invalid service request")
            return response
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def raise_lift(self):
        start_load = time.time()
        while True:
            self.set_do_port(self.lift_motor_raise,True) # Start raising lift
            run_time = time.time() - start_load
            if run_time > self.max_raise_time:
                self.set_do_port(self.lift_motor_raise,True) # Stop raising lift
                return True
        self.set_do_port(self.lift_motor_raise,False) # Stop raising lift
        return True

    def lower_lift(self):
        start_unload = time.time()
        while True: #self.sensor_front_state == True or self.sensor_rear_state == True:
            self.set_do_port(self.lift_motor_lower,True) # Start lowering lift
            run_time = time.time() - start_unload
            if run_time > self.max_lower_time:
                self.set_do_port(self.lift_motor_lower,False) # Stop lowering lift
                return True
        self.set_do_port(self.lift_motor_lower,0) # Stop lowering lift
        return True

    def get_deviceInfo(self): 
        api_url = self.api_url+"/api/slot/0/sysInfo/device"
        headers =  {"Content-Type":"application/json", "Accept": "vdn.dac.v1"}
        try:
            response = requests.get(api_url, headers=headers, timeout=2)
            response.raise_for_status()  # Raises a HTTPError if the status is 4xx, 5xxx
        except requests.exceptions.ConnectionError as ce:
            rospy.logwarn("IO LogixBox is not responding - ConnectionError: {}".format(ce))
        except requests.exceptions.Timeout as to:
            rospy.logwarn("IO LogixBox is not responding - Timeout: {}".format(to))
        except requests.exceptions.HTTPError as he:
            rospy.logwarn("IO LogixBox is not responding - HTTPError: {}".format(he))
        except requests.exceptions.TooManyRedirects as tmr:
            rospy.logwarn("IO LogixBox is not responding - TooManyRedirects: {}".format(tmr))
        else:
            return int(response.status_code)
        #response = requests.get(api_url, headers=headers)
        #print(response.status_code)
        #print(response.content)
#region
    # def get_di_port(self, port):
    #     api_url = self.api_url+"/api/slot/0/io/di/"+str(port)+"/diStatus"
    #     headers =  {
    #         "Content-Type":"application/json",
    #         "Accept": "vdn.dac.v1"}
    #     response = requests.get(api_url, headers=headers)
    #     return int(response.text[-5]) # Returning 5th last char index in example: {"slot":0,"io":{"di":{"0":{"diStatus":1}}}}

    # def get_do_port(self, port): # Digital output
    #     api_url = self.api_url+"/api/slot/0/io/do/"+str(port)+"/doStatus"
    #     headers =  {
    #         "Content-Type":"application/json",
    #         "Accept": "vdn.dac.v1"}
    #     response = requests.get(api_url, headers=headers)
    #     return int(response.text[-5])
#endregion

    def set_do_port(self, port:int, value:bool): # True=On, False=Off
        api_url = self.api_url+"/api/slot/0/io/do/"+str(port)+"/doStatus"
        payload = '{"slot":"0","io":{"do":{"'+str(port)+'":{"doStatus":"'+str(value)+'"}}}}'
        payload_len = str(len(payload))
        headers =  {
            "Content-Type":"application/json",
            "Accept": "vdn.dac.v1",
            "Content-Length": payload_len
            }
        _response = requests.put(api_url, data=payload, headers=headers)

    def clean_shutdown(self):
        # Turn off power from moxa output ports
        rospy.logwarn("Node is being killed and all output power ports are being turned off")
        self.set_do_port(self.lift_motor_raise,False)
        self.set_do_port(self.lift_motor_lower,False)
        # self.set_do_port(self.sensor_front_power,0)
        # self.set_do_port(self.sensor_rear_power,0)

if __name__ == '__main__':
    print('Starting lift_control node')
    rospy.loginfo('Starting lift_control_node')
    rospy.init_node('lift_control_node')
    CC = LiftControl()
    rospy.on_shutdown(CC.clean_shutdown)
    rospy.spin()
