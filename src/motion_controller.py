import json
import numpy as np
from time import time
from pid_python.pid_controller import PIDController
class MotionController:
    def __init__( self, config = None, writeThrusts = lambda thrust: print("Thrusts are: ",thrust) ):
        if config is None:
            config = json.load(open("default_config.json"))
        self.config = config
        self.no_of_thrusters = config["no_of_thrusters"]
        self.direction_vectors = config["direction_vectors"]
        for dof,vector in range(zip(config["direction_vectors"].keys(),config["direction_vectors"].values())):
            self.direction_vectors[dof] = np.array(vector)
        self.thruster_min = config["thruster_min"]
        self.thruster_max = config["thruster_max"]
        self.control_mode = config["control_mode"]
        self.surge_controller = self.updatePIDParams(PIDController(),config["pid_params"]["surge"])
        self.sway_controller = self.updatePIDParams(PIDController(),config["pid_params"]["sway"])
        self.heave_controller = self.updatePIDParams(PIDController(),config["pid_params"]["heave"])
        self.roll_controller = self.updatePIDParams(PIDController(),config["pid_params"]["roll"])
        self.pitch_controller = self.updatePIDParams(PIDController(),config["pid_params"]["pitch"])
        self.yaw_controller = self.updatePIDParams(PIDController(),config["pid_params"]["yaw"])
        self.target_values = {"surge":0,"sway":0,"heave":0,"roll":0,"pitch":0,"yaw":0}
        self.current_values = {"surge":0,"sway":0,"heave":0,"roll":0,"pitch":0,"yaw":0}
        self.last_values = {"surge":0,"sway":0,"heave":0,"roll":0,"pitch":0,"yaw":0}
        self.thrusts = {"surge":0,"sway":0,"heave":0,"roll":0,"pitch":0,"yaw":0}
        self.last_update_time = 0
        self.current_time = 0
        self.net_thrusts = np.array([0]*self.no_of_thrusters)
        


    def updatePIDParams(self,pid_controller,pid_config):
        pid_controller.setKp(pid_config["kp"])
        pid_controller.setKi(pid_config["ki"])
        pid_controller.setKd(pid_config["kd"])
        pid_controller.setAcceptableError(pid_config["acceptable_error"])
        pid_controller.setIntegralMin(pid_config["integral_min"])
        pid_controller.setIntegralMax(pid_config["integral_max"])
        pid_controller.setOutMin(pid_config["output_min"])
        pid_controller.setOutMax(pid_config["output_max"])
        pid_controller.setFrequency(pid_config["frequency"])
        return pid_controller 

    def setSurgeThrust(self,thrust):
        if self.control_mode["surge"] == "open_loop":
            self.thrusts["surge"] = thrust
        else:
            print("Surge control mode is in closed loop")
        
    def setSwayThrust(self,thrust):
        if self.control_mode["sway"] == "open_loop":
            self.thrusts["sway"] = thrust
        else:
            print("Sway control mode is in closed loop")
    
    def setHeaveThrust(self,thrust):
        if self.control_mode["heave"] == "open_loop":
            self.thrusts["heave"] = thrust
        else:
            print("Heave control mode is in closed loop")
    
    def setRollThrust(self,thrust):
        if self.control_mode["roll"] == "open_loop":
            self.thrusts["roll"] = thrust
        else:
            print("Roll control mode is in closed loop")

    def setPitchThrust(self,thrust):
        if self.control_mode["pitch"] == "open_loop":
            self.thrusts["pitch"] = thrust
        else:
            print("Pitch control mode is in closed loop")

    def setYawThrust(self,thrust):
        if self.control_mode["yaw"] == "open_loop":
            self.thrusts["yaw"] = thrust
        else:
            print("Yaw control mode is in closed loop")
    def setTargetSurge(self,target):
        if self.control_mode["surge"] == "closed_loop":
            self.target_values["surge"] = target
            self.surge_controller.setTargetValue(target)
        else:
            print("Surge control mode is in open loop")
    def setTargetSway(self,target):
        if self.control_mode["sway"] == "closed_loop":
            self.target_values["sway"] = target
            self.sway_controller.setTargetValue(target)
        else:
            print("Sway control mode is in open loop")
    def setTargetHeave(self,target):
        if self.control_mode["heave"] == "closed_loop":
            self.target_values["heave"] = target
            self.heave_controller.setTargetValue(target)
        else:
            print("Heave control mode is in open loop")
    def setTargetRoll(self,target):
        if self.control_mode["roll"] == "closed_loop":
            self.target_values["roll"] = target
            self.roll_controller.setTargetValue(target)
        else:
            print("Roll control mode is in open loop")
    def setTargetPitch(self,target):
        if self.control_mode["pitch"] == "closed_loop":
            self.target_values["pitch"] = target
            self.pitch_controller.setTargetValue(target)
        else:
            print("Pitch control mode is in open loop")
    def setTargetYaw(self,target):
        if self.control_mode["yaw"] == "closed_loop":
            self.target_values["yaw"] = target
            self.yaw_controller.setTargetValue(target)
        else:
            print("Yaw control mode is in open loop")

    def setCurrentSurge(self,current):
        if self.control_mode["surge"] == "closed_loop":
            self.current_values["surge"] = current
        else:
            print("Surge control mode is in open loop")
    def setCurrentSway(self,current):
        if self.control_mode["sway"] == "closed_loop":
            self.current_values["sway"] = current
        else:
            print("Sway control mode is in open loop")
    def setCurrentHeave(self,current):
        if self.control_mode["heave"] == "closed_loop":
            self.current_values["heave"] = current
        else:
            print("Heave control mode is in open loop")
            
    def setCurrentRoll(self,current):
        if self.control_mode["roll"] == "closed_loop":
            self.current_values["roll"] = current
        else:
            print("Roll control mode is in open loop")
    def setCurrentPitch(self,current):
        if self.control_mode["pitch"] == "closed_loop":
            self.current_values["pitch"] = current
        else:
            print("Pitch control mode is in open loop")
    def setCurrentYaw(self,current):
        if self.control_mode["yaw"] == "closed_loop":
            self.current_values["yaw"] = current
        else:
            print("Yaw control mode is in open loop")
    
    def updateThrust(self):
        self.current_time = time() 
        if self.control_mode["surge"] == "closed_loop":
            self.thrusts["surge"] = self.surge_controller.updateOutput(self.current_values["surge"],(self.current_values["surge"]-self.last_values["surge"])/(self.current_time-self.last_update_time),self.current_time - self.last_update_time)
        if self.control_mode["sway"] == "closed_loop":
            self.thrusts["sway"] = self.sway_controller.updateOutput(self.current_values["sway"],(self.current_values["sway"]-self.last_values["sway"])/(self.current_time-self.last_update_time),self.current_time - self.last_update_time)
        if self.control_mode["heave"] == "closed_loop":
            self.thrusts["heave"] = self.heave_controller.updateOutput(self.current_values["heave"],(self.current_values["heave"]-self.last_values["heave"])/(self.current_time-self.last_update_time),self.current_time - self.last_update_time)
        if self.control_mode["roll"] == "closed_loop":
            self.thrusts["roll"] = self.roll_controller.updateOutput(self.current_values["roll"],(self.current_values["roll"]-self.last_values["roll"])/(self.current_time-self.last_update_time),self.current_time - self.last_update_time)
        if self.control_mode["pitch"] == "closed_loop":
            self.thrusts["pitch"] = self.pitch_controller.updateOutput(self.current_values["pitch"],(self.current_values["pitch"]-self.last_values["pitch"])/(self.current_time-self.last_update_time),self.current_time - self.last_update_time)
        if self.control_mode["yaw"] == "closed_loop":
            self.thrusts["yaw"] = self.yaw_controller.updateOutput(self.current_values["yaw"],(self.current_values["yaw"]-self.last_values["yaw"])/(self.current_time-self.last_update_time),self.current_time - self.last_update_time)
        self.last_values = self.current_values.copy()
        self.last_update_time = self.current_time
        self.net_thrusts = self.thrusts["surge"]*self.direction_vectors["surge"] + self.thrusts["sway"]*self.direction_vectors["sway"] + self.thrusts["heave"]*self.direction_vectors["heave"] + self.thrusts["roll"]*self.direction_vectors["roll"] + self.thrusts["pitch"]*self.direction_vectors["pitch"] + self.thrusts["yaw"]*self.direction_vectors["yaw"]
        self.net_thrusts = np.clip(self.net_thrusts,self.thruster_min,self.thruster_max)
        self.writeThrusts(self.net_thrusts) 


    
    
    