# Copyright 2024 @libphy
#
# Licensed under the Apache License, Version 2.0 (the "License");
#     http://www.apache.org/licenses/LICENSE-2.0

from dataclasses import dataclass
from abc import abstractmethod
import numpy as np
from pynput import keyboard
from pynput.keyboard import Key, Listener
from inputs import get_gamepad
import pygame
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import pickle
import os

@dataclass
class Control:
    vel_X: float = 0
    vel_Y: float = 0
    vel_Z: float = 0
    arm_0: float = 0
    arm_1: float = 0
    arm_2: float = -np.pi / 4
    arm_3: float = -np.pi / 4
    arm_4: float = 0
    gripper: bool = False


class YouBot:
    def __init__(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.require("sim")
        self.run_flag = True
        self.control = Control()
        self.data=dict({'counter':list([]), 'gripper_camera':list([])})

    def on_press(self):
        deltaX, deltaZ = 5.0, np.pi/2
        events = pygame.event.get()
        print(len(events))
        if len(events)>0:
            event = events[0]
            if event.type==pygame.JOYHATMOTION:
                if event.value==(0,1):
                    print('forward')
                    self.control.vel_X += deltaX
                    self.control.vel_Z += min(deltaZ, abs(self.control.vel_Z)) * (
                        -1 if self.control.vel_Z > 0 else 1
                    )   
                if event.value==(0,-1):
                    print('backward')
                    self.control.vel_X -= deltaX
                    self.control.vel_Z += min(deltaZ, abs(self.control.vel_Z)) * (
                        -1 if self.control.vel_Z > 0 else 1
                    )                           
                if event.value==(-1,0):
                    self.control.vel_X += min(deltaX, abs(self.control.vel_X)) * (
                        -1 if self.control.vel_X > 0 else 1
                    )
                    self.control.vel_Z += deltaZ
                if event.value==(1,0):
                    self.control.vel_X += min(deltaX, abs(self.control.vel_X)) * (
                        -1 if self.control.vel_X > 0 else 1
                    )
                    self.control.vel_Z -= deltaZ
                if event.value==(0,0):
                    self.control.vel_X = 0
                    self.control.vel_Y = 0
                    self.control.vel_Z = 0

                self.control.vel_X = min(max(self.control.vel_X, -20), 20)
                self.control.vel_Y = 0
                self.control.vel_Z = min(max(self.control.vel_Z, -np.pi), np.pi)

            if event.type==pygame.JOYAXISMOTION:
                if event.axis==2:
                    delta = 0.1
                    self.control.arm_0 -= delta*event.value
                if event.axis==3:
                    delta = 0.1
                    self.control.arm_1 -= delta*event.value    
                if event.axis==4 and event.value>0:    
                    delta = 0.1
                    self.control.arm_2 -=delta*event.value
                if event.axis==5 and event.value>0:    
                    delta = 0.1
                    self.control.arm_2 +=delta*event.value
                if event.axis==1:    
                    delta = 0.1
                    self.control.arm_3 -= delta*event.value    
                if event.axis==0:    
                    delta = 0.1
                    self.control.arm_4 += delta*event.value  

            if event.type==pygame.JOYBUTTONDOWN:
                if event.button==2:
                    self.control.gripper = True
                if event.button==0:   
                    self.control.gripper = False 

                if event.button==1: #quit simulation
                    self.run_flag=False        

    def init_coppelia(self):
        if not os.path.isdir('data'):
            os.makedirs('data')
        # reference
        self.youBot_ref = self.sim.getObject("/youBot_ref")
        # Wheel Joints: front left, rear left, rear right, front right
        self.wheels = []
        self.wheels.append(self.sim.getObject("/rollingJoint_fl"))
        self.wheels.append(self.sim.getObject("/rollingJoint_rl"))
        self.wheels.append(self.sim.getObject("/rollingJoint_fr"))
        self.wheels.append(self.sim.getObject("/rollingJoint_rr"))
        # Arm Joints
        self.arms = []
        for i in range(5):
            self.arms.append(self.sim.getObject(f"/youBotArmJoint{i}"))
        # Gripper Joint
        # self.gripper = self.sim.getObject(f"/youBotGripperJoint1")
        self.j1 = self.sim.getObject("/youBotGripperJoint1")
        self.j2 = self.sim.getObject("/youBotGripperJoint2")
        # self.graph = self.sim.getObject("/Graph")
        # self.graph_x = self.sim.addGraphStream(self.graph, "pos_x", "m", 0,[1,0,0])
        # self.graph_y = self.sim.addGraphStream(self.graph, "pos_y", "m", 0,[0,0.5,1])   

        # lidar
        self.lidars = []
        for i in range(13):
            self.lidars.append(self.sim.getObject(f"/lidar_{i+1:02d}"))
        # camera
        self.gripper_camera = self.sim.getObject(f"/gripper_camera")

    def control_car(self):
        self.sim.setJointTargetVelocity(
            self.wheels[0],
            -self.control.vel_X + self.control.vel_Z,
        )
        self.sim.setJointTargetVelocity(
            self.wheels[1],
            -self.control.vel_X + self.control.vel_Z,
        )
        self.sim.setJointTargetVelocity(
            self.wheels[2],
            -self.control.vel_X - self.control.vel_Z,
        )
        self.sim.setJointTargetVelocity(
            self.wheels[3],
            -self.control.vel_X - self.control.vel_Z,
        )

    def control_arm(self):
        self.sim.setJointTargetPosition(self.arms[0], self.control.arm_0)
        self.sim.setJointTargetPosition(self.arms[1], self.control.arm_1)
        self.sim.setJointTargetPosition(self.arms[2], self.control.arm_2)
        self.sim.setJointTargetPosition(self.arms[3], self.control.arm_3)
        self.sim.setJointTargetPosition(self.arms[4], self.control.arm_4)
    
    # def control_gripper(gripper, state):
    def control_gripper(self):    
        gripper1 = self.j1
        gripper2 = self.j2
        state = self.control.gripper
        position1 = self.sim.getJointPosition(gripper1)
        position2 = self.sim.getJointPosition(gripper2)
   
        position1 = 0.000 if state else 0.025
        position2 = 0.000 if state else -0.025
        self.sim.setJointTargetPosition(gripper1, position1)
        self.sim.setJointTargetPosition(gripper2, position2)
        return position1, position2
    
    # def control_gripper(self):
    #     # self.sim.setJointTargetVelocity(self.gripper, self.control.gripper)

    #     self.sim.setJointTargetVelocity(self.j1, self.control.gripper)

    def read_lidars(self):
        scan = []
        for id in self.lidars:
            scan.append(self.sim.readProximitySensor(id))
        return scan

    def read_gripper_camera(self):
        result = self.sim.getVisionSensorImg(self.gripper_camera)
        img = np.frombuffer(result[0], dtype=np.uint8)
        img = img.reshape((result[1][1], result[1][0], 3))
        return img
    
    def read_gripper(self):
        result1 = self.sim.getObjectPosition(self.j1)
        result2 = self.sim.getObjectVelocity(self.j2)
        # print('gripper', result, self.control.gripper)
        # self.sim.setGraphStreamValue(self.graph, self.graph_x, result[0])
        # self.sim.setGraphStreamValue(self.graph, self.graph_y, result[1])
        # self.sim.setGraphStreamValue(self.graph, self.graph_y, self.control.gripper)
        
    def run_coppelia(self):
        # key input
        pygame.init()
        # Initialize the joystick module
        pygame.joystick.init()
        # Get the first joystick
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()

        # Listener(on_press=self.on_press).start()
        # start simulation

        self.sim.setStepping(True)
        self.sim.startSimulation()
        count = 0
        while self.run_flag:
            count += 1
            # step
            self.run_step(count)
            self.on_press()
            self.sim.step()
        self.sim.stopSimulation()
        with open('data/data.pkl','wb') as f:
            pickle.dump(self.data, f)

    @abstractmethod
    def run_step(self, count):
        pass
