from youBot2 import YouBot


class GamepadBot(YouBot):
    def run_step(self, count):
        # car control
        self.control_car()
        # arm control
        self.control_arm()
        # arm gripper
        self.control_gripper()
        # read lidar
        self.read_lidars()
        # read camera
        self.data['gripper_camera'].append(self.read_gripper_camera())
        self.data['counter'].append(count)
        self.read_gripper()
        


if __name__ == "__main__":
    client = GamepadBot()
    client.init_coppelia()
    client.run_coppelia()