# Copyright 2023 Josh Newans
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Int32
from std_msgs.msg import ColorRGBA
import time


from tkinter import *
from tkinter import colorchooser



class ButtonTester(Node):

    def __init__(self):
        super().__init__('button_tester')


        self.btn_press_sub = self.create_subscription(Int32,"/pico_publisher",self.btn_press_cb,rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self.col_pub = self.create_publisher(ColorRGBA, '/button_color', 10)


        self.tk = Tk()
        self.tk.title("Press The Button")
        Label(self.tk, text="Press the button!", font=("Arial",64)).pack(side=TOP)

        self.colour_code = (0,0,0)
        self.colour_changed = False

        self.update_gui()


    def update_gui(self):
        self.tk.update()

    

    def btn_press_cb(self, data):

        self.get_logger().info(f'Button pressed')

        color_code = colorchooser.askcolor(title ="Choose color") 
        print(color_code)
        self.colour_code = color_code[0]
        self.colour_changed = True


def main(args=None):
    
    rclpy.init(args=args)

    button_tester = ButtonTester()

    button_tester.create_rate(100)

    while rclpy.ok():
        rclpy.spin_once(button_tester)
        button_tester.update_gui()
        if button_tester.colour_changed:
            button_tester.colour_changed = False
            msg = ColorRGBA()
            msg.r = button_tester.colour_code[0]/255.0
            msg.g = button_tester.colour_code[1]/255.0
            msg.b = button_tester.colour_code[2]/255.0
            button_tester.col_pub.publish(msg)


    button_tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

