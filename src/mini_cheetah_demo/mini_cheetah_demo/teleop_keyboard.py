#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg_banner = """
---------------------------------------------------
Mini Cheetah 四足机器人 键盘控制节点 (Teleop Keyboard)
---------------------------------------------------
控制键功能说明:
   w : 增加前进速度 (+Vx)
   s : 增加后退速度 (-Vx)
   a : 增加左转角速度 (+Wz)
   d : 增加右转角速度 (-Wz)
   j : 增加左侧移速度 (+Vy)
   l : 增加右侧移速度 (-Vy)

空格 / k : 紧急刹车静止 (Vx=0, Vy=0, Wz=0)
   m : 切换 原地踏步 模式 (不移动时仍交替抬腿)
   r : 复位机器狗 (重置 Gazebo 坐标)
q / Ctrl+C : 退出程序
---------------------------------------------------
"""


class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # 初始速度
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # 步长增量
        self.speed_step = 0.2
        self.turn_step = 0.3
        self.march_in_place = False

        self.get_logger().info('键盘遥控节点已启动！')

    def publish_cmd(self):
        if not rclpy.ok():
            return
        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy
        twist.linear.z = 1.0 if self.march_in_place else 0.0
        twist.angular.z = self.wz
        try:
            self.publisher_.publish(twist)
            mode_str = "踏步" if self.march_in_place else "静止"
            print(f"\r当前状态 -> 线性 X: {self.vx:+.2f} | 侧移 Y: {self.vy:+.2f} | 转向 Z: {self.wz:+.2f} | 待机模式: {mode_str}      ", end="", flush=True)
        except Exception:
            pass


def getKey(settings):
    if not sys.stdin.isatty():
        return ''
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    except Exception:
        return ''


def main(args=None):
    if not sys.stdin.isatty():
        print("[提示] 当前终端不支持交互式按键读取。后台控制器已默认启动 0.4 m/s 前进小跑！")
        print("如需发布控制命令，可使用命令行: ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {z: 0.5}}\"")
        return

    rclpy.init(args=args)
    node = TeleopKeyboardNode()

    try:
        settings = termios.tcgetattr(sys.stdin)
    except Exception:
        settings = None

    print(msg_banner)

    try:
        while rclpy.ok():
            if settings is None:
                break
            key = getKey(settings)
            if key == 'w':
                node.vx = min(1.0, node.vx + node.speed_step)
            elif key == 's':
                node.vx = max(-1.0, node.vx - node.speed_step)
            elif key == 'a':
                node.wz = min(1.5, node.wz + node.turn_step)
            elif key == 'd':
                node.wz = max(-1.5, node.wz - node.turn_step)
            elif key == 'j':
                node.vy = min(0.5, node.vy + node.speed_step)
            elif key == 'l':
                node.vy = max(-0.5, node.vy - node.speed_step)
            elif key == ' ' or key == 'k':
                node.vx = 0.0
                node.vy = 0.0
                node.wz = 0.0
            elif key == 'm':
                node.march_in_place = not node.march_in_place
                print(f"\n[系统提示] 原地踏步模式已 {'开启' if node.march_in_place else '关闭'}")
            elif key == 'r':
                print("\n[系统提示] 正在将机器狗位置重置到出生点...")
                node.vx = 0.0
                node.vy = 0.0
                node.wz = 0.0
                os.system("gz service -s /world/mini_cheetah_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --req 'name: \"mini_cheetah\", position: {x: 0, y: 0, z: 0.35}, orientation: {w: 1, x: 0, y: 0, z: 0}' >/dev/null 2>&1")
            elif key == 'q' or key == '\x03':
                break

            if rclpy.ok():
                node.publish_cmd()

    except Exception as e:
        pass

    finally:
        if settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("\n键盘控制节点已正常退出。")


if __name__ == '__main__':
    main()
