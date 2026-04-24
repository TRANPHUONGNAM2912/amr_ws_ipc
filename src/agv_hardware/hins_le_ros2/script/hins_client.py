#!/usr/bin/env  python3
#coding:utf-8

from hins_laser_interfaces.srv import HinsSrv
import sys
from rclpy.node import Node
import rclpy

def hins_response_callback(self,response):
    print("hins_response_callback")
    pass

class GetHinsNode(Node): #GetHinsNode是继承于Node
#在__init__函数中创建一个服务的客户端
    def __init__(self):
        super().__init__("hins_client")
        self.get_logger().info("节点已启动：hins_client!")
        self.get_laser_data_client_ = self.create_client(HinsSrv, "HinsLESrv")

    def respoonse_callback(self,response):
        # 打印一下信息
        result = response.result()
        self.get_logger().info("success:{0}  area1:{1},  area2:{2},  area3:{3}".format(result.success, result.area1, result.area2, result.area3))

    def request_laser_data(self):
        #等待服务启动，每1s检查一次，如果服务没有启动，则一直循环
        while not self.get_laser_data_client_.wait_for_service(1.0):
            self.get_logger().warn("服务未启动")
        # 构建请求内容
        request = HinsSrv.Request()
        request.channel = int(0)  # 通道号
        
        self.get_laser_data_client_.call_async(request).add_done_callback(self.respoonse_callback)




def main(args=None):
    """
    ros2运行该节点的入口函数，可配置函数名称
    """
    print("main begin")
    rclpy.init(args=args) # 初始化rclpy
    node = GetHinsNode()  # 新建一个节点
    while(1):
        node.request_laser_data() 
        rclpy.spin_once(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）  
    rclpy.shutdown() # rcl关闭
    
main()
