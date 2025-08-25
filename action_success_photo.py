import rclpy
from rclpy.node import Node
from rclpy.client import Client

# 导入动作结果消息类型
from irobot_create_msgs.msg import ActionResult
# 导入拍照服务类型
from std_srvs.srv import Trigger

class ActionSuccessPhotoTrigger(Node):
    def __init__(self):
        super().__init__('action_success_photo_trigger')
        
        # 创建拍照服务客户端
        self.photo_client = self.create_client(Trigger, '/capture_photo')
        
        # 等待拍照服务就绪
        while not self.photo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待拍照服务 /capture_photo 就绪...')
        
        # 订阅所有动作结果话题（可以根据需要指定特定动作）
        self.action_result_sub = self.create_subscription(
            ActionResult,
            '/rotate_angle/result',  # 监听旋转动作结果
            self.action_result_callback,
            10)
            
        self.action_result_sub2 = self.create_subscription(
            ActionResult,
            '/drive_distance/result',  # 监听移动距离动作结果
            10)
            
        self.get_logger().info('已启动：等待动作成功信号...')

    def action_result_callback(self, msg):
        """处理动作结果回调"""
        # 检查动作是否成功
        if msg.result.success:
            self.get_logger().info('检测到动作执行成功，发送拍照指令...')
            self.send_photo_request()

    def send_photo_request(self):
        """发送拍照请求"""
        req = Trigger.Request()
        future = self.photo_client.call_async(req)
        future.add_done_callback(self.photo_response_callback)

    def photo_response_callback(self, future):
        """处理拍照服务响应"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"拍照成功：{response.message}")
            else:
                self.get_logger().info(f"拍照失败：{response.message}")
        except Exception as e:
            self.get_logger().error(f"调用拍照服务失败：{str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ActionSuccessPhotoTrigger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    