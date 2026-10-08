"""首次调用时记录控制函数和 ROS 接口，避免高频日志刷屏。"""
import json


def log_control_interface(node, function, endpoint, message_type, controller, publisher=None):
    """按函数和接口去重；优先记录重映射后的实际话题。"""
    if publisher is not None and hasattr(publisher, 'get_topic_name'):
        endpoint = publisher.get_topic_name()
    logged = getattr(node, '_logged_control_interfaces', None)
    if logged is None:
        logged = node._logged_control_interfaces = set()
    key = (function, endpoint)
    if key in logged:
        return
    node.get_logger().info(json.dumps(dict(
        event='control_interface', message='控制指令接口已调用', function=function,
        endpoint=endpoint, message_type=message_type, controller=controller), ensure_ascii=False))
    logged.add(key)
