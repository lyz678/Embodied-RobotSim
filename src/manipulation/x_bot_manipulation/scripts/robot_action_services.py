"""Shared scan and home service calls for manipulation demos."""
import time
from std_srvs.srv import Trigger


class RobotActionServices:
    def perform_scan(self) -> bool:
        """执行扫描"""
        self.get_logger().info(">>> Triggering Scan Sequence...")

        if not self.scan_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Scan service not available")
            return False

        req = Trigger.Request()
        future = self.scan_client.call_async(req)
        while not future.done():
            time.sleep(0.1)

        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"Scan Success: {res.message}")
                return True
            else:
                self.get_logger().error(f"Scan Failed: {res.message}")
                return False
        except Exception as e:
            self.get_logger().error(f"Scan call failed: {e}")
            return False

    def perform_go_home(self) -> bool:
        """返回初始位置 (使用 Robot Actions Service)"""
        self.get_logger().info(">>> Triggering Go Home...")

        if not self.go_home_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Go Home service not available")
            return False

        req = Trigger.Request()
        future = self.go_home_client.call_async(req)
        while not future.done():
            time.sleep(0.1)

        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"Go Home Success: {res.message}")
                return True
            else:
                self.get_logger().error(f"Go Home Failed: {res.message}")
                return False
        except Exception as e:
            self.get_logger().error(f"Go Home call failed: {e}")
            return False
