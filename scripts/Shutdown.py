#!/usr/bin/python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
import time
import subprocess
import time
import signal
import os
import psutil
import sys


class ShutdownNode(Node):
    def __init__(self):
        super().__init__("orchestrator_node")
        self.subscription = self.create_subscription(
            Empty, "shutdown_trigger", self.shutdown_callback, 10
        )
        self.nodes_to_shutdown = ["shutdownable_node"]  # Add more node names as needed
        self.shutdown_initiated = False
        self.get_logger().info("Shutdown node initialized")

    def shutdown_callback(self, msg):
        if self.shutdown_initiated:
            return
        self.shutdown_initiated = True
        self.get_logger().info("Received shutdown trigger. Shutting down all nodes...")
        self.shutdown_all_nodes()
        self.get_logger().info(
            "All nodes have been shut down. Shutting down Shutdown node..."
        )
        self.destroy_node()
        try:
            rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"Error during rclpy shutdown: {str(e)}")
        os._exit(0)  # Force exit the process

    def shutdown_all_nodes(self):
        self.shutdown_ros2_nodes()
        self.shutdown_rtabmap_processes()
        time.sleep(2)
        self.kill_remaining_processes()

    def shutdown_ros2_nodes(self):
        result = subprocess.run(
            ["ros2", "node", "list"], capture_output=True, text=True
        )
        nodes = result.stdout.strip().split("\n")

        for node in nodes:
            if node.strip() and not node.endswith("Shutdown"):
                self.get_logger().info(f"Shutting down node: {node}")
                self.shutdown_node(node)
                time.sleep(1)

    def shutdown_node(self, node_name):
        try:
            for proc in psutil.process_iter(["pid", "cmdline", "name"]):
                if proc.info["cmdline"] and node_name in " ".join(proc.info["cmdline"]):
                    self.get_logger().info(f"Found PID for {node_name}: {proc.pid}")
                    os.kill(proc.pid, signal.SIGINT)
                    return
            self.get_logger().warn(f"Could not find PID for {node_name}")
        except Exception as e:
            self.get_logger().error(f"Error shutting down {node_name}: {str(e)}")

    def shutdown_rtabmap_processes(self):
        rtabmap_processes = [
            "stereo_odometry",
            "rtabmap",
            "rtabmapviz",
            "rgbd_odometry",
            "icp_odometry",
            "rtabmap_odom",
            "orbslam3_rgbd_loc",
        ]

        for process_name in rtabmap_processes:
            self.shutdown_process_by_name(process_name)

    def shutdown_process_by_name(self, process_name):
        try:
            for proc in psutil.process_iter(["pid", "name", "cmdline"]):
                if process_name in proc.info["name"] or process_name in " ".join(
                    proc.info["cmdline"]
                ):
                    self.get_logger().info(f"Found {process_name} PID: {proc.pid}")
                    for _ in range(3):
                        self.get_logger().info(f"Sending SIGINT to {process_name}")
                        os.kill(proc.pid, signal.SIGINT)
                        time.sleep(5)
                        if not psutil.pid_exists(proc.pid):
                            break
                    time.sleep(2)
                    if psutil.pid_exists(proc.pid):
                        self.get_logger().warn(
                            f"Force killing {process_name} PID: {proc.pid}"
                        )
                        os.kill(proc.pid, signal.SIGKILL)
            self.get_logger().info(f"Finished shutting down {process_name} processes")
        except Exception as e:
            self.get_logger().error(f"Error shutting down {process_name}: {str(e)}")

    def kill_remaining_processes(self):
        for proc in psutil.process_iter(["pid", "name", "cmdline"]):
            try:
                if proc.info["cmdline"] and (
                    "ros2 run" in " ".join(proc.info["cmdline"])
                    or "_ros2_daemon" in proc.info["name"]
                    or "rtabmap" in proc.info["name"]
                    or (
                        "python3" in proc.info["cmdline"][0]
                        and any("rtabmap" in arg for arg in proc.info["cmdline"])
                    )
                ):
                    if proc.pid != os.getpid():
                        self.get_logger().warn(
                            f'Forcefully killing process: {proc.info["name"]} (PID: {proc.pid})'
                        )
                        proc.kill()
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass


def main(args=None):
    rclpy.init(args=args)
    shutdown_node = ShutdownNode()
    try:
        rclpy.spin(shutdown_node)
    except KeyboardInterrupt:
        pass
    finally:
        if not shutdown_node.shutdown_initiated:
            shutdown_node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception as e:
                print(f"Error during rclpy shutdown: {str(e)}")
        os._exit(0)  # Force exit the process


if __name__ == "__main__":
    main()
