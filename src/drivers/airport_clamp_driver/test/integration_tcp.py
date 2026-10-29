#!/usr/bin/env python3
"""Isolated ROS master + mock STM32; never connects to actuator hardware."""
import math
import os
from pathlib import Path
import socket
import subprocess
import threading
import time
import xmlrpc.client

MASTER = "http://127.0.0.1:11339"
os.environ["ROS_MASTER_URI"] = MASTER
os.environ["ROS_HOSTNAME"] = "127.0.0.1"
os.environ.pop("ROS_IP", None)

import rospy
from std_msgs.msg import Bool, Float32MultiArray, UInt8
from std_srvs.srv import Trigger
from airport_clamp_driver.srv import SetEndpoints


def wait_for(predicate, timeout=6.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("Condition timed out")


class MockSTM32:
    def __init__(self):
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.port = self.listener.getsockname()[1]
        self.listener.listen(2)
        self.listener.settimeout(0.1)
        self.stop = threading.Event()
        self.log = []
        self.client = None
        self.pause_until = 0
        self.invalid = False
        self.reject_grip = False
        self.reject_endpoints_axis = None
        self.wrong_readback = False
        self.endpoints = {1: (300.0, 100.0), 2: (300.0, 100.0)}
        self.axis_states = {1: 0, 2: 0}
        self.targets = {1: 100.0, 2: 100.0}
        self.distances = {1: 300.0, 2: 300.0}
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def disconnect(self):
        self.pause_until = time.monotonic() + 1.0
        if self.client:
            try:
                self.client.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def run(self):
        while not self.stop.is_set():
            if time.monotonic() < self.pause_until:
                time.sleep(0.02)
                continue
            try:
                client, _ = self.listener.accept()
            except socket.timeout:
                continue
            self.client = client
            client.settimeout(0.1)
            data = b""
            try:
                while not self.stop.is_set():
                    try:
                        chunk = client.recv(512)
                    except socket.timeout:
                        continue
                    if not chunk:
                        break
                    data += chunk
                    while b"\n" in data:
                        raw, _, data = data.partition(b"\n")
                        command = raw.decode().strip()
                        self.log.append(command)
                        verb = command.split()[0]
                        if verb == "ENDPOINTS":
                            fields = command.split()
                            axis = int(fields[1])
                            if axis == self.reject_endpoints_axis:
                                response = "ERR ENDPOINTS\r\n"
                            else:
                                self.endpoints[axis] = tuple(float(v) for v in fields[2:])
                                self.targets[axis] = self.endpoints[axis][1]
                                response = "OK ENDPOINTS\r\n"
                        elif verb == "STATUS":
                            response = "".join(
                                "AXIS {} distance_mm={} valid={} state={} start_mm={} end_mm={} target_mm={} fault=NONE\r\n".format(
                                    i, self.distances[i], int(not self.invalid), self.axis_states[i],
                                    self.endpoints[i][0], self.endpoints[i][1] + int(self.wrong_readback),
                                    self.targets[i])
                                for i in (1, 2)) + "OK STATUS\r\n"
                        elif verb in ("GRIP", "RELEASE"):
                            target_index = 1 if verb == "GRIP" else 0
                            rejected = verb == "GRIP" and self.reject_grip
                            if not rejected:
                                self.axis_states = {1: 2, 2: 2}
                                self.targets = {i: self.endpoints[i][target_index] for i in (1, 2)}
                            response = ("ERR " if rejected else "OK ") + verb + "\r\n"
                        elif verb == "STOP":
                            self.axis_states = {1: 0, 2: 0}
                            response = "OK STOP\r\n"
                        else:
                            response = ("ERR " if verb == "GRIP" and self.reject_grip else "OK ") + verb + "\r\n"
                        # Deliberately split ACKs/status across TCP reads.
                        encoded = response.encode()
                        client.sendall(encoded[:3])
                        time.sleep(0.002)
                        client.sendall(encoded[3:])
            except OSError:
                pass
            finally:
                client.close()
                self.client = None

    def reach(self, locked):
        index = 1 if locked else 0
        self.axis_states = {1: 0, 2: 0}
        self.distances = {i: self.endpoints[i][index] for i in (1, 2)}

    def close(self):
        self.stop.set()
        self.disconnect()
        self.thread.join(2)
        self.listener.close()


def main():
    master = node = None
    mock = MockSTM32()
    directory = Path("/tmp/airport_clamp_test")
    directory.mkdir(exist_ok=True)
    master_log = (directory / "master.log").open("w")
    node_log = (directory / "node.log").open("w")
    try:
        master = subprocess.Popen(["roscore", "-p", "11339"], stdout=master_log, stderr=subprocess.STDOUT)
        def master_ready():
            try:
                return xmlrpc.client.ServerProxy(MASTER).getPid("/clamp_test")[0] == 1
            except OSError:
                return False
        wait_for(master_ready)
        rospy.init_node("clamp_integration_test", anonymous=True, disable_signals=True)
        connected, distances = [False], [[]]
        rospy.Subscriber("/clamp_connected", Bool, lambda msg: connected.__setitem__(0, msg.data))
        rospy.Subscriber("/clamp_distance_mm", Float32MultiArray, lambda msg: distances.__setitem__(0, msg.data))
        pub = rospy.Publisher("/clamp_command", UInt8, queue_size=1)
        node = subprocess.Popen([
            "rosrun", "airport_clamp_driver", "airport_clamp_node.py", "__name:=airport_clamp_test",
            "_host:=127.0.0.1", "_bind_ip:=127.0.0.1", "_port:=" + str(mock.port),
            "_axis1_release_mm:=270.0", "_axis1_grip_mm:=125.0",
            "_axis2_release_mm:=272.0", "_axis2_grip_mm:=126.0",
            "_status_period:=0.1", "_request_timeout:=0.2", "_reconnect_delay:=0.1"],
            stdout=node_log, stderr=subprocess.STDOUT)
        wait_for(lambda: connected[0] and pub.get_num_connections() > 0)
        assert mock.endpoints == {1: (270.0, 125.0), 2: (272.0, 126.0)}
        assert mock.log[:4] == ["STOP", "ENDPOINTS 1 270.0 125.0", "ENDPOINTS 2 272.0 126.0", "STATUS"]
        wait_for(lambda: len(distances[0]) == 2 and abs(distances[0][1] - 300.0) < 0.01)
        pub.publish(1)
        time.sleep(0.15)
        assert mock.log.count("GRIP") == 1, "Repeated GRIP restarted the active goal"
        for _ in range(8):
            pub.publish(1)
            time.sleep(0.03)
        time.sleep(0.2)
        assert mock.log.count("GRIP") == 1, "Duplicate restarted GRIP"
        releases_before_invalid = mock.log.count("RELEASE")
        pub.publish(2)
        time.sleep(0.15)
        assert mock.log.count("RELEASE") == releases_before_invalid, "Invalid UInt8 became RELEASE"
        pub.publish(0)
        wait_for(lambda: mock.log.count("RELEASE") == 1)
        service = rospy.ServiceProxy("/airport_clamp_test/set_endpoints", SetEndpoints)
        assert service(1, 280.0, 140.0).success
        assert "ENDPOINTS 1 280.0 140.0" in mock.log
        assert rospy.get_param("/airport_clamp_test/axis1_release_mm") == 280.0
        assert rospy.get_param("/airport_clamp_test/axis1_grip_mm") == 140.0
        assert not service(0, 275.0, 130.0).success
        assert not service(1, 130.0, 275.0).success
        assert not service(1, float("nan"), 130.0).success
        stop = rospy.ServiceProxy("/airport_clamp_test/stop", Trigger)
        clear = rospy.ServiceProxy("/airport_clamp_test/clear", Trigger)
        assert stop().success and clear().success
        mock.invalid = True
        wait_for(lambda: len(distances[0]) == 2 and all(math.isnan(v) for v in distances[0]))
        mock.invalid = False
        wait_for(lambda: len(distances[0]) == 2 and all(math.isfinite(v) for v in distances[0]))
        mock.reject_grip = True
        pub.publish(1)
        wait_for(lambda: mock.log.count("GRIP") == 2)
        time.sleep(0.15)
        mock.reject_grip = False
        pub.publish(1)
        wait_for(lambda: mock.log.count("GRIP") == 3)
        mock.disconnect()
        wait_for(lambda: not connected[0])
        pub.publish(0)
        assert not stop().success
        wait_for(lambda: connected[0])
        assert mock.endpoints == {1: (280.0, 140.0), 2: (272.0, 126.0)}
        time.sleep(0.2)
        assert mock.log.count("GRIP") == 3 and mock.log.count("RELEASE") == 1, "Offline command replayed"
        pub.publish(0)
        wait_for(lambda: mock.log.count("RELEASE") == 2)
        mock.reject_endpoints_axis = 2
        mock.disconnect()
        wait_for(lambda: not connected[0])
        attempts = mock.log.count("ENDPOINTS 2 272.0 126.0")
        wait_for(lambda: mock.log.count("ENDPOINTS 2 272.0 126.0") > attempts)
        assert not connected[0], "Rejected configuration advertised ready"
        pub.publish(1)
        time.sleep(0.2)
        assert mock.log.count("GRIP") == 3, "Motion sent without applied thresholds"
        mock.reject_endpoints_axis = None
        mock.wrong_readback = True
        attempts = mock.log.count("STATUS")
        wait_for(lambda: mock.log.count("STATUS") > attempts)
        time.sleep(0.1)
        assert not connected[0], "Wrong endpoint readback advertised ready"
        mock.wrong_readback = False
        wait_for(lambda: connected[0])
        stops_before = mock.log.count("STOP")
        node.terminate()
        node.wait(timeout=5)
        assert mock.log.count("STOP") > stops_before, "Shutdown did not issue STOP"
        print("PASS: mapping, duplicate suppression, invalid command, fragmented TCP, endpoint service,")
        print("      STOP/CLEAR, invalid distances, ERR retry, disconnect/no replay, shutdown STOP")
        print("      host thresholds before motion, readback verification, parameter updates,")
        print("      reconnect reapplies thresholds, rejected/mismatched configuration blocks motion")
    finally:
        if node and node.poll() is None:
            node.terminate()
            node.wait(timeout=5)
        rospy.signal_shutdown("test complete")
        mock.close()
        if master:
            master.terminate()
            master.wait(timeout=5)
        master_log.close()
        node_log.close()


if __name__ == "__main__":
    main()
