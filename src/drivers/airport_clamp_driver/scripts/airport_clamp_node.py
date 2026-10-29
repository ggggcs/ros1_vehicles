#!/usr/bin/env python3
"""Single socket owner; ROS callbacks never block the transport worker."""
import math
import queue
import socket
import threading
import time

import rospy
from std_msgs.msg import Bool, Float32MultiArray, String, UInt8
from std_srvs.srv import Trigger, TriggerResponse
from airport_clamp_driver.srv import SetEndpoints, SetEndpointsResponse


class Transport:
    def __init__(self, host, port, bind_ip, timeout):
        self.host, self.port, self.bind_ip = host, port, bind_ip
        self.timeout = timeout
        self.sock = None
        self.buffer = bytearray()

    def connect(self):
        self.close()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        if self.bind_ip:
            self.sock.bind((self.bind_ip, 0))
        self.sock.connect((self.host, self.port))

    def close(self):
        if self.sock:
            self.sock.close()
        self.sock = None
        self.buffer.clear()

    def request(self, command):
        self.sock.settimeout(self.timeout)
        self.sock.sendall((command + "\r\n").encode("ascii"))
        deadline = time.monotonic() + self.timeout
        verb = command.split()[0]
        lines = []
        while True:
            while b"\n" in self.buffer:
                raw, _, rest = self.buffer.partition(b"\n")
                self.buffer = bytearray(rest)
                line = raw.rstrip(b"\r").decode("ascii", errors="strict")
                if not line:
                    continue
                lines.append(line)
                if line in ("OK " + verb, "ERR " + verb):
                    return line.startswith("OK "), lines
                if line.startswith("ERR "):
                    return False, lines
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("Missing acknowledgement for " + verb)
            self.sock.settimeout(remaining)
            data = self.sock.recv(1024)
            if not data:
                raise ConnectionError("STM32 closed TCP connection")
            self.buffer.extend(data)
            if len(self.buffer) > 4096 or len(lines) > 32:
                raise ValueError("STM32 response exceeds protocol limits")


class ServiceRequest:
    def __init__(self, command, endpoints=None):
        self.command = command
        self.endpoints = endpoints
        self.done = threading.Event()
        self.success, self.message = False, "Request cancelled"


class ClampNode:
    def __init__(self):
        timeout = float(rospy.get_param("~request_timeout", 1.0))
        self.period = float(rospy.get_param("~status_period", 0.2))
        self.reconnect = float(rospy.get_param("~reconnect_delay", 1.0))
        self.endpoints = {}
        for axis in (1, 2):
            self.endpoints[axis] = self.validate_endpoints(
                rospy.get_param("~axis{}_release_mm".format(axis), 275.0),
                rospy.get_param("~axis{}_grip_mm".format(axis), 130.0))
        if not (0.05 <= self.period <= 1.0 and 0.1 <= timeout <= 1.0 and self.reconnect >= 0.1):
            raise ValueError("Invalid timing: status 0.05..1s, timeout 0.1..1s, reconnect >=0.1s")
        self.transport = Transport(rospy.get_param("~host", "192.168.8.103"),
                                   int(rospy.get_param("~port", 5000)),
                                   rospy.get_param("~bind_ip", "192.168.8.102"), timeout)
        self.lock = threading.Lock()
        self.connected = False
        self.pending = None
        self.last_command = None
        self.motion_goal = None
        self.confirmed_state = None
        self.state_pub = rospy.Publisher("/clamp_state", UInt8, queue_size=1, latch=True)
        self.report_state(255)
        self.services = queue.Queue(maxsize=8)
        self.stop_event = threading.Event()
        self.status_pub = rospy.Publisher("/clamp_status", String, queue_size=10)
        self.distance_pub = rospy.Publisher("/clamp_distance_mm", Float32MultiArray, queue_size=1)
        self.connected_pub = rospy.Publisher("/clamp_connected", Bool, queue_size=1, latch=True)
        self.connected_pub.publish(False)
        rospy.Subscriber(rospy.get_param("~command_topic", "/clamp_command"),
                         UInt8, self.on_command, queue_size=1)
        rospy.Service("~stop", Trigger, self.on_stop)
        rospy.Service("~clear", Trigger, self.on_clear)
        rospy.Service("~set_endpoints", SetEndpoints, self.on_endpoints)
        self.worker = threading.Thread(target=self.run, name="clamp_tcp", daemon=True)
        self.worker.start()
        rospy.on_shutdown(self.shutdown)

    def on_command(self, msg):
        if msg.data not in (0, 1):
            rospy.logwarn("Rejected clamp command %s; expected 0 or 1", msg.data)
            return
        with self.lock:
            if not self.connected:
                rospy.logwarn_throttle(2.0, "Clamp disconnected; command discarded, publish again after reconnect")
                return
            # Continuously published identical commands must not restart a completed move.
            self.pending = "GRIP" if msg.data else "RELEASE"

    @staticmethod
    def validate_endpoints(release_mm, grip_mm):
        if isinstance(release_mm, bool) or isinstance(grip_mm, bool):
            raise ValueError("Endpoint distances must be numbers")
        release_mm, grip_mm = float(release_mm), float(grip_mm)
        if not all(math.isfinite(v) and 0 < v <= 100000 for v in (release_mm, grip_mm)):
            raise ValueError("Endpoint distances must be finite and within 0..100000 mm")
        # Match the sensors' 0.1 mm resolution and the values sent on the wire.
        release_mm, grip_mm = float("{:.1f}".format(release_mm)), float("{:.1f}".format(grip_mm))
        if grip_mm <= 0 or release_mm - grip_mm <= 1.0:
            raise ValueError("Release distance must exceed grip distance by more than 1 mm")
        return release_mm, grip_mm

    @staticmethod
    def endpoint_command(axis, endpoints):
        return "ENDPOINTS {} {:.1f} {:.1f}".format(axis, *endpoints)

    def service(self, command, endpoints=None):
        request = ServiceRequest(command, endpoints)
        with self.lock:
            if not self.connected:
                return False, "STM32 disconnected"
            if command == "STOP":
                self.pending = None
            try:
                self.services.put_nowait(request)
            except queue.Full:
                return False, "Command queue full"
        # Worker always completes or cancels queued requests on disconnect.
        while not request.done.wait(0.1):
            if self.stop_event.is_set() or rospy.is_shutdown():
                return False, "Node stopping"
        return request.success, request.message

    def on_stop(self, _):
        return TriggerResponse(*self.service("STOP"))

    def on_clear(self, _):
        return TriggerResponse(*self.service("CLEAR"))

    def on_endpoints(self, req):
        if req.axis not in (1, 2):
            return SetEndpointsResponse(False, "Axis must be 1 or 2")
        try:
            endpoints = self.validate_endpoints(req.start_mm, req.end_mm)
        except (TypeError, ValueError) as exc:
            return SetEndpointsResponse(False, str(exc))
        return SetEndpointsResponse(*self.service(
            self.endpoint_command(req.axis, endpoints), (req.axis, endpoints)))

    def publish_lines(self, lines):
        for line in lines:
            self.status_pub.publish(line)

    def report_state(self, state):
        if state not in (0, 1, 2, 3):
            state = 0x80
        self.state_pub.publish(UInt8(data=state))

    def update_state(self, axes):
        if any(a["valid"] != "1" or a["fault"] != "NONE" or a["state"] not in ("0", "2")
               for a in axes.values()):
            self.confirmed_state = None
            self.report_state(255)
            return
        if self.motion_goal is not None:
            goal = self.motion_goal
            endpoint_index = 1 if goal == "GRIP" else 0
            final_state = 2 if goal == "GRIP" else 0
            transition_state = 1 if goal == "GRIP" else 3
            reached = all(a["state"] == "0" and abs(float(a["target_mm"]) - self.endpoints[i][endpoint_index]) <= 0.05
                          for i, a in axes.items())
            if reached:
                self.confirmed_state = final_state
                self.report_state(self.confirmed_state)
            elif any(a["state"] == "2" for a in axes.values()):
                self.report_state(transition_state)
            else:
                self.confirmed_state = None
                self.report_state(255)
        elif all(a["state"] == "0" for a in axes.values()):
            # Boot/reconnect can recognize endpoints, but never infer motion direction.
            distances = {i: float(a["distance_mm"]) for i, a in axes.items()}
            if all(abs(distances[i] - self.endpoints[i][0]) <= 0.5 for i in axes):
                self.confirmed_state = 0
            elif all(abs(distances[i] - self.endpoints[i][1]) <= 0.5 for i in axes):
                self.confirmed_state = 2
            self.report_state(self.confirmed_state if self.confirmed_state is not None else 255)
        else:
            self.report_state(255)

    def status(self):
        ok, lines = self.transport.request("STATUS")
        if not ok:
            raise ValueError("STATUS rejected")
        distances = [float("nan"), float("nan")]
        axes = {}
        for line in lines:
            if not line.startswith("AXIS "):
                continue
            fields = line.split()
            axis = int(fields[1])
            if axis not in (1, 2) or axis in axes:
                raise ValueError("Malformed axis status")
            values = dict(field.split("=", 1) for field in fields[2:])
            axes[axis] = values
            if values["valid"] == "1":
                value = float(values["distance_mm"])
                if not math.isfinite(value) or value <= 0:
                    raise ValueError("Invalid sensor distance")
                distances[axis - 1] = value
            if values["state"] == "3":
                self.last_command = None
        if set(axes) != {1, 2}:
            raise ValueError("Missing axis status")
        self.publish_lines(lines)
        self.distance_pub.publish(Float32MultiArray(data=distances))
        self.update_state(axes)
        return axes

    def verify_endpoints(self, axes, expected):
        for axis, endpoints in expected.items():
            actual = float(axes[axis]["start_mm"]), float(axes[axis]["end_mm"])
            if not all(math.isfinite(v) for v in actual) or any(
                    abs(a - b) > 0.05 for a, b in zip(actual, endpoints)):
                raise ValueError("Axis {} endpoint readback mismatch: {} != {}".format(axis, actual, endpoints))

    def configure_endpoints(self):
        for axis, endpoints in self.endpoints.items():
            ok, lines = self.transport.request(self.endpoint_command(axis, endpoints))
            self.publish_lines(lines)
            if not ok:
                raise ValueError("STM32 rejected axis {} endpoint configuration".format(axis))
        self.verify_endpoints(self.status(), self.endpoints)
        rospy.loginfo("Clamp thresholds applied (release/grip mm): axis1=%s axis2=%s",
                      self.endpoints[1], self.endpoints[2])

    def cancel_services(self, message):
        while True:
            try:
                req = self.services.get_nowait()
            except queue.Empty:
                return
            req.message = message
            req.done.set()

    def run(self):
        active = None
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            try:
                self.transport.connect()
                self.motion_goal = self.confirmed_state = None
                self.report_state(255)
                ok, lines = self.transport.request("STOP")
                if not ok:
                    raise ConnectionError("Initial STOP rejected")
                self.publish_lines(lines)
                self.configure_endpoints()
                with self.lock:
                    self.pending = self.last_command = None
                    self.connected = True
                self.connected_pub.publish(True)
                rospy.loginfo("Clamp connected to %s:%s", self.transport.host, self.transport.port)
                next_status = time.monotonic() + self.period
                while not self.stop_event.is_set() and not rospy.is_shutdown():
                    if time.monotonic() >= next_status:
                        self.status()
                        next_status = time.monotonic() + self.period
                    try:
                        active = self.services.get_nowait()
                    except queue.Empty:
                        active = None
                    with self.lock:
                        motion, self.pending = self.pending, None
                    # Service has priority; retain the latest motion unless STOP requested.
                    if active:
                        if motion and active.command != "STOP":
                            with self.lock:
                                if self.pending is None:
                                    self.pending = motion
                        ok, lines = self.transport.request(active.command)
                        self.publish_lines(lines)
                        command = active.command
                        if ok and active.endpoints:
                            axis, endpoints = active.endpoints
                            self.verify_endpoints(self.status(), {axis: endpoints})
                            self.endpoints[axis] = endpoints
                            rospy.set_param("~axis{}_release_mm".format(axis), endpoints[0])
                            rospy.set_param("~axis{}_grip_mm".format(axis), endpoints[1])
                        if ok and (active.endpoints or command in ("STOP", "CLEAR")):
                            self.motion_goal = self.confirmed_state = None
                            self.report_state(255)
                        active.success, active.message = ok, "\n".join(lines)
                        active.done.set()
                        active = None
                        self.last_command = None
                    elif motion and motion != self.last_command:
                        ok, lines = self.transport.request(motion)
                        self.publish_lines(lines)
                        if ok:
                            self.last_command = motion
                            self.motion_goal = motion
                            self.confirmed_state = None
                            self.report_state(1 if motion == "GRIP" else 3)
                            self.status()
                        else:
                            self.motion_goal = self.confirmed_state = None
                            self.report_state(255)
                            rospy.logwarn("Clamp rejected %s: %s", motion, lines[-1])
                    self.stop_event.wait(0.01)
            except (OSError, ValueError, KeyError, UnicodeError) as exc:
                rospy.logwarn_throttle(5.0, "Clamp transport: %s", exc)
            finally:
                # Best effort orderly stop; firmware watchdog handles broken links.
                if self.transport.sock:
                    try:
                        self.transport.request("STOP")
                    except (OSError, ValueError):
                        pass
                self.transport.close()
                with self.lock:
                    self.connected = False
                    self.pending = self.last_command = None
                    self.cancel_services("Connection lost; command discarded")
                if active:
                    active.message = "Connection lost; outcome unknown, motion stopped"
                    active.done.set()
                    active = None
                self.connected_pub.publish(False)
                self.motion_goal = self.confirmed_state = None
                self.report_state(255)
                self.distance_pub.publish(Float32MultiArray(data=[float("nan"), float("nan")]))
            self.stop_event.wait(self.reconnect)

    def shutdown(self):
        self.stop_event.set()
        self.worker.join(timeout=3.0)


if __name__ == "__main__":
    rospy.init_node("airport_clamp")
    try:
        node = ClampNode()
        rospy.spin()
    except (ValueError, OSError) as exc:
        rospy.logfatal("Clamp startup failed: %s", exc)
        raise
