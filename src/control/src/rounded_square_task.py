#!/usr/bin/env python3
"""ROS1 odometry closed-loop rounded-square controller with recording."""
from types import SimpleNamespace
import math
import signal
import threading
import time
import csv
import json
import os
from pathlib import Path
import subprocess
import rospy
from yhs_can_msgs.msg import ctrl_cmd
from nav_msgs.msg import Odometry


def plan(wheelbase, side, radius, speed, direction):
    for name, value in (("wheelbase", wheelbase), ("side", side),
                        ("radius", radius), ("speed", speed)):
        if not math.isfinite(value) or value <= 0:
            raise ValueError(name + " must be finite and positive")
    if speed > 2:
        raise ValueError("speed must be <= 2 m/s")
    if side <= 2 * radius:
        raise ValueError("side must exceed 2 * radius to retain straight segments")
    steering = math.degrees(math.atan(wheelbase / radius))
    if steering > 30 + 1e-9:
        raise ValueError("radius too small: minimum radius = {:.3f} m".format(
            wheelbase / math.tan(math.radians(30))))
    if direction not in ("left", "right"):
        raise ValueError("direction must be left or right")
    steering *= 1 if direction == "left" else -1
    straight = side - 2 * radius
    arc = math.pi * radius / 2
    segments = [("start_half_straight", straight / 2, 0.0)]
    for corner in range(1, 5):
        segments.append(("arc_{}".format(corner), arc, steering))
        if corner < 4:
            segments.append(("straight_{}".format(corner), straight, 0.0))
    segments.append(("finish_half_straight", straight / 2, 0.0))
    return [(name, distance, angle, distance / speed)
            for name, distance, angle in segments]


def self_test():
    segments = plan(0.5, 4, 1, 0.1, "left")
    x, y, heading = 0.0, 0.0, 0.0
    for _, distance, angle, _ in segments:
        curvature = math.tan(math.radians(angle)) / 0.5
        if abs(curvature) < 1e-10:
            x += distance * math.cos(heading)
            y += distance * math.sin(heading)
        else:
            new_heading = heading + distance * curvature
            x += (math.sin(new_heading) - math.sin(heading)) / curvature
            y += (math.cos(heading) - math.cos(new_heading)) / curvature
            heading = new_heading
    assert math.hypot(x, y) < 1e-9
    assert abs(heading - 2 * math.pi) < 1e-9
    assert len(segments) == 9
    right = plan(0.5, 4, 1, 0.1, "right")
    assert right[1][2] == -segments[1][2]
    for values in ((1, 4, 1, 0.1), (0.5, 2, 1, 0.1),
                   (0.5, 4, 1, 0), (0.5, 4, 1, 3),
                   (float("nan"), 4, 1, 0.1)):
        try:
            plan(*values, direction="left")
        except ValueError:
            pass
        else:
            raise AssertionError("invalid parameters accepted")
    print("Self-test passed: path closure, direction, parameter constraints.")


class MotionRecorder:
    """Raw feedback in rosbag; published commands in a separate CSV."""
    def __init__(self, args, segments):
        root = Path(rospy.get_param('~log_dir', '~/vehicle_motion_logs')).expanduser()
        self.directory = root / (time.strftime('%Y%m%d_%H%M%S') + '_' + str(os.getpid()))
        self.directory.mkdir(parents=True, exist_ok=False)
        self.process = None
        self.stream = None
        self.console = None
        self.started = time.monotonic()
        self.last_flush = self.started
        self.phase = 'startup'
        published = rospy.get_published_topics()
        feedback = []
        for topic, msg_type in published:
            if (msg_type.startswith('yhs_can_msgs/') or
                msg_type in ('nav_msgs/Odometry', 'sensor_msgs/Imu',
                             'sensor_msgs/NavSatFix', 'geometry_msgs/PoseStamped',
                             'geometry_msgs/PoseWithCovarianceStamped',
                             'geometry_msgs/TwistStamped',
                             'geometry_msgs/TwistWithCovarianceStamped') or
                topic in ('/tf', '/tf_static', '/clock')):
                feedback.append(topic)
        extra = rospy.get_param('~record_topics', [])
        if not isinstance(extra, list) or not all(isinstance(t, str) and t.startswith('/') for t in extra):
            raise ValueError('~record_topics must be a list of absolute topic names')
        topics = sorted(set(feedback + extra + [args.topic, args.odom_topic, '/rosout']))
        metadata = {'parameters': vars(args), 'segments': segments,
                    'published_topics_at_start': published, 'recorded_topics': topics,
                    'command_csv': 'Published requests, not measured motion',
                    'time': 'CSV wall time=Unix seconds; elapsed=monotonic seconds; ros time=ROS seconds',
                    'control': 'Odometry closed-loop Pure Pursuit'}
        (self.directory / 'run.json').write_text(json.dumps(metadata, indent=2, ensure_ascii=False), encoding='utf-8')
        try:
            self.stream = (self.directory / 'commands.csv').open('w', newline='', encoding='utf-8')
            self.writer = csv.writer(self.stream)
            self.writer.writerow(['wall_time_s', 'elapsed_s', 'ros_time_s', 'phase',
                                  'gear_cmd', 'velocity_cmd', 'steering_cmd_deg', 'brake_cmd'])
            self.stream.flush()
            self.console = (self.directory / 'rosbag_record.log').open('w', encoding='utf-8')
            self.process = subprocess.Popen(
                ['rosbag', 'record', '-O', str(self.directory / 'feedback.bag')] + topics,
                stdout=self.console, stderr=subprocess.STDOUT, start_new_session=True)
            time.sleep(1.0)
            if self.process.poll() is not None:
                raise RuntimeError('rosbag exited; see rosbag_record.log')
        except Exception:
            self.close()
            raise
        rospy.loginfo('Motion logs: %s', str(self.directory))
        rospy.loginfo('Recorded topics: %s', ', '.join(topics))
        if not any(typ in ('nav_msgs/Odometry', 'geometry_msgs/PoseStamped',
                           'geometry_msgs/PoseWithCovarianceStamped') for _, typ in published):
            rospy.logwarn('No odometry/pose topic found. Actual path cannot be reconstructed from commands alone; check TF/localization.')

    def record(self, command):
        if self.process.poll() is not None:
            raise RuntimeError('rosbag recorder stopped unexpectedly')
        now = time.monotonic()
        self.writer.writerow([time.time(), now - self.started, rospy.Time.now().to_sec(),
                              self.phase, command.ctrl_cmd_gear, command.ctrl_cmd_velocity,
                              command.ctrl_cmd_steering, command.ctrl_cmd_Brake])
        if now - self.last_flush >= 1:
            self.stream.flush()
            self.last_flush = now

    def close(self):
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()
                rospy.logwarn('rosbag did not exit cleanly; bag may require rosbag reindex')
        for stream in (self.stream, self.console):
            if stream is not None:
                stream.close()


def reference_path(args):
    x = y = yaw = distance = 0.0
    points = [(0.0, 0.0, 0.0, 0.0)]
    for _, length, angle, _ in plan(args.wheelbase, args.side, args.radius, args.speed, args.direction):
        k = math.tan(math.radians(angle)) / args.wheelbase
        count = math.ceil(length / 0.02)
        ds = length / count
        for _ in range(count):
            if abs(k) < 1e-10:
                x += ds * math.cos(yaw)
                y += ds * math.sin(yaw)
            else:
                nxt = yaw + ds * k
                x += (math.sin(nxt)-math.sin(yaw))/k
                y += (math.cos(yaw)-math.cos(nxt))/k
                yaw = nxt
            distance += ds
            points.append((distance, x, y, yaw))
    return points


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Tracker:
    def __init__(self, args):
        self.args = args
        self.path = load_path(args.path_file)
        self.index = 0

    def step(self, x, y, yaw):
        a = self.args
        upper = self.index
        # A local search avoids confusing the coincident start and end.
        while upper+1 < len(self.path) and self.path[upper+1][0]-self.path[self.index][0] <= 0.5:
            upper += 1
        self.index = min(range(self.index, upper+1), key=lambda i: math.hypot(x-self.path[i][1], y-self.path[i][2]))
        error = math.hypot(x-self.path[self.index][1], y-self.path[self.index][2])
        remaining = self.path[-1][0]-self.path[self.index][0]
        goal_distance = math.hypot(x-self.path[-1][1], y-self.path[-1][2])
        done = remaining < a.goal_tolerance and goal_distance < a.goal_tolerance and abs(wrap(yaw-self.path[-1][3])) < math.radians(10)
        if error > a.max_error:
            raise RuntimeError('Path deviation exceeds limit: %.3f m' % error)
        target = self.index
        while target+1 < len(self.path) and self.path[target][0]-self.path[self.index][0] < a.lookahead:
            target += 1
        dx, dy = self.path[target][1]-x, self.path[target][2]-y
        lateral = -math.sin(yaw)*dx + math.cos(yaw)*dy
        forward = math.cos(yaw)*dx + math.sin(yaw)*dy
        if forward < -0.05 and not done:
            raise RuntimeError('Target behind vehicle; check localization')
        curvature = 2*lateral/max(dx*dx+dy*dy, 0.0025)
        # Equivalent angle limit also keeps the inner Ackermann wheel <=30 deg.
        limit = math.degrees(math.atan(a.wheelbase/(a.track/2+a.wheelbase/math.tan(math.radians(30)))))
        angle = max(-limit, min(limit, math.degrees(math.atan(a.wheelbase*curvature))))
        speed = min(a.speed, math.sqrt(0.15/max(abs(curvature), 1e-6)), max(0.02, math.sqrt(0.2*max(remaining, goal_distance))))
        return (0.0 if done else speed), angle, error, remaining, target, done


class OdometryState:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.latest = None
        self.failure = None
        self.frame = None
        self.subscriber = rospy.Subscriber(args.odom_topic, Odometry, self.callback, queue_size=1)

    def callback(self, msg):
        try:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            if not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w,msg.twist.twist.linear.x,msg.twist.twist.angular.z)):
                raise ValueError('Non-finite odometry')
            norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
            if not 0.9 <= norm <= 1.1:
                raise ValueError('Invalid quaternion')
            qx,qy,qz,qw = q.x/norm,q.y/norm,q.z/norm,q.w/norm
            yaw = math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
            # Select based on the quaternion convention, not a displayed heading.
            if self.args.yaw_convention == 'north_cw':
                yaw = wrap(math.pi/2-yaw)
            elif self.args.yaw_convention == 'north_ccw':
                yaw = wrap(math.pi/2+yaw)
            stamp = msg.header.stamp.to_sec()
            age = rospy.Time.now().to_sec()-stamp
            if stamp <= 0 or age > self.args.odom_timeout or age < -0.1:
                raise ValueError('Stale or unsynchronized odometry timestamp')
            frame = (msg.header.frame_id, msg.child_frame_id)
            if not all(frame):
                raise ValueError('Missing odometry frame IDs')
            now = time.monotonic()
            x = p.x+self.args.rear_offset*math.cos(yaw)
            y = p.y+self.args.rear_offset*math.sin(yaw)
            with self.lock:
                if self.frame is not None and self.frame != frame:
                    raise ValueError('Odometry frame changed')
                if self.latest:
                    old = self.latest
                    dt = now-old[0]
                    if math.hypot(x-old[1],y-old[2]) > 2*dt+0.15 or abs(wrap(yaw-old[3])) > 2*dt+0.25:
                        raise ValueError('Odometry pose jump')
                self.frame = frame
                self.latest = (now,x,y,yaw,msg.twist.twist.linear.x,msg.twist.twist.angular.z,stamp)
        except Exception as exc:
            with self.lock:
                self.failure = str(exc)

    def get(self):
        with self.lock:
            if self.failure:
                raise RuntimeError('Odometry error: '+self.failure)
            value = self.latest
        if value and time.monotonic()-value[0] > self.args.odom_timeout:
            raise RuntimeError('Odometry feedback timed out')
        return value


def execute(args, segments, recorder):
    cancelled = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: cancelled.set())
    rospy.on_shutdown(cancelled.set)
    publisher = rospy.Publisher(args.topic, ctrl_cmd, queue_size=1, latch=False)
    command = ctrl_cmd()
    command.ctrl_cmd_gear = 4
    command.ctrl_cmd_velocity = 0.0
    command.ctrl_cmd_steering = 0.0
    command.ctrl_cmd_Brake = args.stop_brake
    state = OdometryState(args)
    tracker = Tracker(args)
    tracking = (recorder.directory/'tracking.csv').open('w',newline='',encoding='utf-8')
    writer = csv.writer(tracking)
    writer.writerow(['wall_time_s','elapsed_s','odom_stamp_s','x_local_m','y_local_m','yaw_local_rad','measured_vx','measured_yaw_rate','nearest_error_m','remaining_m','target_index','velocity_cmd','steering_cmd_deg'])
    with (recorder.directory/'reference_path.csv').open('w',newline='',encoding='utf-8') as stream:
        output = csv.writer(stream)
        output.writerow(['s_m','x_m','y_m','yaw_rad'])
        output.writerows(tracker.path)
    try:
        deadline = time.monotonic()+10
        origin = None
        while not cancelled.is_set() and not rospy.is_shutdown():
            sample = state.get()
            if sample and publisher.get_num_connections()>0:
                origin = sample
                break
            if time.monotonic()>deadline:
                raise RuntimeError('No valid odometry or control subscriber within 10 s; no motion started')
            publisher.publish(command)
            recorder.record(command)
            cancelled.wait(0.01)
        if origin is None:
            return
        ox,oy,oyaw = origin[1:4]
        (recorder.directory/'origin.json').write_text(json.dumps({'x':ox,'y':oy,'yaw':oyaw,'frames':state.frame}),encoding='utf-8')
        started = last_progress = last_flush = time.monotonic()
        previous_index = 0
        tick = started
        recorder.phase = 'closed_loop'
        while not cancelled.is_set() and not rospy.is_shutdown():
            _,gx,gy,gyaw,vx,wz,stamp = state.get()
            dx,dy = gx-ox,gy-oy
            x = math.cos(oyaw)*dx+math.sin(oyaw)*dy
            y = -math.sin(oyaw)*dx+math.cos(oyaw)*dy
            yaw = wrap(gyaw-oyaw)
            speed,angle,error,remaining,target,done = tracker.step(x,y,yaw)
            now = time.monotonic()
            if tracker.index > previous_index:
                last_progress = now
                previous_index = tracker.index
            if now-last_progress > 10:
                raise RuntimeError('No path progress for 10 s')
            if now-started > 3*sum(s[3] for s in segments)+30:
                raise RuntimeError('Task duration exceeded limit')
            if publisher.get_num_connections()==0:
                raise RuntimeError('Control subscriber disconnected')
            command.ctrl_cmd_velocity = speed
            command.ctrl_cmd_steering = angle
            command.ctrl_cmd_Brake = 0
            publisher.publish(command)
            recorder.record(command)
            writer.writerow([time.time(),now-started,stamp,x,y,yaw,vx,wz,error,remaining,target,speed,angle])
            if now-last_flush >= 1:
                tracking.flush()
                last_flush = now
            rospy.loginfo_throttle(1,'Tracking: x=%.3f y=%.3f error=%.3f m remaining=%.2f m',x,y,error,remaining)
            if done:
                rospy.loginfo('Goal reached within %.3f m and 10 degree heading tolerance',args.goal_tolerance)
                break
            tick += 0.01
            if tick<now: tick=now
            cancelled.wait(max(0,tick-time.monotonic()))
    finally:
        recorder.phase = 'stop'
        command.ctrl_cmd_velocity = 0.0
        command.ctrl_cmd_steering = 0.0
        command.ctrl_cmd_Brake = args.stop_brake
        for _ in range(100):
            try: publisher.publish(command)
            except rospy.ROSException: break
            try: recorder.record(command)
            except Exception: pass
            time.sleep(0.01)
        state.subscriber.unregister()
        tracking.close()


def load_path(filename):
    points = []
    with Path(filename).expanduser().open(newline='', encoding='utf-8-sig') as stream:
        for row in csv.DictReader(stream):
            point = tuple(float(row[k]) for k in ('s_m','x_m','y_m','yaw_rad'))
            if not all(math.isfinite(v) for v in point):
                raise ValueError('Non-finite reference path point')
            if points:
                ds = point[0]-points[-1][0]
                if ds <= 0 or ds > 0.1:
                    raise ValueError('Reference s_m must increase with spacing <=0.1 m')
                chord = math.hypot(point[1]-points[-1][1],point[2]-points[-1][2])
                if abs(chord-ds) > max(0.005, ds*0.1):
                    raise ValueError('Path distance does not match coordinates')
            points.append(point)
    if len(points)<3:
        raise ValueError('Reference path requires at least three points')
    if abs(points[0][0])>1e-6 or math.hypot(points[0][1],points[0][2])>1e-6 or abs(wrap(points[0][3]))>1e-6:
        raise ValueError('Use a local path starting at s=0, x=0, y=0, yaw=0')
    return points


def main():
    rospy.init_node("rounded_square_task", anonymous=False, disable_signals=True)
    args = SimpleNamespace()
    args.wheelbase = rospy.get_param("~wheelbase_m", 0.66)
    args.side = rospy.get_param("~side_m", 4.0)
    args.radius = rospy.get_param("~radius_m", 1.6)
    args.speed = rospy.get_param("~speed_mps", 0.1)
    args.direction = rospy.get_param("~direction", "left")
    args.topic = rospy.get_param("~topic", "/ctrl_cmd")
    args.stop_brake = rospy.get_param("~stop_brake", 0)
    args.path_file = rospy.get_param('~path_file', str(Path(__file__).resolve().with_name('rounded_square_path.csv')))
    args.odom_topic = rospy.get_param('~odom_topic', '/AIMhc_UGV1/odom')
    args.yaw_convention = rospy.get_param('~yaw_convention', 'unconfirmed')
    if args.yaw_convention not in ('ros_enu', 'north_cw', 'north_ccw'):
        raise ValueError('Confirm odometry quaternion convention: set ~yaw_convention to ros_enu (east zero, CCW), north_cw, or north_ccw')
    args.rear_offset = rospy.get_param('~rear_axle_offset_m', 0.0)
    args.track = rospy.get_param('~track_m', 0.665)
    args.lookahead = rospy.get_param('~lookahead_m', 0.25)
    args.odom_timeout = rospy.get_param('~odom_timeout_s', 0.5)
    args.goal_tolerance = rospy.get_param('~goal_tolerance_m', 0.08)
    args.max_error = rospy.get_param('~max_error_m', 0.5)
    for name in ('track','lookahead','odom_timeout','goal_tolerance','max_error'):
        value = getattr(args,name)
        if not isinstance(value,(int,float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(name+' must be finite and positive')
    if not math.isfinite(args.rear_offset):
        raise ValueError('Invalid rear axle offset')
    if args.wheelbase is None:
        rospy.logfatal("Set ~wheelbase_m to measured front/rear axle distance; vehicle length is not wheelbase")
        return
    if args.stop_brake < 0 or args.stop_brake > 255:
        rospy.logfatal("~stop_brake must be between 0 and 255; verify chassis range")
        return
    try:
        segments = plan(args.wheelbase, args.side, args.radius, args.speed, args.direction)
        if args.radius < args.track/2+args.wheelbase/math.tan(math.radians(30)):
            raise ValueError('Radius violates inner front wheel 30 degree constraint')
    except (ValueError, TypeError) as exc:
        rospy.logfatal("Invalid path parameters: %s", str(exc))
        raise SystemExit(1)
    points = load_path(args.path_file)
    max_curvature = 1/(args.track/2+args.wheelbase/math.tan(math.radians(30)))
    for previous, current in zip(points, points[1:]):
        if abs(wrap(current[3]-previous[3]))/(current[0]-previous[0]) > max_curvature+0.01:
            raise ValueError('Reference path curvature exceeds chassis limit')
    # Only used for a generous overall timeout, never for steering phases.
    segments = [('external_reference',points[-1][0],0,points[-1][0]/args.speed)]
    rospy.loginfo('Following CSV reference: %s, speed %.2f m/s',args.path_file,args.speed)
    recorder = MotionRecorder(args, segments)
    try:
        execute(args, segments, recorder)
    finally:
        recorder.close()
        rospy.loginfo('Motion logs saved to %s', str(recorder.directory))


if __name__ == "__main__":
    main()

