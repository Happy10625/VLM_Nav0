#!/usr/bin/env python3
"""Small read-only dashboard: incremental experiment logs plus chassis telemetry."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import sys
import time


class ExperimentView:
    def __init__(self):
        self.directory = None
        self.latest, self.result, self.motion = {}, {}, {}
        self.last_stage = '-'
        self.path_count, self.path_state = 0, '未收到'
        self.path_at = self.path_wall_at = None
        self.finished = False
        self.failure_reason = ''
        self.failure_detail, self.vlm = '', '-'
        self.event_reason = ''
        self.offset, self.errors = 0, 0
        self.identity = self.result_identity = None
        self.io_error = ''

    def poll(self, directory):
        if directory != self.directory:
            self.__init__()
            self.directory = directory
        if directory is None:
            return
        self.io_error = ''
        try:
            path = directory / 'events.jsonl'
            stat = path.stat()
            identity = (stat.st_dev, stat.st_ino)
            if identity != self.identity or stat.st_size < self.offset:
                self.__init__()
                self.directory, self.identity = directory, identity
            with path.open('rb') as stream:
                stream.seek(self.offset)
                while True:
                    line = stream.readline()
                    if not line.endswith(b'\n'):
                        break
                    self.offset += len(line)
                    try:
                        row = json.loads(line)
                        if not isinstance(row, dict):
                            raise ValueError('event must be an object')
                        self.apply(row)
                    except (ValueError, UnicodeError):
                        self.errors += 1
        except FileNotFoundError:
            pass
        except OSError:
            self.io_error = '事件日志不可读'
        try:
            path = directory / 'result.json'
            stat = path.stat()
            identity = (stat.st_ino, stat.st_size, stat.st_mtime_ns)
            if identity != self.result_identity:
                value = json.loads(path.read_text())
                if not isinstance(value, dict):
                    raise ValueError('result must be an object')
                self.result, self.result_identity = value, identity
        except FileNotFoundError:
            self.result, self.result_identity = {}, None
        except (OSError, ValueError):
            self.result = {}
            self.io_error = '结果写入中或不可读'
        if self.motion.get('status') == 'ABORTED' and self.motion.get('log_attempts', 0) < 3 and not self.motion.get('log_reason'):
            self.motion['log_attempts'] = self.motion.get('log_attempts', 0) + 1
            self.motion['log_reason'] = spin_log_reason(self.motion, os.environ.get('ROS_LOG_DIR'))

    def apply(self, row):
        self.latest.update({k: row[k] for k in ('state', 'event', 'viewpoint_id', 'target_id',
                                               'target', 'reason', 'wall_time') if k in row})
        state, event = row.get('state'), row.get('event')
        if row.get('failure_reason') or event == 'SCAN_FAILED':
            self.failure_detail = row.get('failure_reason') or row.get('reason', '')
        if row.get('reason') and event not in ('PATH_CLEARED', 'STOP_COMMAND_SENT', 'STOP_OBSERVATION', 'SESSION_END') and state not in ('STOPPING', 'STOPPED'):
            self.event_reason = f'{event or state or "说明"}/{row["reason"]}'
        if state in ('STOPPING', 'REJECTED', 'HEALTH_FAILED') and not event and row.get('reason'):
            self.failure_reason = row['reason']
        if state in ('SELECTED', 'PLANNED', 'NAVIGATING', 'SCANNING', 'SCAN_RESULT', 'APPROACHING'):
            self.last_stage = state
            if not row.get('reason'):
                self.event_reason = ''
        if event == 'MOTION_REQUESTED':
            self.motion = dict(id=row.get('goal_id'), kind=row.get('goal_kind', '-'), status='等待接受',
                               start=row.get('wall_time'))
        elif row.get('goal_id') == self.motion.get('id'):
            status = {'MOTION_ACCEPTED': '已接受', 'CANCEL_REQUESTED': '取消中',
                      'MOTION_TERMINAL': row.get('status', '已结束')}.get(event)
            if status:
                self.motion['status'] = status
                if event == 'MOTION_TERMINAL':
                    self.motion.update(end=row.get('wall_time'), reason=row.get('reason', ''),
                                       error_code=row.get('error_code'), error_msg=row.get('error_msg', ''))
        if event == 'VLM_VIEW_RESULT':
            statuses = sorted({o.get('status', '-') for o in row.get('observations', [])})
            self.vlm = f'视角{row.get("view_index", 0)+1}/' + (row.get('error_type') or ','.join(statuses) or row.get('consumer_status', '-'))
        if event in ('PATH_RECEIVED', 'PATH_BUFFERED'):
            self.path_count = row.get('poses', 0)
            self.path_state = '已接收' if event == 'PATH_RECEIVED' else '暂存待接受'
            self.path_at, self.path_wall_at = row.get('path_received_monotonic'), row.get('wall_time')
        elif event == 'PATH_CLEARED':
            self.path_count, self.path_state = 0, '已清空'
            self.path_at = self.path_wall_at = None
        elif event == 'PATH_RECEIVE_TIMEOUT':
            self.path_state = '路径接收过期'
        if event == 'SESSION_END':
            self.finished = True
            self.failure_reason = row.get('reason') or self.failure_reason
            if self.motion.get('status') not in ('SUCCEEDED', 'CANCELED', 'ABORTED'):
                self.motion['status'] = '任务已结束'


def spin_log_reason(motion, log_dir):
    """Humble Spin has no error field; only correlate warnings inside this action."""
    start, end = motion.get('start'), motion.get('end')
    if not log_dir or motion.get('kind') != 'SCAN_SPIN' or start is None or end is None:
        return ''
    try:
        paths = sorted(Path(log_dir).glob('behavior_server_*.log'), key=lambda p: p.stat().st_mtime, reverse=True)[:8]
        matches = []
        for path in paths:
            with path.open('rb') as stream:
                # shortcut: cap log IO; use archived logs for details outside this tail.
                stream.seek(max(0, path.stat().st_size - 65536))
                for line in stream.read().decode(errors='replace').splitlines():
                    found = re.search(r'\[(?:WARN|ERROR)\]\s+\[([\d.]+)\]\s+\[behavior_server\]:\s*(.*)', line)
                    if found and start <= float(found[1]) <= end + .05:
                        matches.append((float(found[1]), found[2]))
        if matches:
            message = min(matches)[1]
            return ('旋转预测碰撞：' if 'Collision Ahead' in message else '') + message
    except (OSError, ValueError):
        pass
    return ''


def compact(value, limit=100):
    return ' '.join(str(value).split())[:limit]


def age(value):
    return f'{max(0., value):.2f}s' if isinstance(value, (float, int)) and math.isfinite(value) else '--'


def velocity(row):
    if not row:
        return '[--, --, --]'
    values = row.get('values', [])
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        return '非法速度'
    return '[' + ', '.join(f'{v:+.3f}' for v in values) + ']'


def render(view, telemetry, *, now, ros_now, can_delta, odom_max_age=.5, path_max_age=5.):
    row, result = view.latest, view.result
    odom, cmd = telemetry.get('odom', {}), telemetry.get('cmd', {})
    odom_age = max(now - odom['at'], abs(ros_now - odom['stamp'])) if odom else None
    cmd_age = now - cmd['at'] if cmd else None
    notes = []
    if not odom:
        notes.append('未收到底盘速度')
    elif odom_age > odom_max_age:
        notes.append('速度过期')
    if can_delta == 0:
        notes.append('CAN无接收增长，odom不能证明通信正常')
    elif can_delta is None:
        notes.append('CAN接收未采样')
    if view.errors:
        notes.append(f'日志坏行:{view.errors}')
    if view.io_error:
        notes.append(view.io_error)
    state = row.get('state', '等待实验/日志')
    target = compact(row.get('target', result.get('target', '-')), 24)
    if view.finished or result:
        motion_status = view.motion.get('status', '已结束')
    else:
        motion_status = view.motion.get('status', '无动作记录')
    path_age = now - view.path_at if view.path_at is not None else (
        abs(ros_now - view.path_wall_at) if view.path_wall_at is not None else None)
    path_state = view.path_state
    if view.path_count and path_age is not None and path_age > path_max_age:
        path_state = '路径接收过期'
    if view.motion.get('kind') == 'SCAN_SPIN' and not view.finished and not result:
        path_state = '旋转阶段无需导航路径'
    outcome = result.get('result', '已结束，等待结果文件' if view.finished else '运行中' if row else '等待实验')
    confirmed = {True: '是', False: '否'}.get(result.get('stop_confirmed'), '未记录')
    reason = view.failure_detail or result.get('failure_reason') or view.failure_reason or result.get('reason')
    reason = reason or ('；'.join(result.get('rejected', [])) if result.get('rejected') else '-')
    diagnostic = []
    if motion_status == 'ABORTED':
        detail = view.motion.get('error_msg') or view.motion.get('reason') or view.motion.get('log_reason')
        diagnostic.append('动作:' + compact(detail or '未取得具体原因', 110))
        if view.motion.get('error_code') is not None:
            diagnostic.append(f'错误码:{view.motion["error_code"]}')
    failed = [k for k, v in result.get('health_details', {}).get('checks', {}).items() if v is False]
    if failed:
        diagnostic.append('未就绪:' + ','.join(failed))
    if view.event_reason and not view.failure_detail:
        diagnostic.append('最近说明:' + compact(view.event_reason, 100))
    diagnostic.append('VLM:' + compact(view.vlm, 65))
    lines = [
        f'Ranger只读诊断 | 实验:{compact(view.directory.name if view.directory else "未选择", 55)} | 1Hz / Ctrl+C退出',
        f'任务:{state} | 最近阶段:{view.last_stage} | 目标:{row.get("target_id", result.get("target_id", "-"))}/{target} | 点:{compact(row.get("viewpoint_id", "-"), 35)}',
        f'动作:{view.motion.get("kind", "-")} / {motion_status} | 日志龄:{age(abs(ros_now-row["wall_time"]) if row.get("wall_time") else None)}',
        f'实测速度[vx,vy,wz]={velocity(odom)} | odom:{age(odom_age)} | CAN:{"+"+str(can_delta) if can_delta is not None else "--"}帧/刷新',
        f'指令速度[vx,vy,wz]={velocity(cmd)} | 接收龄:{age(cmd_age)} （单位:m/s,m/s,rad/s）',
        f'执行路径:{path_state} / {view.path_count}点 | 接收龄:{age(path_age)}',
        f'结果:{compact(outcome, 75)} | 任务停止确认:{confirmed} | 原因:{compact(reason, 100)}',
        '排障:' + ' | '.join(diagnostic),
        '提示:' + ('；'.join(notes) if notes else '只读观察；速度近零不代替实体停稳确认'),
    ]
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Ranger状态/动作/速度只读诊断终端，默认自动跟随当前实验。')
    parser.add_argument('--once', action='store_true', help='采样约2秒后仅显示一屏，替代打印完整result.json')
    parser.add_argument('--experiment-dir', type=Path, help='固定观察一个实验目录，不跟随当前实验指针')
    args = parser.parse_args(argv)
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    import yaml

    config = {}
    if os.environ.get('RANGER_CONFIG'):
        try:
            config = yaml.safe_load((Path(os.environ['RANGER_CONFIG']) / 'robot_ranger.yaml').read_text()) or {}
        except (OSError, ValueError, yaml.YAMLError):
            pass
    marker = Path(os.environ.get('ACCEPTANCE_RUN') or 'build/ranger-local') / 'current-single-experiment.txt'
    view, telemetry = ExperimentView(), {}
    rclpy.init()
    node = rclpy.create_node('ranger_diagnostic', enable_rosout=False, start_parameter_services=False)

    def receive(kind, message):
        v = message.twist.twist if kind == 'odom' else message
        row = dict(values=[v.linear.x, v.linear.y, v.angular.z], at=time.monotonic())
        if kind == 'odom':
            row['stamp'] = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        telemetry[kind] = row

    node.create_subscription(Odometry, config.get('odom_topic', '/odom'), lambda m: receive('odom', m), qos_profile_sensor_data)
    node.create_subscription(Twist, '/cmd_vel', lambda m: receive('cmd', m), qos_profile_sensor_data)

    def can_packets():
        try:
            return int(Path('/sys/class/net/can0/statistics/rx_packets').read_text())
        except (OSError, ValueError):
            return None

    last_can = can_packets()
    next_refresh = time.monotonic() + (2. if args.once else 0.)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if now < next_refresh:
                continue
            next_refresh = now + 1.
            directory = args.experiment_dir
            if directory is None:
                try:
                    value = marker.read_text().strip()
                    directory = Path(value) if value else None
                except OSError:
                    directory = None
            view.poll(directory)
            current_can = can_packets()
            delta = max(0, current_can-last_can) if current_can is not None and last_can is not None else None
            last_can = current_can
            screen = render(view, telemetry, now=now, ros_now=node.get_clock().now().nanoseconds*1e-9,
                            can_delta=delta, odom_max_age=config.get('odom_max_age', .5),
                            path_max_age=config.get('path_receive_timeout', 5.))
            if sys.stdout.isatty() and not args.once:
                sys.stdout.write('\033[2J\033[H')
            print(screen, flush=True)
            if args.once:
                return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
