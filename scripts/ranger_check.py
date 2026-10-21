#!/usr/bin/env python3
"""Read-only Ranger checks. Raw diagnostics stay in files, never on the console."""
import argparse
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time

REPO = Path(__file__).resolve().parents[1]


def format_check(passed, message):
    return f"[{'通过' if passed else '失败'}] {message}"


def gate_text(reason):
    translations = (
        ('stop margin', '停车余量 stop_margin 尚未填写有效实测值'),
        ('camera_calibrated', '相机外参尚未完成现场确认'),
        ('costmap_source_verified', 'costmap 来源尚未完成现场确认'),
        ('viewpoint', '观察点尚未填写有效实测坐标'),
        ('target description', '目标描述尚未填写'),
        ('map_digest', '地图摘要与实际地图文件不一致或尚未填写'),
        ('readable map', '地图 YAML 或图像文件不可读'),
        ('map identity', '地图身份或版本尚未填写'),
        ('synthetic configuration', '当前仍为模拟配置'),
    )
    return next((text for key, text in translations if key in str(reason)), str(reason))


def configuration_checks(report, code):
    rows = [(code == 0, '配置检查程序' + ('正常结束' if code == 0 else '未通过；详见配置日志'))]
    valid = report.get('mode') == 'configuration_only' and report.get('robot_id') == 'ranger'
    rows.append((valid, 'Ranger 单机配置报告' + ('已取得' if valid else '缺失或格式异常')))
    info = report.get('map_files') or {}
    rows.append((bool(info.get('map_digest')) and not info.get('error'),
                 '地图文件可读及摘要已取得' if info.get('map_digest') else '地图文件或摘要缺失；检查 map_path'))
    for field, title in (('robot_local_gates', '本机配置'),
                         ('single_robot_experiment_gates', '单机实验配置')):
        reasons = report.get(field)
        if isinstance(reasons, list) and not reasons:
            rows.append((True, title + '完整'))
        elif isinstance(reasons, list):
            rows.extend((False, title + '：' + gate_text(reason)) for reason in reasons)
        else:
            rows.append((False, title + '检查结果缺失'))
    rows.append((report.get('motion_sent') is False, '配置检查未发送运动指令'))
    return rows


def age_text(value):
    return f'{value:.3f}s' if isinstance(value, (float, int)) and math.isfinite(value) else '未取得'


def inspection_checks(report, code):
    health = report.get('health') or {}
    details = report.get('health_details') or {}
    checks = details.get('checks') or {}
    window = report.get('health_window') or {}
    if not health:
        return [(False, f'在线检查未取得健康报告（退出码 {code}）；详见 inspect.log')]
    actual = report.get('active_map_yaml')
    expected = (report.get('map_files') or {}).get('yaml_path', '本轮配置的地图')
    if report.get('active_map_matches') is True:
        map_message = '当前 Nav2 地图与本轮配置一致'
    elif not actual:
        map_message = '地图参数未取得；检查 /map_server/get_parameters 服务响应，勿直接判定地图不同'
    else:
        map_message = f'地图不一致：Nav2={actual}，配置={expected}；检查 R2 地图路径'
    rows = []
    rows.extend([
        (report.get('active_map_matches') is True, map_message),
        (health.get('localization_valid') is True,
         '定位 TF map→base_link，时间龄 ' + age_text(details.get('pose_age_s'))
         + ('' if health.get('localization_valid') is True else '；检查初始位姿与 TF')),
        (report.get('rgbd_ready') is True,
         'RGB-D 同步及图像时刻 TF，图像时间龄 ' + age_text(details.get('rgbd_frame_age_s'))
         + ('' if report.get('rgbd_ready') is True else '；检查深度流与相机 TF')),
        (report.get('depth_units_verified') is True,
         '深度单位与编码已验证' if report.get('depth_units_verified') is True else '深度单位或编码尚未验证'),
        (checks.get('odom_fresh') is True,
         '底盘 ROS 里程计时间龄 ' + age_text(details.get('odom_age_s')) + '（真实通信另见 CAN 检查）'),
        (report.get('costmap_valid') is True,
         '局部 costmap 及坐标变换' + ('有效' if report.get('costmap_valid') is True
             else '无效：' + str((details.get('local_costmap') or {}).get('reason', '未取得')) + '；检查 Nav2 生命周期与 TF')),
    ])
    for key, title in (('navigate_server', '导航'), ('planner_server', '规划'), ('spin_server', '旋转')):
        rows.append((checks.get(key) is True, f'Nav2 {title} action 服务'
                     + ('可用' if checks.get(key) is True else '不可用；检查 R2 及 Nav2 生命周期')))
    rows.extend([
        (checks.get('motion_fault_clear') is True, '本检查进程无运动故障'),
        (health.get('stopped') is True,
         '软件停稳判据满足（仍需真实 CAN 反馈及现场确认）' if health.get('stopped') is True else '软件停稳判据未满足；先确认实体停止及速度反馈'),
        (health.get('motion_terminal') is True,
         '本检查进程没有未结束动作' if health.get('motion_terminal') is True else '无法确认本检查进程动作均已结束'),
        (window.get('non_motion_inputs_stable') is True,
         '非运动输入稳定窗口 ' + age_text(window.get('post_input_ready_observed_s'))
         + ('' if window.get('non_motion_inputs_stable') is True else '；检查输入中断或适当延长 --timeout')),
        (report.get('motion_sent') is False, '在线检查未发送运动指令'),
    ])
    if all(passed for passed, _ in rows) and (
            code != 0 or report.get('local_observation_ready') is not True or health.get('nav_ready') is not True):
        rows.append((False, f'其他在线就绪条件未满足（退出码 {code}）；检查任务占用及 inspect.log'))
    # A passive check has no executing goal, so an empty /plan is expected.
    return rows


def parse_report(raw):
    decoder = json.JSONDecoder()
    result = None
    for match in re.finditer(r'(?m)^\s*\{', raw):
        try:
            value, _ = decoder.raw_decode(raw[match.start():].lstrip())
        except ValueError:
            continue
        if isinstance(value, dict) and ('mode' in value or 'health' in value):
            result = value
    return result


def run_report(command, log, timeout):
    with log.open('w') as stream:
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            code = process.wait(timeout=timeout)
        except (subprocess.TimeoutExpired, KeyboardInterrupt) as error:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            if isinstance(error, KeyboardInterrupt):
                raise
            code = 124
    return parse_report(log.read_text(errors='replace')), code


def can_snapshot(interface):
    def query(command):
        result = subprocess.run(command, capture_output=True, text=True, timeout=3)
        return json.loads(result.stdout) if result.returncode == 0 else []
    try:
        links = query(['ip', '-j', '-details', '-statistics', 'link', 'show', 'dev', interface])
        if not links:
            return {}
        link = links[0]
        data = (link.get('linkinfo') or {}).get('info_data') or {}
        stats = link.get('stats64') or link.get('stats') or {}
        qdiscs = query(['tc', '-j', '-s', 'qdisc', 'show', 'dev', interface])
        return dict(up='UP' in link.get('flags', []), state=data.get('state'),
                    bitrate=(data.get('bittiming') or {}).get('bitrate'),
                    rx=(stats.get('rx') or {}).get('packets'),
                    tx=(stats.get('tx') or {}).get('packets'),
                    backlog=sum(q['qlen'] for q in qdiscs) if qdiscs and all('qlen' in q for q in qdiscs) else None,
                    dropped=sum(q['drops'] for q in qdiscs) if qdiscs and all('drops' in q for q in qdiscs) else None)
    except (OSError, ValueError, subprocess.TimeoutExpired):
        return {}


def can_checks(before, after):
    configured = after.get('up') is True and after.get('state') == 'ERROR-ACTIVE' and after.get('bitrate') == 500000
    rx = after.get('rx', 0) - before.get('rx', 0) if isinstance(after.get('rx'), int) and isinstance(before.get('rx'), int) else 0
    backlog = after.get('backlog') if isinstance(after.get('backlog'), int) else '未知'
    dropped = after.get('dropped') if isinstance(after.get('dropped'), int) else '未知'
    return [
        (configured, 'CAN 接口为 UP / ERROR-ACTIVE / 500000 bit/s' if configured
         else 'CAN 接口缺失或状态/波特率异常；检查 USB-CAN 及接口配置'),
        (configured and rx > 0,
         f'CAN 实际接收增加 {rx} 帧' if configured and rx > 0
         else 'CAN 未取得连续真实接收数据；检查底盘供电、CAN 接线及适配器状态，不能仅凭 /odom 放行'),
        (isinstance(after.get('backlog'), int) and after['backlog'] == 0,
         f'CAN 发送队列积压 {backlog} 帧，累计丢弃 {dropped} 帧'
         + ('；需恢复通信后复查，重启 ROS 不会清空接口队列' if after.get('backlog') else '')),
    ]


@contextmanager
def quiet_native_output(log):
    """ROS C/C++ logging also writes to fd 1/2, outside Python redirects."""
    sys.stdout.flush()
    sys.stderr.flush()
    saved = [os.dup(fd) for fd in (1, 2)]
    with log.open('w') as stream:
        try:
            for fd in (1, 2):
                os.dup2(stream.fileno(), fd)
            yield
        finally:
            sys.stdout.flush()
            sys.stderr.flush()
            for fd, original in zip((1, 2), saved):
                os.dup2(original, fd)
                os.close(original)


def hardware_probe(duration):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, Imu
    from nav_msgs.msg import Odometry
    from livox_ros_driver2.msg import CustomMsg

    topics = {
        '/camera/color/image_raw': (Image, '相机彩色图像', 1., 0.),
        '/camera/aligned_depth_to_color/image_raw': (Image, '相机对齐深度', 1., 0.),
        '/livox/lidar': (CustomMsg, 'MID-360 点云', 1., 0.),
        '/livox/imu': (Imu, 'MID-360 IMU', 1., 0.),
        '/Odometry': (Odometry, 'FAST-LIO 原始里程计', .5, 2.),
        '/odom': (Odometry, '底盘 ROS 里程计（真实通信另见 CAN）', .5, 2.),
    }
    observed = {}
    rclpy.init()
    node = rclpy.create_node('ranger_readonly_hardware_check')
    try:
        def receive(topic, message):
            row = observed.setdefault(topic, {'count': 0, 'first': time.monotonic()})
            row.update(count=row['count'] + 1, last=time.monotonic(),
                       stamp=message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)
            if isinstance(message, Image):
                row.update(width=message.width, height=message.height, encoding=message.encoding)
        for topic, (message_type, _, _, _) in topics.items():
            node.create_subscription(message_type, topic,
                                     lambda msg, topic=topic: receive(topic, msg), qos_profile_sensor_data)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        rows = []
        for topic, (_, title, max_age, min_rate) in topics.items():
            row = observed.get(topic, {})
            count = row.get('count', 0)
            rate = (count - 1) / max(row['last'] - row['first'], .001) if count > 1 else 0.
            age = node.get_clock().now().nanoseconds * 1e-9 - row['stamp'] if count else None
            publishers = len(node.get_publishers_info_by_topic(topic))
            valid = (publishers == 1 and count > 1 and age is not None
                     and -.1 <= age <= max_age and rate >= min_rate)
            if isinstance(age, float) and not math.isfinite(age):
                valid = False
            row.update(hz=rate, age_s=age, publishers=publishers)
            rows.append((valid, f'{title}：{publishers} 个发布者，采样 {count} 帧，{rate:.2f} Hz，时间龄 {age_text(age)}'
                         + ('' if valid else '；检查对应硬件终端、停流或重复节点')))
        depth = observed.get('/camera/aligned_depth_to_color/image_raw', {})
        color = observed.get('/camera/color/image_raw', {})
        aligned = (depth.get('encoding') in ('16UC1', '32FC1') and depth.get('width', 0) > 0
                   and (depth.get('width'), depth.get('height')) == (color.get('width'), color.get('height')))
        rows.append((aligned, '对齐深度编码及尺寸与彩色图像匹配'))
        return rows, observed
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description='Ranger 协作验收只读检查；不发运动或 VLM 请求，不重启硬件。')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--offline', action='store_true', help='仅检查配置，不连接 ROS 或硬件')
    mode.add_argument('--hardware-only', action='store_true', help='仅检查 CAN 与真实传感器数据，不要求 Nav2')
    parser.add_argument('--config-dir', default=os.environ.get('RANGER_CONFIG'))
    parser.add_argument('--timeout', type=float, default=15., help='原有在线 inspect 窗口，默认15秒')
    parser.add_argument('--hardware-window', type=float, default=5., help='硬件采样窗口，默认5秒')
    parser.add_argument('--output-dir', help='保存检查证据的全新目录；默认当前验收目录下自动创建')
    args = parser.parse_args(argv)
    for name in ('timeout', 'hardware_window'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'--{name.replace("_", "-")} 必须为正有限秒数')
    if not args.hardware_only and not args.config_dir:
        parser.error('缺少 RANGER_CONFIG；先 source RangerVLM_env.bash 或指定 --config-dir')
    root = Path(os.environ.get('ACCEPTANCE_RUN') or REPO / 'build/ranger-local')
    try:
        if args.output_dir:
            destination = Path(args.output_dir).expanduser().resolve()
            destination.mkdir(parents=True, exist_ok=False)
        else:
            root.mkdir(parents=True, exist_ok=True)
            destination = Path(tempfile.mkdtemp(prefix='check-' + time.strftime('%Y%m%d_%H%M%S') + '-', dir=root))
    except OSError as error:
        print(format_check(False, '不能创建检查日志目录：' + str(error)))
        return 1
    all_rows = []

    def show(rows):
        all_rows.extend(rows)
        for row in rows:
            print(format_check(*row), flush=True)

    print('Ranger 单机只读检查（不发送运动指令或 VLM 请求）', flush=True)
    try:
        base = ['bash', str(REPO / 'scripts/start_ranger_local.sh'), '--config-dir', args.config_dir or '']
        if not args.hardware_only:
            print('正在检查本轮配置…', flush=True)
            report, code = run_report(base + ['--offline', '--require-ready'], destination / 'configuration.log', 30)
            show(configuration_checks(report or {}, code))
        if not args.offline:
            print(f'正在采样 CAN、相机、雷达和里程计（{args.hardware_window:g}秒）…', flush=True)
            before = can_snapshot('can0')
            began = time.monotonic()
            try:
                with quiet_native_output(destination / 'hardware-ros.log'):
                    hardware_rows, observations = hardware_probe(args.hardware_window)
            except Exception as error:
                hardware_rows = [(False, f'硬件采样失败：{type(error).__name__}: {error}；检查环境和硬件日志')]
                observations = {}
            # A missing ROS dependency must not turn the CAN observation into a zero-length window.
            remaining = args.hardware_window - (time.monotonic() - began)
            if remaining > 0:
                time.sleep(remaining)
            after = can_snapshot('can0')
            (destination / 'hardware.json').write_text(json.dumps(
                dict(can_before=before, can_after=after, topics=observations), ensure_ascii=False, indent=2) + '\n')
            show(can_checks(before, after))
            show(hardware_rows)
            if not args.hardware_only:
                print(f'正在检查定位、RGB-D、costmap 及 Nav2（{args.timeout:g}秒观察窗口）…', flush=True)
                report, code = run_report(base + ['--action', 'inspect', '--timeout', str(args.timeout), '--require-ready'],
                                          destination / 'inspect.log', args.timeout + 30)
                show(inspection_checks(report or {}, code))
    except KeyboardInterrupt:
        print('\n检查已中断；未恢复或启动任何运动任务。')
        return 130
    except (OSError, ValueError) as error:
        show([(False, '检查未能完成：' + str(error))])
    failed = sum(not passed for passed, _ in all_rows)
    print(f'\n检查结束：{len(all_rows) - failed} 项通过，{failed} 项失败。')
    if not failed:
        print('本轮所选检查通过；实际运动仍按验收文档由现场操作者启动。')
    print('完整诊断日志：' + str(destination))
    (destination / 'summary.txt').write_text('\n'.join(format_check(*row) for row in all_rows) + '\n')
    return int(failed > 0)


if __name__ == '__main__':
    raise SystemExit(main())
