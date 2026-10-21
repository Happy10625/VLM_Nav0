#!/usr/bin/env python3
"""Two-PC PAUSED task-state transport, following the supplied communication procedure."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET


def validate_bridge(data, side):
    expected = {f'/multi_agent/{robot}/{kind}': 'multi_agent_task_msgs/msg/' + message
                for robot in ('go2', 'ranger')
                for kind, message in [('state', 'AgentState'), ('observation', 'Observation')]}
    expected['/multi_agent/go2/command'] = 'multi_agent_task_msgs/msg/Command'
    if data.get('from_domain') != 0 or data.get('to_domain') != 40 or not data.get('name'):
        raise ValueError('task bridge必须为命名的Domain0→40桥')
    if set(data.get('topics', {})) != set(expected):
        raise ValueError('task bridge只允许规定的5条新任务话题，不允许旧state/TF/速度/图像等')
    for topic, message in expected.items():
        route = data['topics'][topic]
        reverse = topic.startswith('/multi_agent/ranger/') if side == 'go2' else topic.startswith('/multi_agent/go2/')
        qos = route.get('qos', {})
        if (route.get('type') != message or route.get('reversed', False) is not reverse
                or route.get('bidirectional', False) is not False
                or any(key in route for key in ('from_domain', 'to_domain', 'remap', 'remap_name'))
                or qos.get('reliability') != 'reliable' or qos.get('durability') != 'volatile'
                or qos.get('history') != 'keep_last'
                or qos.get('depth') != (50 if topic.endswith('/observation') else 10)):
            raise ValueError(f'桥接方向、类型或QoS不符：{topic}')
    return data['name']


def dds_interfaces(path):
    root = ET.fromstring(os.path.expandvars(Path(path).read_text()))
    domains = {}
    for domain in root.findall('{*}Domain'):
        names = [item.get('name', '') for item in domain.findall('.//{*}NetworkInterface')]
        identifier = domain.get('Id', 'any')
        if identifier in domains or len(names) != 1 or not names[0] or '$' in names[0]:
            raise ValueError(f'DDS须按域显式绑定唯一实际网卡：{path}')
        domains[identifier] = names[0]
    return domains


def validate_dds(local_path, bridge_path, side):
    local, bridge = dds_interfaces(local_path), dds_interfaces(bridge_path)
    if (set(bridge) != {'0', '40'} or not ({'0', 'any'} & set(local))
            or {'0', 'any'} <= set(local)):
        raise ValueError('普通节点DDS须明确Domain0网卡，bridge须分别配置Domain0和40')
    if local.get('0', local.get('any')) != bridge['0'] or bridge['0'] == bridge['40']:
        raise ValueError('本地Domain0与桥Domain0须同网卡，且不能走共享Domain40网卡')
    if side == 'go2' and bridge != {'0': 'enp4s0', '40': 'wlo1'}:
        raise ValueError('本项目Go2要求Domain0=enp4s0，Domain40=wlo1')
    return bridge


def blocked_config(directory, side):
    from multi_agent_nav.config import load_config, live_gates
    config = load_config(str(directory))
    gates = live_gates(config, local_robot=side)
    if not gates:
        raise ValueError('live_gates 已为空，不能套用PAUSED无运动adapter流程；不要人为修改配置制造阻塞')
    return config, gates


def launch_command(action, side, directory):
    if action == 'bridge':
        return ['ros2', 'run', 'domain_bridge', 'domain_bridge',
                str(directory / f'bridge_{side}_pending.yaml')]
    return ['ros2', 'run', 'multi_agent_nav', 'multi_agent_agent_node', '--ros-args',
            '-r', f'__node:=multi_agent_{side}_agent', '-p', f'robot_id:={side}',
            '-p', 'simulation:=false', '-p', 'topic_prefix:=/multi_agent',
            '-p', f'config_dir:={directory}']


class StateEvidence:
    def __init__(self, robot):
        self.robot, self.count, self.advances = robot, 0, 0
        self.stamp = self.received = self.sequence = self.latest = None
        self.max_gap = 0.
        self.errors = set()

    def observe(self, message, received):
        self.count += 1
        for field, expected in [('schema_version', 1), ('robot_id', self.robot), ('simulation', False),
                                ('run_state', 'PAUSED'), ('ready', False), ('deposited', False)]:
            if getattr(message, field) != expected:
                self.errors.add(f'{field}不符：{getattr(message, field)!r}')
        if not message.reason.strip():
            self.errors.add('PAUSED缺少真实阻塞原因')
        stamp = message.stamp
        if not math.isfinite(stamp) or stamp <= 0:
            self.errors.add('stamp非有限正数')
        if self.stamp is not None:
            if stamp < self.stamp or message.sequence < self.sequence:
                self.errors.add('stamp或sequence倒退')
            self.advances += int(stamp > self.stamp)
            self.max_gap = max(self.max_gap, received - self.received)
        self.stamp, self.sequence, self.received = stamp, message.sequence, received
        fields = ('robot_id', 'schema_version', 'simulation', 'run_state', 'reason', 'ready',
                  'deposited', 'local_map_id', 'map_version', 'common_frame_id', 'alignment_id',
                  'pose_valid', 'localization_valid', 'stopped', 'stamp', 'sequence')
        self.latest = {name: getattr(message, name) for name in fields}

    def issues(self, now, lost_timeout):
        errors = sorted(self.errors)
        if self.count < 3 or self.advances < 2:
            errors.append('至少需3条状态、stamp推进2次；阻塞态sequence允许保持0')
        if self.received is None or now - self.received > lost_timeout:
            errors.append('状态接收超时')
        if self.max_gap > lost_timeout:
            errors.append(f'采样中最大接收间隔{self.max_gap:.3f}s超过lost_timeout')
        return errors


def publisher_issues(publishers, expected_name, bridge=False):
    errors = []
    if len(publishers) != 1:
        errors.append(f'要求唯一发布者，实际{len(publishers)}个')
    for publisher in publishers:
        name = publisher.node_name
        matches = name == expected_name or (bridge and name.startswith(expected_name + '_'))
        if not matches or publisher.topic_type != 'multi_agent_task_msgs/msg/AgentState':
            errors.append(f'发布来源/类型不符：{name}，期望{expected_name}')
        if (publisher.qos_profile.reliability.name != 'RELIABLE'
                or publisher.qos_profile.durability.name != 'VOLATILE'):
            errors.append('发布端必须RELIABLE/VOLATILE')
    return errors


def preflight(args):
    import yaml
    from ament_index_python.packages import get_package_prefix
    from multi_agent_task_msgs.msg import AgentState, Observation, Command
    import multi_agent_nav.config as config_module
    import multi_agent_nav.agent_node as agent_module
    if os.environ.get('ROS_DOMAIN_ID', '0') != '0' or os.environ.get('ROS_LOCALHOST_ONLY', '0') != '0':
        raise ValueError('普通终端须Domain0，ROS_LOCALHOST_ONLY=0')
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_cyclonedds_cpp':
        raise ValueError('本文PAUSED验收要求环境文件加载rmw_cyclonedds_cpp')
    raw = args.config_dir or (os.environ.get('RANGER_CONFIG') if args.side == 'ranger'
                             else os.environ.get('GO2_CONFIG') or os.environ.get('GO2_EXPLORE_CONFIG'))
    if not raw or not Path(raw).is_absolute():
        raise ValueError('须指定实际绝对配置目录：--config-dir或RANGER_CONFIG/GO2_CONFIG')
    directory = Path(raw).resolve()
    env_file = os.environ.get('COMM_ENV_FILE')
    repo = Path(env_file).resolve().parent if env_file else Path(__file__).resolve().parents[1]
    bridge_xml = (Path(os.environ.get('RANGER_BRIDGE_DDS_XML',
                        str(directory / 'cyclonedds_ranger_multi_agent_bridge.xml')))
                  if args.side == 'ranger' else repo / 'config/cyclonedds_go2_multi_agent_bridge.xml')
    uri = urlparse(os.environ.get('CYCLONEDDS_URI', ''))
    if uri.scheme != 'file' or uri.netloc not in ('', 'localhost') or not uri.path:
        raise ValueError('CYCLONEDDS_URI须指向本机实际DDS XML文件（file:///绝对路径）')
    interfaces = validate_dds(Path(unquote(uri.path)), bridge_xml, args.side)
    route_file = directory / f'bridge_{args.side}_pending.yaml'
    bridge_name = validate_bridge(yaml.safe_load(route_file.read_text()), args.side)
    print(f'环境={env_file}；配置={directory}；Domain0；RMW=rmw_cyclonedds_cpp')
    print(f'普通DDS={unquote(uri.path)}；桥DDS={bridge_xml}；域/网卡={interfaces}')
    addresses = json.loads(subprocess.check_output(['ip', '-j', 'address'], text=True))
    for name in interfaces.values():
        entries = [entry for entry in addresses if entry['ifname'] == name]
        if (not entries or 'UP' not in entries[0].get('flags', [])
                or not any(a['family'] == 'inet' for a in entries[0].get('addr_info', []))):
            raise ValueError(f'网卡{name}须存在、UP且有IPv4；不自动修改网络')
        print('网卡', name, [a['local'] for a in entries[0]['addr_info'] if a['family'] == 'inet'])
    for package in ('multi_agent_nav', 'multi_agent_task_msgs', 'domain_bridge', 'rmw_cyclonedds_cpp'):
        print(f'包 {package}={get_package_prefix(package)}')
    print(f'实际config模块={config_module.__file__}；agent模块={agent_module.__file__}')
    for message in (AgentState, Observation, Command):
        definition = json.dumps(message.get_fields_and_field_types(), separators=(',', ':'))
        print(f'{message.__name__} schema_sha256={hashlib.sha256(definition.encode()).hexdigest()} {definition}')
    if (repo / '.git').exists():
        for command in (['branch', '--show-current'], ['rev-parse', 'HEAD'], ['status', '--short']):
            subprocess.run(['git', '-C', str(repo), *command], check=True)
    config, gates = blocked_config(directory, args.side)
    for gate in gates:
        print('真实阻塞项:', gate)
    lost_timeout = float(config['scene'].get('lost_timeout', 3.))
    heartbeat = float(config['scene'].get('heartbeat_hz', 2.))
    if not all(math.isfinite(v) and v > 0 for v in (lost_timeout, heartbeat)):
        raise ValueError('heartbeat_hz/lost_timeout必须为有限正数')
    print(f'PREFLIGHT_OK：真实gates非空；heartbeat_hz={heartbeat}；lost_timeout={lost_timeout}s。'
          '仅配置核对，尚未证明通信。两端须人工比对3个schema_sha256。', flush=True)
    return dict(directory=directory, bridge_xml=bridge_xml, bridge_name=bridge_name,
                lost_timeout=lost_timeout)


def reject_existing_node(name):
    result = subprocess.run(['ros2', 'node', 'list', '--no-daemon'], capture_output=True,
                            text=True, timeout=10, check=True)
    if any(node.rsplit('/', 1)[-1] == name or node.rsplit('/', 1)[-1].startswith(name + '_')
           for node in result.stdout.splitlines()):
        raise ValueError(f'已有节点{name}；先核实既有进程，拒绝重复启动')


def monitor(side, context, duration, expect_stop=False):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from multi_agent_task_msgs.msg import AgentState
    peer = 'ranger' if side == 'go2' else 'go2'
    samples = {robot: StateEvidence(robot) for robot in (side, peer)}
    node = None
    try:
        rclpy.init(args=[])
        node = rclpy.create_node(f'check_{side}_task_comm_{os.getpid()}',
                                 enable_rosout=False, start_parameter_services=False)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        for robot, sample in samples.items():
            node.create_subscription(AgentState, f'/multi_agent/{robot}/state',
                lambda message, sample=sample: sample.observe(message, time.monotonic()), qos)
        print(f'采样{duration:g}s：本机及对端新任务PAUSED状态（不订阅里程计）。', flush=True)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(.2, max(0., deadline - time.monotonic())))
        errors = []
        for robot, sample in samples.items():
            topic = f'/multi_agent/{robot}/state'
            publishers = node.get_publishers_info_by_topic(topic)
            expected = f'multi_agent_{side}_agent' if robot == side else context['bridge_name']
            issues = sample.issues(time.monotonic(), context['lost_timeout'])
            issues += publisher_issues(publishers, expected, bridge=robot == peer)
            print(topic, f'接收={sample.count}，stamp推进={sample.advances}，最大间隔={sample.max_gap:.3f}s')
            print('最新状态:', json.dumps(sample.latest, ensure_ascii=False))
            for publisher in publishers:
                print('发布端:', publisher.node_namespace, publisher.node_name,
                      publisher.qos_profile.reliability.name, publisher.qos_profile.durability.name)
            errors += [f'{robot}: {issue}' for issue in issues]
        if errors:
            print('FAIL：' + '；'.join(errors))
            return 1
        if expect_stop:
            print(f'现在请在{peer}电脑Ctrl+C退出该agent；本机agent及两个bridge保持运行。'
                  f'将在{duration:g}s内观察停流。', flush=True)
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.2)
                if samples[side].issues(time.monotonic(), context['lost_timeout']):
                    print('FAIL：停流核对期间本机状态异常')
                    return 1
                if samples[peer].errors:
                    print('FAIL：停流核对期间对端状态异常：' + '；'.join(sorted(samples[peer].errors)))
                    return 1
                if time.monotonic() - samples[peer].received > context['lost_timeout']:
                    for robot in (side, peer):
                        expected = f'multi_agent_{side}_agent' if robot == side else context['bridge_name']
                        issues = publisher_issues(node.get_publishers_info_by_topic(
                            f'/multi_agent/{robot}/state'), expected, bridge=robot == peer)
                        if issues:
                            print('FAIL：停流阶段发布端变化：' + '；'.join(issues))
                            return 1
                    print('STOP_OBSERVED：对端状态流已停止；结合人工退出记录确认来源。')
                    break
            else:
                print('FAIL：未观察到对端状态流停止')
                return 1
        print('DATA_OK：本机PAUSED及对端经本机导入桥的新鲜状态满足采样判据。'
              '双向结果及人工停流记录需两端共同确认；定位/Observation/任务未验收。')
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def positive_seconds(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError('必须为有限正秒数')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description='新任务PAUSED静止通信：默认preflight不初始化ROS；按步骤显式启动bridge/agent/check。')
    parser.add_argument('--side', choices=('go2', 'ranger'), required=True)
    parser.add_argument('action', nargs='?', default='preflight', choices=('preflight', 'bridge', 'agent', 'check'))
    parser.add_argument('--config-dir', help='实际公共配置目录绝对路径；默认取本机环境变量')
    parser.add_argument('--duration', type=positive_seconds, default=12., help='check采样秒数，默认12')
    parser.add_argument('--expect-peer-stop', action='store_true', help='check采样后等待人工退出对端agent，验证停流')
    args = parser.parse_args(argv)
    if args.expect_peer_stop and args.action != 'check':
        parser.error('--expect-peer-stop仅用于check')
    try:
        context = preflight(args)
        if args.action == 'preflight':
            return 0
        if args.action == 'check':
            return monitor(args.side, context, args.duration, args.expect_peer_stop)
        reject_existing_node(context['bridge_name'] if args.action == 'bridge'
                             else f'multi_agent_{args.side}_agent')
        # Recheck the same installed gates immediately before any process is started.
        blocked_config(context['directory'], args.side)
        command = launch_command(args.action, args.side, context['directory'])
        env = dict(os.environ)
        if args.action == 'bridge':
            env['CYCLONEDDS_URI'] = context['bridge_xml'].resolve().as_uri()
            env.pop('ROS_DOMAIN_ID', None)
        else:
            env['ROS_DOMAIN_ID'] = '0'
        print('执行并保持本终端运行：', shlex.join(command), flush=True)
        os.execvpe(command[0], command, env)
    except KeyboardInterrupt:
        print('已中断，不能记为通过。', file=sys.stderr)
        return 130
    except Exception as exc:
        print(f'ERROR：{exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
