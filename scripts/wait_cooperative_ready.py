#!/usr/bin/env python3
"""Read-only preparation sampling; never starts a task or publishes motion."""
import argparse
import math
import os
from pathlib import Path
import time

from check_multi_agent_comm import publisher_issues


class PreparationEvidence:
    def __init__(self, robot, entry):
        self.robot,self.entry=robot,entry
        self.count=0
        self.stamp=self.received=None
        self.reason='waiting first valid state'

    def observe(self, state, now):
        from multi_agent_nav.models import Pose2D, SCHEMA_VERSION, startup_state
        identity=(state.local_map_id,state.map_version)==(self.entry['local_map_id'],self.entry['map_version'])
        valid=(state.robot_id==self.robot and state.schema_version==SCHEMA_VERSION and not state.simulation and
            not state.common_frame_id and not state.alignment_id and not state.pose_valid and
            state.common_pose==Pose2D() and identity and startup_state(state))
        if not valid:
            self.count=0;self.stamp=self.received=None
            self.reason=state.reason or 'state/identity not clean and ready: '+state.run_state
            return
        if not math.isfinite(state.stamp) or state.stamp<=0 or self.stamp is not None and state.stamp<=self.stamp:
            self.count=0
            self.reason='waiting advancing stamp'
            return
        if self.received is not None and now-self.received>3.:self.count=0
        self.count+=1
        self.stamp,self.received=state.stamp,now
        self.reason=''

    def issue(self,now):
        if self.received is None or now-self.received>3.:return self.reason or 'valid state stale'
        if self.count<3:return f'waiting advancing states: {self.count}/3'
        return self.reason


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side',choices=('ranger','go2'),required=True)
    parser.add_argument('--config-dir',required=True)
    args=parser.parse_args(argv)
    import yaml
    import rclpy
    from multi_agent_task_msgs.msg import AgentState
    from multi_agent_nav.config import load_config
    from multi_agent_nav.node_common import qos
    from multi_agent_nav.ros_codec import from_ros
    config=load_config(args.config_dir)
    if config['scene'].get('share_c_coordinates') is not False:
        raise ValueError('preparation sampling requires private coordinate mode')
    bridge=yaml.safe_load((Path(args.config_dir)/f'bridge_{args.side}_pending.yaml').read_text())['name']
    samples={r:PreparationEvidence(r,entry) for r,entry in config['alignment']['robots'].items()}
    node=None
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
        node=rclpy.create_node(f'wait_{args.side}_cooperative_ready_{os.getpid()}',
            enable_rosout=False,start_parameter_services=False)
        for robot,sample in samples.items():
            node.create_subscription(AgentState,f'/multi_agent/{robot}/state',
                lambda message,sample=sample:sample.observe(from_ros(message),time.monotonic()),qos())
        next_report=0.
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.2)
            now=time.monotonic();issues=[]
            for robot,sample in samples.items():
                publishers=node.get_publishers_info_by_topic(f'/multi_agent/{robot}/state')
                expected=f'multi_agent_{args.side}_agent' if robot==args.side else bridge
                errors=publisher_issues(publishers,expected,bridge=robot!=args.side)
                if any(p.qos_profile.history.name!='KEEP_LAST' or p.qos_profile.depth!=10 for p in publishers):
                    errors.append('publisher QoS requires KEEP_LAST depth=10')
                if errors:sample.count=0
                issues += [robot+': '+error for error in errors]
                issue=sample.issue(now)
                if issue:issues.append(robot+': '+issue)
            if not issues:
                print('PREPARATION_OK: both peers supplied 3 fresh advancing clean WAITING_PEER/STANDBY states; no start sent.',flush=True)
                return 0
            if now>=next_report:
                print('WAITING: '+'; '.join(issues),flush=True)
                next_report=now+5.
        return 1
    except KeyboardInterrupt:
        print('Preparation wait interrupted; no start sent.',flush=True)
        return 130
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
