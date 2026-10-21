import argparse
import rclpy
from multi_agent_task_msgs.srv import StartTask
from std_srvs.srv import Trigger

def main():
    parser=argparse.ArgumentParser(description='Explicit local start/pause; never accepts arbitrary coordinates')
    parser.add_argument('action',choices=('start','pause'))
    parser.add_argument('--scenario',default='lab')
    parser.add_argument('--prefix',default='/multi_agent')
    args=parser.parse_args()
    rclpy.init(args=[])
    node=rclpy.create_node('cooperative_local_client')
    try:
        cls=StartTask if args.action=='start' else Trigger
        client=node.create_client(cls,f'{args.prefix}/coordinator/{args.action}')
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('coordinator local service unavailable')
        request=cls.Request()
        if args.action=='start':request.scenario_id=args.scenario
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        if not future.done():raise RuntimeError('local service timeout')
        response=future.result()
        accepted=response.accepted if args.action=='start' else response.success
        print(response)
        return 0 if accepted else 1
    finally:
        node.destroy_node();rclpy.shutdown()
