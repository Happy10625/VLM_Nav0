"""Select a known local runtime; importing this module does not initialize ROS."""
import math

def create_runtime(robot_id,config,*,read_only=False,global_goal_checks=False,node_name=None):
    if robot_id not in ('go2','ranger'):
        raise ValueError('unknown robot_id: '+str(robot_id))
    from ..frame_adapter import configured_frame
    frame=configured_frame(config,robot_id)
    if robot_id=='go2':
        from .go2 import make_observer,Go2Adapter as Adapter
    else:
        from .ranger import make_observer,RangerAdapter as Adapter
    robot_config=dict(config['robot_'+robot_id])
    if config['scene'].get('share_c_coordinates',True) is False and config.get('coordinate_mode')!='local_only':
        robot_config['scan_angles']=[-(i+1)*math.pi/4 for i in range(8)]
    observer=make_observer(robot_config,read_only=read_only,node_name=node_name)
    try:
        adapter=Adapter(observer,robot_config,frame,read_only=read_only,
            global_goal_checks=global_goal_checks)
        if robot_id=='ranger' and not read_only:
            from ..ranger_visualization import RangerVisualization
            adapter.visualization=RangerVisualization(observer,frame)
        return observer,adapter
    except BaseException:
        observer.destroy_node()
        raise
