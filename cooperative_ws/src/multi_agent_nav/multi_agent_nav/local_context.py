"""A local reference is one robot's map, never a cross-map calibration."""
from copy import deepcopy


def local_config(config,robot_id):
    if robot_id not in ('go2','ranger'):
        raise ValueError('unknown robot_id: '+str(robot_id))
    result=deepcopy(config)
    result['coordinate_mode']='local_only'
    if robot_id=='go2':
        return result
    robot=result['robot_'+robot_id]
    entry=result['alignment']['robots'][robot_id]
    entry.update(local_map_id=robot.get('local_map_id',''),map_version=robot.get('map_version',''),
        transform=[0.,0.,0.])
    result['alignment'].update(common_frame_id=robot_id+'_local_map',alignment_id='local_only',
        validated=False,report=None)
    result['scene']['resources']={}
    result['coordinate_mode']='local_only'
    return result
