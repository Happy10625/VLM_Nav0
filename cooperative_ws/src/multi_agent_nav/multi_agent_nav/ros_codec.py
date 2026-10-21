from dataclasses import fields
from . import models

def to_ros(dto):
    from multi_agent_task_msgs import msg
    message=getattr(msg,type(dto).__name__)()
    for f in fields(dto):
        value=getattr(dto,f.name)
        if isinstance(value,models.Pose2D):
            target=getattr(message,f.name)
            target.x,target.y,target.theta=float(value.x),float(value.y),float(value.yaw)
        else:
            setattr(message,f.name,float(value) if f.type is float else value)
    return message

def from_ros(message):
    cls=getattr(models,type(message).__name__)
    values={}
    for f in fields(cls):
        value=getattr(message,f.name)
        values[f.name]=models.Pose2D(value.x,value.y,value.theta) if f.type is models.Pose2D else value
    return cls(**values)
