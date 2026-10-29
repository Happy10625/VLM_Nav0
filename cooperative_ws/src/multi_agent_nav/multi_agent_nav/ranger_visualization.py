"""Local RViz markers for the submitted candidates and located A/B/C targets."""
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from .models import Pose2D, TARGETS


class RangerVisualization:
    def __init__(self,node,frame):
        self.node,self.frame=node,frame
        self.task_id=None
        self.targets={}
        self.points={}
        self.observed_radius=0.
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.candidates_pub=node.create_publisher(MarkerArray,
            '/vlm_nav/ranger_exploration/vlm_candidates',qos)
        self.targets_pub=node.create_publisher(MarkerArray,
            '/vlm_nav/ranger_exploration/targets',qos)
        self.observed_pub=node.create_publisher(MarkerArray,
            '/vlm_nav/ranger_exploration/observed_points',qos)

    def markers(self,points,color,namespace):
        clear=Marker();clear.header.frame_id='map';clear.action=Marker.DELETEALL
        output=MarkerArray(markers=[clear])
        for index,(label,pose) in enumerate(points):
            for offset,kind in enumerate((Marker.SPHERE,Marker.TEXT_VIEW_FACING)):
                marker=Marker();marker.header.frame_id='map'
                marker.header.stamp=self.node.get_clock().now().to_msg()
                marker.ns=namespace;marker.id=index*2+offset;marker.type=kind;marker.action=Marker.ADD
                marker.pose.position.x=float(pose.x);marker.pose.position.y=float(pose.y)
                marker.pose.position.z=.15 if offset==0 else .45;marker.pose.orientation.w=1.
                marker.scale.x=marker.scale.y=marker.scale.z=.22 if offset==0 else .18
                marker.color.r,marker.color.g,marker.color.b=color;marker.color.a=1.
                marker.text=label
                output.markers.append(marker)
        return output

    def begin(self,task_id):
        self.candidates_pub.publish(self.markers([], (0.,0.,1.),'vlm_candidates'))
        if task_id!=self.task_id:
            self.task_id=task_id;self.targets.clear();self.points.clear()
            self.targets_pub.publish(self.markers([], (0.,1.,0.),'located_targets'))
            self.observed_pub.publish(self.markers([], (.5,.5,.5),'observed_points'))

    def record(self,event):
        if event.get('state')=='CANDIDATE_GEOMETRY':
            if 'task_id' in event:self.begin(event['task_id'])
            self.points={p['id']:p for p in event['candidates']}
            self.observed_radius=event.get('observed_radius',0.)
        if event.get('state') in ('CANDIDATE_GEOMETRY','OBSERVATION_RECORD'):
            points=[('observed',Pose2D(*p['pose'])) for pid,p in self.points.items()
                    if event.get('records',{}).get(pid,{}).get('status')=='observed']
            markers=self.markers(points,(.5,.5,.5),'observed_points')
            for index,(_,pose) in enumerate(points):
                if not self.observed_radius:continue
                disk=Marker();disk.header.frame_id='map';disk.header.stamp=self.node.get_clock().now().to_msg()
                disk.ns='observed_areas';disk.id=index;disk.type=Marker.CYLINDER;disk.action=Marker.ADD
                disk.pose.position.x=float(pose.x);disk.pose.position.y=float(pose.y)
                disk.pose.position.z=.02;disk.pose.orientation.w=1.
                disk.scale.x=disk.scale.y=2*self.observed_radius;disk.scale.z=.02
                disk.color.r=disk.color.g=disk.color.b=.5;disk.color.a=.15
                markers.markers.append(disk)
            self.observed_pub.publish(markers)
            if event.get('observation_status')=='observed':
                self.candidates_pub.publish(self.markers([], (0.,0.,1.),'vlm_candidates'))
        if event.get('state')=='CANDIDATE_REQUEST':
            points=[(p['display_id'],Pose2D(*p['pose'])) for p in event['candidates']]
            self.candidates_pub.publish(self.markers(points,(0.,0.,1.),'vlm_candidates'))

    def observe(self,observations):
        changed=False
        for observation in observations:
            if observation.target_id in TARGETS and observation.navigable:
                self.targets[observation.target_id]=self.frame.to_local(observation.common_position)
                changed=True
        if changed:
            self.targets_pub.publish(self.markers(sorted(self.targets.items()),(0.,1.,0.),'located_targets'))
