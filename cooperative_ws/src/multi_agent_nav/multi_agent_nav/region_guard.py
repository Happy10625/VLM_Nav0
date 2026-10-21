from dataclasses import dataclass
import math
from .models import Pose2D

def cross(a,b,c):
    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])

def on_segment(a,b,p):
    return abs(cross(a,b,p))<1e-9 and min(a[0],b[0])-1e-9<=p[0]<=max(a[0],b[0])+1e-9 and min(a[1],b[1])-1e-9<=p[1]<=max(a[1],b[1])+1e-9

def segments_intersect(a,b,c,d):
    return (cross(a,b,c)*cross(a,b,d)<0 and cross(c,d,a)*cross(c,d,b)<0) or any(
        (on_segment(a,b,c),on_segment(a,b,d),on_segment(c,d,a),on_segment(c,d,b)))

def edges(poly):
    return zip(poly, poly[1:]+poly[:1])

def inside(p,poly):
    result=False
    for a,b in edges(poly):
        if on_segment(a,b,p): return True
        if (a[1]>p[1])!=(b[1]>p[1]) and p[0] < (b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1])+a[0]:
            result=not result
    return result

def footprint_intersects(pose, footprint, polygon):
    c,s=math.cos(pose.yaw),math.sin(pose.yaw)
    body=[(pose.x+c*x-s*y,pose.y+s*x+c*y) for x,y in footprint]
    return any(inside(p,polygon) for p in body) or any(inside(p,body) for p in polygon) or any(
        segments_intersect(a,b,c,d) for a,b in edges(body) for c,d in edges(polygon))

def point_segment_distance(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    length=dx*dx+dy*dy
    t=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length)) if length else 0
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)

def path_intersects(path,polygon,radius):
    points=[(p.x,p.y) for p in path]
    if any(inside(p,polygon) for p in points): return True
    segments=list(zip(points,points[1:])) or [(points[0],points[0])]
    for a,b in segments:
        for c,d in edges(polygon):
            if segments_intersect(a,b,c,d) or min(point_segment_distance(a,c,d),point_segment_distance(b,c,d),
                                                point_segment_distance(c,a,b),point_segment_distance(d,a,b))<=radius:
                return True
    return False

@dataclass
class GuardDecision:
    action: str
    resource_id: str = ''
    reason: str = ''

class RegionGuard:
    def __init__(self, regions, stop_margin):
        self.regions, self.stop_margin=regions,stop_margin
        for region in regions.values():
            polygon=region['polygon']
            if len(polygon)<3 or not all(len(p)==2 and all(math.isfinite(v) for v in p) for p in polygon):
                raise ValueError('invalid resource polygon')

    def check(self, pose, footprint, path, permits, health):
        if not health.localization_valid or not pose.valid or not health.path_valid or not path or not footprint or self.stop_margin is None or self.stop_margin<0:
            return GuardDecision('STOP',reason='invalid pose/path/footprint/stopping margin')
        if not self.regions: return GuardDecision('ALLOW')
        radius=max(math.hypot(x,y) for x,y in footprint)
        # ponytail: circumcircle swept envelope is conservative at turns; exact swept polygon if field clearance is too restrictive.
        for key,region in sorted(self.regions.items(),key=lambda pair: pair[0]=='basket'):
            if key in permits: continue
            polygon=region['polygon']
            if path_intersects([pose],polygon,radius+self.stop_margin):
                return GuardDecision('STOP',key,'unpermitted boundary/stopping envelope')
            if path_intersects(path,polygon,radius+self.stop_margin):
                return GuardDecision('REQUEST',key)
        return GuardDecision('ALLOW')

    def transit_exit(self, resource_id, robot_id, path):
        points=self.regions[resource_id].get('transit_exits',{}).get(robot_id,[])
        if not points: return None
        # Select an exit nearer the final destination than the starting side.
        end=path[-1]
        candidate=min((Pose2D(*p) for p in points),key=lambda p:math.hypot(end.x-p.x,end.y-p.y))
        return candidate
