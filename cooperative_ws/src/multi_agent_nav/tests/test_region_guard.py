from multi_agent_nav.models import Health, Pose2D
from multi_agent_nav.region_guard import RegionGuard, footprint_intersects

REGION = [[1,-1],[3,-1],[3,1],[1,1]]
FOOT = [[.4,.2],[.4,-.2],[-.4,-.2],[-.4,.2]]

def test_center_outside_but_body_inside_and_replanned_path():
    g = RegionGuard({'door':{'polygon':REGION}}, .3)
    h = Health(Pose2D(), True, True, True, footprint=FOOT, path_valid=True)
    assert footprint_intersects(Pose2D(.8,0), FOOT, REGION)
    assert not footprint_intersects(Pose2D(-1,0), FOOT, REGION)
    assert g.check(h.pose, FOOT, [Pose2D(),Pose2D(0,3)], {}, h).action=='ALLOW'
    assert g.check(h.pose, FOOT, [Pose2D(),Pose2D(4,0)], {}, h).resource_id=='door'
    assert g.check(h.pose, FOOT, [Pose2D(),Pose2D(4,0)], {'door':'token'}, h).action=='ALLOW'
    h.pose=Pose2D(.8,0)
    assert g.check(h.pose, FOOT, [h.pose,Pose2D(4,0)], {}, h).action=='STOP'

def test_unavailable_geometry_path_or_margin_stops():
    h = Health(Pose2D(), True, True, True, footprint=FOOT, path_valid=False)
    assert RegionGuard({'door':{'polygon':REGION}}, .3).check(h.pose, FOOT, [], {}, h).action=='STOP'
    h.path_valid=True
    assert RegionGuard({'door':{'polygon':REGION}}, None).check(h.pose, FOOT, [h.pose], {}, h).action=='STOP'

def test_empty_resources_still_require_valid_localization_path_and_stop_margin():
    from dataclasses import replace
    h=Health(Pose2D(),True,True,True,footprint=FOOT,path_valid=True)
    guard=RegionGuard({},.3)
    assert guard.check(h.pose,FOOT,[h.pose],{},h).action=='ALLOW'
    for health,path,footprint in ((replace(h,localization_valid=False),[h.pose],FOOT),
        (replace(h,path_valid=False),[h.pose],FOOT),(h,[],FOOT),(h,[h.pose],[])):
        assert guard.check(health.pose,footprint,path,{},health).action=='STOP'
    assert RegionGuard({},None).check(h.pose,FOOT,[h.pose],{},h).action=='STOP'
