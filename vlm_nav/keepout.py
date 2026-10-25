"""Map-aligned binary keepout masks shared by Nav2 launch and local candidates."""
import hashlib
import math
from pathlib import Path

import numpy as np
from PIL import Image
import yaml


def _read(path):
    path=Path(path).expanduser().resolve()
    data=yaml.safe_load(path.read_text())
    resolution=data['resolution'];origin=data['origin']
    if (not isinstance(resolution,(int,float)) or not math.isfinite(resolution) or resolution<=0 or
            len(origin)!=3 or not all(math.isfinite(v) for v in origin)):
        raise ValueError('finite map geometry required')
    image=(path.parent/data['image']).resolve()
    return path,data,image


class KeepoutMask:
    def __init__(self,source_yaml,mask_yaml):
        source,source_data,source_image=_read(source_yaml)
        self.path,data,image=_read(mask_yaml)
        if (data.get('mode')!='trinary' or data.get('negate')!=0 or
                data['resolution']!=source_data['resolution'] or data['origin']!=source_data['origin'] or
                not 0<data.get('free_thresh',0)<data.get('occupied_thresh',0)<1):
            raise ValueError('keepout mask must be aligned trinary/negate=0')
        with Image.open(source_image) as original,Image.open(image) as mask:
            if mask.size!=original.size:raise ValueError('keepout mask dimensions differ from localization map')
            self.pixels=np.array(mask.convert('L'))
        if set(np.unique(self.pixels))!={0,255}:
            raise ValueError('keepout mask must contain black outside and white inside, without gray/unknown')
        binding=data.get('source_map',{})
        for key,path in (('yaml_sha256',source),('image_sha256',source_image)):
            if key in binding and binding[key]!=hashlib.sha256(path.read_bytes()).hexdigest():
                raise ValueError('keepout source map digest mismatch')
        self.resolution,self.origin=data['resolution'],data['origin']
        self.free=np.flipud(self.pixels==255).copy()
        self.digests={p:hashlib.sha256(p.read_bytes()).hexdigest() for p in (self.path,image)}
        self.digest=hashlib.sha256(''.join(self.digests.values()).encode()).hexdigest()

    def contains(self,x,y):
        if not math.isfinite(x) or not math.isfinite(y):return False
        ox,oy,yaw=self.origin;c,s=math.cos(yaw),math.sin(yaw)
        mx=(c*(x-ox)+s*(y-oy))/self.resolution
        my=(-s*(x-ox)+c*(y-oy))/self.resolution
        height,width=self.free.shape
        return 0<=mx<width and 0<=my<height and bool(self.free[int(my),int(mx)])

    def matches_grid(self,message):
        info=message.info;pose=info.origin;ox,oy,yaw=self.origin
        return (message.header.frame_id=='map' and (info.height,info.width)==self.free.shape and
            abs(info.resolution-self.resolution)<1e-6 and abs(pose.position.x-ox)<1e-6 and
            abs(pose.position.y-oy)<1e-6 and abs(pose.position.z)<1e-6 and
            abs(pose.orientation.x)<1e-6 and abs(pose.orientation.y)<1e-6 and
            abs(pose.orientation.z-math.sin(yaw/2))<1e-6 and abs(pose.orientation.w-math.cos(yaw/2))<1e-6 and
            bytes(message.data)==np.where(self.free,0,100).astype('int8').tobytes())

    @staticmethod
    def matches_info(message):
        return (message.type==0 and message.base==0. and message.multiplier==1. and
            message.filter_mask_topic=='/keepout_filter_mask')


def configured_mask(robot):
    if not robot.get('keepout_mask_yaml'):return None
    mask=KeepoutMask(robot['map_path'],robot['keepout_mask_yaml'])
    if robot.get('keepout_mask_digest')!=mask.digest:
        raise ValueError('keepout mask digest missing or changed; validate mask and restart Nav2 before updating configuration')
    return mask


def write_mask(source_yaml,polygon,output_yaml,margin):
    from multi_agent_nav.region_guard import edges,segments_intersect
    source,data,image=_read(source_yaml)
    output=Path(output_yaml).resolve();pgm=output.with_suffix('.pgm')
    if output.exists() or pgm.exists():raise FileExistsError('keepout output already exists; choose a new directory')
    if (len(polygon)<3 or any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in polygon) or
            len(set(map(tuple,polygon)))!=len(polygon) or not math.isfinite(margin) or margin<0):
        raise ValueError('finite distinct polygon vertices and nonnegative margin required')
    segments=list(edges(polygon))
    if any(segments_intersect(a,b,c,d) for i,(a,b) in enumerate(segments) for j,(c,d) in enumerate(segments)
            if j>i+1 and not (i==0 and j==len(segments)-1)):
        raise ValueError('self-intersecting site polygon')
    with Image.open(image) as original:width,height=original.size
    res=data['resolution'];ox,oy,yaw=data['origin'];c,s=math.cos(yaw),math.sin(yaw)
    for x,y in polygon:
        mx,my=c*(x-ox)+s*(y-oy),-s*(x-ox)+c*(y-oy)
        if not (0<=mx<width*res and 0<=my<height*res):raise ValueError('site vertex outside localization map')
    gy,gx=np.mgrid[:height,:width];mx=(gx+.5)*res;my=(gy+.5)*res
    wx,wy=ox+c*mx-s*my,oy+s*mx+c*my
    allowed=np.zeros((height,width),dtype=bool)
    clearance=np.full((height,width),np.inf)
    for (ax,ay),(bx,by) in segments:
        dx,dy=bx-ax,by-ay
        if dy:allowed ^= ((ay>wy)!=(by>wy)) & (wx<(bx-ax)*(wy-ay)/dy+ax)
        t=np.clip(((wx-ax)*dx+(wy-ay)*dy)/(dx*dx+dy*dy),0,1)
        clearance=np.minimum(clearance,(wx-ax-t*dx)**2+(wy-ay-t*dy)**2)
    # A complete white cell must fit inside the inset polygon, including its corners.
    allowed &= clearance>=(margin+res/math.sqrt(2))**2
    if not allowed.any():raise ValueError('site polygon has no allowed cells after footprint margin')
    output.parent.mkdir(parents=True,exist_ok=True)
    with pgm.open('xb') as stream:Image.fromarray(np.flipud(allowed).astype('uint8')*255).save(stream,format='PPM')
    document={k:data[k] for k in ('resolution','origin','free_thresh','occupied_thresh')}
    document.update(image=pgm.name,mode='trinary',negate=0,source_map=dict(
        yaml_path=str(source),yaml_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        image_sha256=hashlib.sha256(image.read_bytes()).hexdigest()),boundary_margin_m=margin,site_polygon=polygon)
    with output.open('x') as stream:yaml.safe_dump(document,stream,sort_keys=False)
    return output
