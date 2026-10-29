"""Map-only observation geometry and image/ID selection; no ROS or motion authority."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import yaml

from .config import map_file_info, valid_numbers
from .region_guard import edges, inside, segments_intersect, path_intersects
from .models import Pose2D
from vlm_nav.keepout import configured_mask


def rotation_radius(footprint, padding=0.):
    if (not isinstance(footprint, list) or len(footprint)<3
            or any(not valid_numbers(p,2) for p in footprint)
            or not valid_numbers([padding],1) or padding<0):
        raise ValueError('finite footprint and nonnegative padding required')
    # Nav2 pads each nonzero coordinate before checking the footprint.
    radius=max(math.hypot(abs(x)+padding if x else 0.,
                          abs(y)+padding if y else 0.) for x,y in footprint)
    if radius<=0:raise ValueError('positive footprint radius required')
    return radius


def circle_free(raw, x, y, radius, resolution, blocked):
    """Cell-square intersection with the complete rotation disk, including map edges."""
    height,width=raw.shape
    if x-radius<0 or y-radius<0 or x+radius>=width*resolution or y+radius>=height*resolution:
        return False
    gx0,gy0=int((x-radius)/resolution),int((y-radius)/resolution)
    gx1,gy1=int((x+radius)/resolution)+1,int((y+radius)/resolution)+1
    gy,gx=np.mgrid[gy0:gy1,gx0:gx1]
    dx=np.maximum(np.abs((gx+.5)*resolution-x)-resolution/2,0)
    dy=np.maximum(np.abs((gy+.5)*resolution-y)-resolution/2,0)
    disk=dx*dx+dy*dy<=radius*radius
    return not np.any(disk & blocked(raw[gy0:gy1,gx0:gx1]))


class GeneratedCandidates:
    def __init__(self,config,robot_id='go2'):
        self.robot_id=robot_id
        self.observed_radius=3. if robot_id=='ranger' else 0.
        robot=config['robot_'+robot_id];options=robot['generated_candidates']
        distance=options['observation_distance']
        if not valid_numbers([distance],1) or distance<=0:
            raise ValueError('positive finite observation_distance required')
        self.radius=rotation_radius(robot['footprint'],robot.get('footprint_padding',0.))
        self.map_path=Path(robot['map_path']).resolve()
        self.semantic_path=Path(options['semantic_yaml']).resolve() if options.get('semantic_yaml') else None
        info=map_file_info(self.map_path)
        identity=config['alignment']['robots'][robot_id]
        if (info['map_digest']!=robot['map_digest'] or identity['map_version']!='sha256:'+info['map_digest']):
            raise ValueError('generated map identity mismatch')
        source=yaml.safe_load(self.map_path.read_text())
        semantic_bytes=None
        if self.semantic_path is not None:
            try:semantic_bytes=self.semantic_path.read_bytes()
            except FileNotFoundError:pass
        self.furniture=[]
        if semantic_bytes is not None:
            try:semantic=yaml.safe_load(semantic_bytes)
            except yaml.YAMLError as error:raise ValueError('invalid semantic YAML') from error
            if (not isinstance(semantic,dict) or semantic.get('schema_version')!=1 or semantic.get('source_robot')!=robot_id
                    or semantic.get('frame_id')!='map' or semantic.get('units')!='meters'):
                raise ValueError('invalid semantic map frame/schema')
            for key in ('yaml_sha256','image_sha256'):
                if semantic['source_map'][key]!=info[key]:raise ValueError('semantic source digest mismatch')
            for key,other in (('yaml','yaml_path'),('image','image_path')):
                if Path(semantic['source_map'][key]).resolve()!=Path(info[other]):
                    raise ValueError('semantic source path mismatch')
            self.furniture=semantic['furniture']
        if (source.get('mode','trinary')!='trinary' or source.get('negate') not in (0,1)
                or not valid_numbers(source.get('origin'),3)
                or not valid_numbers([source.get('resolution')],1) or source['resolution']<=0
                or not valid_numbers([source.get('free_thresh'),source.get('occupied_thresh')],2)
                or not 0<=source['free_thresh']<source['occupied_thresh']<=1):
            raise ValueError('valid trinary occupancy map required')
        with Image.open(info['image_path']) as image:
            pixels=np.array(image.convert('L'))
        occupancy=pixels/255. if source['negate'] else 1.-pixels/255.
        # Trinary cells at either threshold are unknown, as in Nav2 map_io.
        self.raw=np.flipud(np.where(occupancy<source['free_thresh'],0,
                                   np.where(occupancy>source['occupied_thresh'],254,255))).copy()
        self.source=dict(source,image=info['image_path'],width=pixels.shape[1],height=pixels.shape[0])
        if not isinstance(self.furniture,list):raise ValueError('furniture list required')
        self.points=[];ids=set()
        for item in self.furniture:
            if not isinstance(item,dict):raise ValueError('furniture must be a mapping')
            fid=item.get('id');poly=item.get('polygon')
            if not isinstance(fid,str) or not fid.strip() or fid in ids:
                raise ValueError('invalid or duplicate furniture ID')
            ids.add(fid)
            if (not isinstance(poly,list) or len(poly)<3 or any(not valid_numbers(p,2) for p in poly)
                    or len({tuple(p) for p in poly})!=len(poly)):
                raise ValueError('finite simple furniture polygon required')
            pairs=list(edges(poly))
            if any(segments_intersect(a,b,c,d) for i,(a,b) in enumerate(pairs)
                   for j,(c,d) in enumerate(pairs) if j>i+1 and not (i==0 and j==len(pairs)-1)):
                raise ValueError('self-intersecting furniture polygon')
            area=sum(a[0]*b[1]-b[0]*a[1] for a,b in pairs)
            if abs(area)<1e-9:raise ValueError('degenerate furniture polygon')
            if type(item.get('enabled',True)) is not bool:raise ValueError('invalid furniture enabled flag')
        for item in self.furniture:
            fid=item['id'];poly=item['polygon'];pairs=list(edges(poly))
            area=sum(a[0]*b[1]-b[0]*a[1] for a,b in pairs)
            if not item.get('enabled',True):continue
            for index,(a,b) in enumerate(pairs,1):
                dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
                nx,ny=(dy/length,-dx/length) if area>0 else (-dy/length,dx/length)
                x,y=(a[0]+b[0])/2+distance*nx,(a[1]+b[1])/2+distance*ny
                if any(path_intersects([Pose2D(x,y)],f['polygon'],self.radius) for f in self.furniture):continue
                mx,my=self.map_xy(x,y)
                if not circle_free(self.raw,mx,my,self.radius,source['resolution'],lambda a:a>=253):continue
                if any(math.hypot(x-p['pose'][0],y-p['pose'][1])<source['resolution'] for p in self.points):continue
                self.points.append(dict(id=f'{fid}/edge_{index}',furniture_id=fid,
                                        pose=[x,y,math.atan2(-ny,-nx)],side=index))
        resolution=source['resolution']
        step=math.ceil(max(1.,2*self.radius)/resolution)
        spacing=step*resolution
        ox,oy,yaw=source['origin'];c,s=math.cos(yaw),math.sin(yaw)
        furniture_points=list(self.points)
        # shortcut: fixed spacing can miss narrow pockets; refine sampling if field coverage requires it.
        for row in range(step//2,self.raw.shape[0],step):
            for col in range(step//2,self.raw.shape[1],step):
                mx,my=(col+.5)*resolution,(row+.5)*resolution
                if not circle_free(self.raw,mx,my,self.radius,resolution,lambda a:a>=253):continue
                x,y=ox+c*mx-s*my,oy+s*mx+c*my
                if any(path_intersects([Pose2D(x,y)],f['polygon'],self.radius) for f in self.furniture):continue
                if any(math.hypot(x-p['pose'][0],y-p['pose'][1])<spacing/2 for p in furniture_points):continue
                self.points.append(dict(id=f'free/{row}/{col}',furniture_id='',pose=[x,y,yaw]))
        self.keepout=configured_mask(robot)
        if self.keepout is not None:
            # Inset candidate centers only; Nav2 needs the original boundary for footprint checks.
            self.points=[p for p in self.points if circle_free(
                self.keepout.free,*self.map_xy(*p['pose'][:2]),self.radius,resolution,
                lambda cells:~cells)]
        self.digests={self.map_path:info['yaml_sha256'],Path(info['image_path']):info['image_sha256']}
        if self.keepout is not None:self.digests.update(self.keepout.digests)
        if self.semantic_path is not None:
            self.digests[self.semantic_path]=hashlib.sha256(semantic_bytes).hexdigest() if semantic_bytes is not None else None

    def observed_points(self,records):
        return [p for p in self.points if records.get(p['id'],{}).get('status')=='observed']

    def uncovered_points(self,records):
        if not self.observed_radius:return self.points
        observed=self.observed_points(records)
        return [p for p in self.points if not any(
            math.hypot(p['pose'][0]-q['pose'][0],p['pose'][1]-q['pose'][1])<=self.observed_radius
            for q in observed)]

    def unchanged(self):
        for path,digest in self.digests.items():
            try:actual=hashlib.sha256(path.read_bytes()).hexdigest()
            except FileNotFoundError:actual=None
            except OSError:return False
            if actual!=digest:return False
        return True

    def map_xy(self,x,y):
        ox,oy,yaw=self.source['origin'];c,s=math.cos(yaw),math.sin(yaw)
        return c*(x-ox)+s*(y-oy),-s*(x-ox)+c*(y-oy)

    def pixel(self,x,y,scale=3):
        mx,my=self.map_xy(x,y);res=self.source['resolution']
        return (mx/res-.5)*scale,(self.source['height']-.5-my/res)*scale


def render_selection(catalog,candidates,records,robot_pose,directory):
    """One overlay is composited onto both maps so every selectable marker matches."""
    scale=3;width,height=catalog.source['width']*scale,catalog.source['height']*scale
    columns=max(1,width//90)
    footer=math.ceil((len(candidates)+1)/columns)*25+80
    occupancy=Image.fromarray(np.flipud(np.where(catalog.raw==0,254,np.where(catalog.raw==255,205,0))).astype('uint8')).convert('RGBA')
    occupancy=occupancy.resize((width,height),getattr(Image,'Resampling',Image).NEAREST)
    bases=[Image.new('RGBA',(width,height+footer),'white') for _ in range(2)]
    bases[0].paste(occupancy,(0,0));bases[1].paste(occupancy,(0,0))
    font_path=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if not font_path.exists():font_path=Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
    font=ImageFont.truetype(str(font_path),14) if font_path.exists() else ImageFont.load_default()
    draw=ImageDraw.Draw(bases[1]);reserved=[]
    for item in catalog.furniture:
        polygon=[catalog.pixel(*p) for p in item['polygon']]
        draw.polygon(polygon,fill=(185,215,190,255),outline=(45,85,55,255))
        cx,cy=sum(p[0] for p in polygon)/len(polygon),sum(p[1] for p in polygon)/len(polygon)
        label=item['id']+'\n'+str(item.get('name') or item.get('category') or '')
        box=draw.multiline_textbbox((cx,cy),label,font=font,anchor='mm',align='center')
        if any(box[0]<b[2] and box[2]>b[0] and box[1]<b[3] and box[3]>b[1] for b in reserved):
            raise ValueError('overlapping furniture labels')
        draw.multiline_text((cx,cy),label,fill='black',font=font,anchor='mm',align='center')
        reserved.append(box)
    overlay=Image.new('RGBA',bases[0].size,(0,0,0,0));draw=ImageDraw.Draw(overlay)
    blue,orange,gray=(30,100,230),(240,140,20),(125,125,125)
    observed=catalog.observed_points(records)
    if catalog.observed_radius:
        coverage=Image.new('RGBA',(width,height),(0,0,0,0));areas=ImageDraw.Draw(coverage)
        radius=catalog.observed_radius/catalog.source['resolution']*scale
        for point in observed:
            x,y=catalog.pixel(*point['pose'][:2])
            areas.ellipse((x-radius,y-radius,x+radius,y+radius),fill=(*gray,35),outline=(*gray,160),width=2)
        overlay.paste(coverage,(0,0))
    rx,ry=catalog.pixel(robot_pose.x,robot_pose.y)
    if not (0<=rx<width and 0<=ry<height):raise ValueError('robot outside map image')
    robot_label=getattr(catalog,'robot_id','go2').capitalize()
    marks=[(rx,ry,robot_label,blue)]
    marks.extend((*catalog.pixel(*p['pose'][:2]),p['display_id'],orange) for p in candidates)
    marks.extend((*catalog.pixel(*p['pose'][:2]),'observed',gray) for p in observed)
    footer_index=0
    for x,y,label,color in marks:
        reserved.append((x-8,y-8,x+8,y+8))
    for x,y,label,color in marks:
        draw.ellipse((x-5,y-5,x+5,y+5),fill=color)
        if label=='observed':continue
        box=None
        # Prefer unused furniture interior, then wide free space so narrow passages stay legible.
        w=draw.textbbox((0,0),label,font=font)[2]+8;h=22
        for interior_only in (True,False):
            for offset in range(15,121,15):
                for dx,dy in ((1,-1),(-1,-1),(1,1),(-1,1)):
                    left,top=x+dx*offset-(w if dx<0 else 0),y+dy*offset
                    trial=(left,top,left+w,top+h)
                    if (left<0 or top<0 or trial[2]>=width or trial[3]>=height
                            or any(left<b[2]+3 and trial[2]+3>b[0] and top<b[3]+3 and trial[3]+3>b[1] for b in reserved)):
                        continue
                    corners=[(left,top),(left+w,top),(left+w,top+h),(left,top+h)]
                    interior=any(all(inside(c,polygon) for c in corners)
                        for polygon in [[catalog.pixel(*p) for p in f['polygon']] for f in catalog.furniture])
                    mx=((left+w/2)/scale+.5)*catalog.source['resolution']
                    my=(catalog.source['height']-.5-(top+h/2)/scale)*catalog.source['resolution']
                    open_space=circle_free(catalog.raw,mx,my,catalog.radius+max(w,h)/scale*catalog.source['resolution']/2,
                                           catalog.source['resolution'],lambda cells:cells>=253)
                    if not (interior if interior_only else open_space):continue
                    box=trial;break
                if box:break
            if box:break
        if box is None:
            row,column=divmod(footer_index,columns);footer_index+=1
            left=10+column*90
            box=(left,height+8+row*25,left+w,height+30+row*25)
        reserved.append(box)
        draw.line((x,y,(box[0]+box[2])/2,(box[1]+box[3])/2),fill=color,width=1)
        draw.rectangle(box,fill=(255,255,255,245))
        draw.text((box[0]+4,box[1]+2),label,fill=color,font=font)
    yaw=robot_pose.yaw-catalog.source['origin'][2]
    ax,ay=rx+22*math.cos(yaw),ry-22*math.sin(yaw)
    draw.line((rx,ry,ax,ay),fill=blue,width=3)
    for sign in (-1,1):
        angle=yaw+sign*.5
        draw.line((ax,ay,ax-8*math.cos(angle),ay+8*math.sin(angle)),fill=blue,width=3)
    observed_legend=(f'Gray disks: observed / excluded within {catalog.observed_radius:g} m'
                     if catalog.observed_radius else 'Gray: observed / cannot select')
    for offset,label,color in ((65,robot_label+': blue circle and heading',blue),(45,'P IDs: orange / selectable',orange),
                               (25,observed_legend,gray)):
        draw.text((10,height+footer-offset),label,fill=color,font=font)
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    paths=[]
    for base,name in zip(bases,('occupancy.png','semantic.png')):
        path=directory/name
        with path.open('xb') as stream:Image.alpha_composite(base,overlay).convert('RGB').save(stream,format='PNG')
        paths.append(str(path))
    return paths


def parse_candidate_selection(raw,allowed):
    def unique(pairs):
        result={}
        for key,value in pairs:
            if key in result:raise ValueError('duplicate candidate field')
            result[key]=value
        return result
    try:reply=json.loads(raw,object_pairs_hook=unique)
    except (TypeError,json.JSONDecodeError) as error:raise ValueError('candidate response must be JSON') from error
    if (not isinstance(reply,dict) or set(reply)!={'candidate_id','reason'}
            or not isinstance(reply['candidate_id'],str) or reply['candidate_id'] not in allowed
            or not isinstance(reply['reason'],str)):
        raise ValueError('invalid or unavailable candidate ID')
    reply['reason']=reply['reason'].strip()[:20]
    return reply


class GeneratedCandidatesVLM:
    def __init__(self,client):self.client=client

    def next_viewpoint(self,snapshot):
        mapping={p['display_id']:p for p in snapshot['candidates']}
        reply=parse_candidate_selection(json.dumps(self.client.choose_candidate(snapshot)),mapping)
        return dict(mapping[reply['candidate_id']],selection_reason=reply['reason'])
