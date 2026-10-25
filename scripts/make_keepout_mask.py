#!/usr/bin/env python3
"""Generate a map-aligned keepout mask from human-selected map-frame polygon vertices."""
import argparse
from pathlib import Path
import re
import yaml
from vlm_nav.keepout import KeepoutMask,write_mask


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config-dir',required=True)
    parser.add_argument('--robot',choices=('ranger','go2'),default='ranger')
    parser.add_argument('--polygon',required=True,help='Markdown containing one txt/yaml ROS PointStamped stream')
    parser.add_argument('--output-dir',required=True)
    args=parser.parse_args(argv)
    robot=yaml.safe_load((Path(args.config_dir)/f'robot_{args.robot}.yaml').read_text())
    blocks=re.findall(r'```(?:txt|yaml)\s*\n(.*?)```',Path(args.polygon).read_text(),re.S)
    if len(blocks)!=1:raise ValueError('expected one PointStamped stream in polygon document')
    vertices=[]
    for sample in yaml.safe_load_all(blocks[0]):
        if sample is None:continue
        if sample['header']['frame_id']!='map':raise ValueError('polygon vertices must use this robot map frame')
        vertices.append([sample['point']['x'],sample['point']['y']])
    # Nav2 checks the padded footprint; the mask describes the site boundary itself.
    margin=0.
    output=write_mask(robot['map_path'],vertices,Path(args.output_dir)/'keepout_mask.yaml',margin)
    mask=KeepoutMask(robot['map_path'],output)
    print(yaml.safe_dump(dict(keepout_mask_yaml=str(output),keepout_mask_digest=mask.digest,
        boundary_margin_m=margin,vertices=len(vertices),allowed_cells=int(mask.free.sum())),sort_keys=False))
    return 0


if __name__=='__main__':raise SystemExit(main())
