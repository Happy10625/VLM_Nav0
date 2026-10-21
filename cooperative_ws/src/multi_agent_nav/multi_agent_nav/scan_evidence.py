"""Per-request artifacts written by the existing vision worker."""
import base64
from dataclasses import asdict
import json
import os
from pathlib import Path
import time
import uuid

import cv2
import numpy as np
from vlm_nav.arm_image_recorder import ArmImageRecorder


class ScanEvidence:
    def __init__(self,root,metadata,*,directory=None):
        self.directory=Path(directory) if directory else Path(root)/f'{uuid.uuid4().hex}_view_{metadata["view_index"]:02d}'
        self.data=dict(metadata,api_returned=False,validation_success=False)
        self.write_failed=False
        self.input_jpeg=None
        self.started=time.monotonic()

    def begin(self,existing=False):
        try:self.directory.mkdir(parents=True,exist_ok=existing)
        except OSError:
            self.write_failed=True
            raise OSError('VLM evidence directory creation failed') from None
        self._metadata()

    def _write(self,name,content):
        path=self.directory/name
        temporary=self.directory/'metadata.tmp' if name=='metadata.json' else path
        try:
            with temporary.open('wb' if name=='metadata.json' else 'xb') as stream:
                stream.write(content);stream.flush();os.fsync(stream.fileno())
            if name=='metadata.json':temporary.replace(path)
        except OSError:
            self.write_failed=True
            raise OSError('VLM evidence write failed') from None

    def _metadata(self):
        try:content=(json.dumps(self.data,ensure_ascii=False,allow_nan=False,indent=2)+'\n').encode()
        except (TypeError,ValueError):
            self.write_failed=True
            raise OSError('VLM evidence metadata serialization failed') from None
        self._write('metadata.json',content)

    def input(self,content):
        self.input_jpeg=base64.b64decode(content['image_url']['url'].split(',',1)[1],validate=True)
        self._write('input.jpg',self.input_jpeg)
        self.data['input_file']='input.jpg'
        self._metadata()

    def request_started(self,model):
        self.data.update(model=model,api_started_monotonic=time.monotonic())
        self._metadata()

    def request(self,payload):
        self._write('request.json',payload.encode())

    def finish_selection(self,reply,error_type=None):
        self.data.update(validation_success=error_type is None,error_type=error_type,
            completed_monotonic=time.monotonic(),latency_s=time.monotonic()-self.started,
            parsed_selection=reply)
        self._metadata()

    def response(self,info,content):
        self.data.update(info,api_returned=True,api_completed_monotonic=time.monotonic())
        if content is not None:
            self._write('response.txt',content.encode())
            self.data['raw_response_file']='response.txt'
        self._metadata()

    def provider_error(self,info):
        self.data.update(info,api_completed_monotonic=time.monotonic())
        self._metadata()

    def finish(self,results,error_type=None):
        self.data.update(validation_success=error_type is None,error_type=error_type,
            completed_monotonic=time.monotonic(),latency_s=time.monotonic()-self.started,
            parsed_results={target:dict(status=status,grounding=asdict(result))
                for target,(status,result) in results.items()})
        if self.input_jpeg is not None:
            try:
                bgr=cv2.imdecode(np.frombuffer(self.input_jpeg,dtype=np.uint8),cv2.IMREAD_COLOR)
                image=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
                for i,(target,(status,result)) in enumerate(results.items()):
                    ArmImageRecorder.draw_target_annotation(image,result)
                    cv2.putText(image,f'{target}: {status} confidence={result.confidence:.3f}',
                        (10,25+25*i),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,0),1,cv2.LINE_AA)
                if error_type:
                    cv2.putText(image,'API_ERROR: '+error_type,(10,25),
                        cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,0),1,cv2.LINE_AA)
                ok,encoded=cv2.imencode('.png',cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
                if not ok:raise ValueError('PNG encoding failed')
            except (cv2.error,ValueError):
                self.write_failed=True
                raise OSError('VLM annotation write failed') from None
            self._write('result.png',encoded.tobytes())
            self.data['annotated_file']='result.png'
        self._metadata()
