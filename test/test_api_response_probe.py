import importlib.util
from pathlib import Path
from types import SimpleNamespace as S


def module():
    spec=importlib.util.spec_from_file_location('api_response_probe',Path(__file__).parents[1]/'scripts/api_response_probe.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    return mod


def test_replays_saved_request_and_measures_full_response():
    probe=module();calls=[];times=iter([10.,17.,20.,23.])
    response=S(model='qwen-test',id='request-id',usage=S(model_dump=lambda:{'completion_tokens':42}),
        choices=[S(message=S(content='{"B":"NOT_DETECTED"}'),finish_reason='stop')])
    client=S(_completion=lambda *args:(calls.append(args) or response),_redact_key=lambda x:x)
    request=dict(messages=[{'role':'user','content':'same request'}],max_tokens=1200,requested_schema={'name':'abc'})
    report=probe.run_probe(client,request,2,clock=lambda:next(times),emit=lambda _:None)
    assert calls==[(request['messages'],1200,request['requested_schema'])]*2
    assert [r['elapsed_s'] for r in report['results']]==[7.,3.]
    assert report['summary']['success']==2 and report['summary']['within_4s']==1
    assert report['summary']['median_s']==5.
    assert report['results'][0]['response_content']=='{"B":"NOT_DETECTED"}'


def test_errors_are_counted_without_exposing_provider_exception():
    probe=module();times=iter([0.,4.]);output=[]
    def fail(*args):raise RuntimeError('secret-key-in-provider-error')
    client=S(_completion=fail,_redact_key=lambda x:x)
    report=probe.run_probe(client,dict(messages=[],max_tokens=1200),1,clock=lambda:next(times),emit=output.append)
    assert report['summary']['success']==0 and report['summary']['failed']==1
    assert report['results'][0]['error_type']=='RuntimeError'
    assert 'secret-key' not in str(report)+str(output)
