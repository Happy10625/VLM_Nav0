#!/usr/bin/env python3
"""Replay a saved VLM request without ROS nodes; report full-response latency."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import time


def run_probe(client, request, samples, *, clock=time.perf_counter, emit=print):
    results=[]
    for index in range(1,samples+1):
        row=dict(attempt=index,success=False)
        started=clock()
        try:
            response=client._completion(request['messages'],request['max_tokens'],request.get('requested_schema'))
            elapsed=clock()-started
            content=response.choices[0].message.content
            json.loads(content)
            row.update(success=True,response_content=client._redact_key(content),model=response.model,
                response_id=client._redact_key(response.id),finish_reason=response.choices[0].finish_reason,
                usage=response.usage.model_dump() if response.usage else None)
        except Exception as error:
            elapsed=clock()-started
            row.update(error_type=type(error).__name__,http_status=getattr(error,'status_code',None))
        row.update(elapsed_s=round(elapsed,4),within_4s=row['success'] and elapsed<=4.)
        results.append(row)
        emit(f"{index}/{samples}: {'OK' if row['success'] else row['error_type']} "
             f"{elapsed:.3f}s within_4s={row['within_4s']}")
    durations=[r['elapsed_s'] for r in results if r['success']]
    summary=dict(total=samples,success=len(durations),failed=samples-len(durations),
        within_4s=sum(r['within_4s'] for r in results),
        min_s=min(durations) if durations else None,
        median_s=statistics.median(durations) if durations else None,
        max_s=max(durations) if durations else None)
    return dict(results=results,summary=summary)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request',required=True,type=Path,help='Saved scan request.json, including its image')
    parser.add_argument('--config',required=True,type=Path,help='Field robot_ranger.yaml')
    parser.add_argument('--samples',type=int,default=3)
    parser.add_argument('--timeout',type=float,default=30.,help='SDK timeout for this probe only')
    parser.add_argument('--output',required=True,type=Path,help='New JSON result file; never overwritten')
    args=parser.parse_args()
    if args.samples<1 or not math.isfinite(args.timeout) or args.timeout<=0:
        parser.error('samples and timeout must be positive')
    if args.output.exists():parser.error('output already exists')
    import yaml
    from multi_agent_nav.adapters.vision import ABCClient
    payload=args.request.read_bytes()
    request=json.loads(payload)
    config=yaml.safe_load(args.config.read_text())
    config.update(api_timeout=args.timeout,cancellable_request=False)
    client=ABCClient(config)
    try:
        if client.model!=request['model']:parser.error('configured model differs from saved request model')
        with args.output.open('x') as output:
            print(f'model={client.model} samples={args.samples} timeout={args.timeout}s',flush=True)
            report=run_probe(client,request,args.samples,emit=lambda line:print(line,flush=True))
            report.update(model=client.model,timeout_s=args.timeout,request_file=str(args.request.resolve()),
                request_sha256=hashlib.sha256(payload).hexdigest(),sdk_retries=0)
            json.dump(report,output,ensure_ascii=False,indent=2)
            output.write('\n')
            print(json.dumps(report['summary'],ensure_ascii=False),flush=True)
    finally:
        client.client.close()
    return 0 if report['summary']['failed']==0 else 1


if __name__=='__main__':
    raise SystemExit(main())
