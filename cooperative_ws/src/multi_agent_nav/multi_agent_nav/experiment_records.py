"""Keep ten managed experiment directories, never deleting active or legacy data."""
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import shutil
import time

OWNER='go2-vlm-experiment-v1'


def _lock(path):
    return os.open(path,os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)


def _prune(root):
    rows=[]
    for path in root.iterdir():
        marker=path/'.experiment.json'
        if path.is_symlink() or not path.is_dir() or marker.is_symlink():continue
        try:data=json.loads(marker.read_text())
        except (OSError,ValueError):continue
        if isinstance(data,dict) and data.get('owner')==OWNER and type(data.get('created_ns')) is int:
            rows.append((data['created_ns'],path))
    remaining=len(rows)
    for _,path in sorted(rows):
        if remaining<=10:break
        fd=_lock(path/'.active.lock')
        try:
            try:fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:continue
            shutil.rmtree(path)
            remaining-=1
        finally:os.close(fd)
    if remaining>10:raise RuntimeError('ten experiment records are still active; cannot allocate another')


@contextmanager
def experiment_bundle(path,root):
    path,root=Path(path).absolute(),Path(root).absolute()
    lease={'pending':None}
    if path.parent!=root:
        path.mkdir(parents=True,exist_ok=False)
        yield lease  # Explicit external output remains unmanaged; never prune its siblings.
        return
    root.mkdir(parents=True,exist_ok=True)
    if root.is_symlink():raise ValueError('experiment root must not be a symlink')
    guard=_lock(root/'.retention.lock');fd=None
    try:
        fcntl.flock(guard,fcntl.LOCK_EX)
        path.mkdir(exist_ok=False)
        fd=_lock(path/'.active.lock');fcntl.flock(fd,fcntl.LOCK_EX)
        (path/'.experiment.json').write_text(json.dumps(dict(owner=OWNER,created_ns=time.time_ns()))+'\n')
        try:_prune(root)
        except Exception:
            shutil.rmtree(path)  # Only the just-created, still empty bundle is rolled back.
            raise
    except BaseException:
        if fd is not None:os.close(fd)
        raise
    finally:os.close(guard)
    try:yield lease
    finally:
        job=lease['pending']
        if job is not None and not job.done():job.add_done_callback(lambda _:os.close(fd))
        else:os.close(fd)
