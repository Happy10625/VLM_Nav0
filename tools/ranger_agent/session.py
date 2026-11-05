"""Own one original Ranger search, without changing its navigation contract."""
from pathlib import Path
from contextlib import contextmanager
import copy
import fcntl
import hashlib
import json
import math
import os
import shutil
import signal
import subprocess
import threading
import time
import uuid


def file_digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def framework_environment(search_env):
    """Keep login/desktop context, not ROS imports or unrelated model credentials."""
    keep = {'HOME', 'USER', 'LOGNAME', 'LANG', 'PATH', 'TERM', 'TMPDIR', 'CODEX_HOME',
            'DISPLAY', 'WAYLAND_DISPLAY', 'XAUTHORITY', 'XDG_RUNTIME_DIR', 'DBUS_SESSION_BUS_ADDRESS'}
    return {k: v for k, v in search_env.items() if k in keep or k.startswith('LC_')}


def configuration_digests(directory, strategy=None):
    import yaml
    directory = Path(directory)
    paths = {directory / (n + '.yaml') for n in ('scene', 'robot_ranger', 'robot_go2', 'alignment')}
    robot = yaml.safe_load((directory / 'robot_ranger.yaml').read_text())
    for key in ('observer_config_file', 'nav2_profile', 'map_path', 'keepout_mask_yaml'):
        if robot.get(key):
            paths.add(Path(robot[key]).expanduser().resolve())
    paths.update(Path(p).expanduser().resolve() for p in robot.get('arrival_config_files', []))
    for key in ('map_path', 'keepout_mask_yaml'):
        if robot.get(key):
            p = Path(robot[key]).expanduser().resolve()
            paths.add((p.parent / yaml.safe_load(p.read_text())['image']).resolve())
    strategy = strategy or robot.get('exploration_strategy')
    if strategy == 'semantic_furniture':
        catalog_path = Path(robot['semantic_search']['observation_points']).resolve()
        catalog = yaml.safe_load(catalog_path.read_text())
        paths.add(catalog_path)
        paths.update((catalog_path.parent / catalog[k]).resolve() for k in ('semantic_yaml', 'annotated_image'))
    hashes = {str(p): file_digest(p) for p in sorted(paths)}
    if strategy == 'generated_candidates_vlm' and robot.get('generated_candidates', {}).get('semantic_yaml'):
        path = Path(robot['generated_candidates']['semantic_yaml']).resolve()
        # The original generator permits a missing optional semantic map.
        hashes[str(path)] = file_digest(path) if path.exists() else None
    return hashes


def prepare_spec(options, repo, search_env, *, session_dir=None):
    import yaml
    repo = Path(repo).resolve()
    interpreter = shutil.which('python3', path=search_env.get('PATH', ''))
    if not interpreter or Path(interpreter).resolve() != Path('/usr/bin/python3').resolve():
        raise ValueError('search interpreter must resolve to /usr/bin/python3')
    directory = Path(options.get('config_dir') or search_env.get('RANGER_CONFIG') or
                     repo / 'cooperative_ws/src/multi_agent_nav/config').expanduser().resolve()
    scene = yaml.safe_load((directory / 'scene.yaml').read_text())
    target_id = options.get('target_id', 'B')
    if target_id not in ('A', 'B', 'C'):
        raise ValueError('invalid target-id')
    target = options.get('target')
    if target is None:
        target = scene['targets'][target_id]
    if not isinstance(target, str) or not target.strip():
        raise ValueError('nonblank target required')
    if '\0' in target:
        raise ValueError('target contains NUL')
    effective_timeout = options.get('timeout')
    if effective_timeout is None:
        effective_timeout = scene.get('task_timeout', 600.)
    if not math.isfinite(float(effective_timeout)) or float(effective_timeout) <= 0:
        raise ValueError('positive finite timeout required')
    session_dir = Path(session_dir or repo / 'build/ranger-roborsi-runs' / uuid.uuid4().hex).resolve()
    managed = repo / 'build/ranger-local/experiments'
    if session_dir == managed or managed in session_dir.parents:
        raise ValueError('session must be outside managed experiments')
    return dict(repo=str(repo), session_dir=str(session_dir), session_id=session_dir.name,
                task_id=session_dir.name + '/search', target=target, target_id=target_id,
                config_dir=str(directory), config_digests=configuration_digests(directory, options.get('strategy')),
                search_python=str(Path(interpreter).absolute()), search_env=dict(search_env),
                allow_motion=options.get('allow_motion', False),
                options={k: options[k] for k in ('timeout', 'strategy', 'search_hint')
                         if options.get(k) is not None},
                effective_timeout=float(effective_timeout))


def validate_inputs(spec):
    try:
        unchanged = all((file_digest(p) if Path(p).exists() else None) == digest
                        for p, digest in spec['config_digests'].items())
    except OSError:
        unchanged = False
    if not unchanged:
        raise ValueError('selected configuration changed; session will not start')
    interpreter = shutil.which('python3', path=spec['search_env'].get('PATH', ''))
    if not interpreter or Path(interpreter).resolve() != Path('/usr/bin/python3').resolve():
        raise ValueError('search interpreter changed')


def build_search_argv(spec):
    argv = ['/bin/bash', str(Path(spec['repo']) / 'scripts/start_ranger_single_robot.sh')]
    if spec['allow_motion']:
        argv.append('--allow-motion')
    argv += ['--target-id', spec['target_id'], '--target', spec['target'],
             '--config-dir', spec['config_dir'], '--output-dir', str(Path(spec['session_dir']) / 'search')]
    for key, value in spec['options'].items():
        argv += ['--' + key.replace('_', '-'), str(value)]
    return argv


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name('.' + path.name + '.' + uuid.uuid4().hex)
    try:
        fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as stream:
            json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        temporary.unlink(missing_ok=True)


def process_identity(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return dict(pid=pid, pgid=int(fields[2]), start_ticks=int(fields[19]),
                    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())
    except (OSError, ValueError, IndexError):
        return None


class SearchSession:
    def __init__(self, spec, *, spawn=None, identity=process_identity, clock=time.monotonic,
                 sleep=time.sleep, send_signal=os.killpg):
        self.spec = copy.deepcopy(spec)
        self.directory = Path(spec['session_dir'])
        self.directory.mkdir(parents=True, exist_ok=True, mode=0o700)
        self._spawn = spawn
        self._identity = identity
        self._clock, self._sleep, self._send_signal = clock, sleep, send_signal
        self._lock = threading.RLock()
        self.process = None
        self._memory = None
        self._storage_failed = False
        self._event_offset = 0
        self._event_tail = b''
        self._events = []
        self._event_inode = None
        self._event_error = False
        with self._locked():
            manifest = self.directory / 'manifest.json'
            public = {k: v for k, v in spec.items() if k in (
                'repo', 'session_id', 'session_dir', 'task_id', 'target', 'target_id', 'config_dir',
                'config_digests', 'search_python', 'options', 'effective_timeout', 'source_identity')}
            if manifest.exists():
                if json.loads(manifest.read_text()) != public:
                    raise ValueError('session identity/spec mismatch')
            else:
                atomic_json(manifest, public)
            if not (self.directory / 'session.json').exists():
                self._save(dict(session_id=spec['session_id'], task_id=spec['task_id'], state='IDLE',
                    consumed=False, process_identity=None, process_exit_code=None, executor_result=None,
                    stop_status='NOT_STARTED', executor_success=False, evidence_status='UNKNOWN',
                    framework_error=None, active_process=False, stop_confirmed=None,
                    evidence_write_failed=None, vlm_evidence_pending=None, cancel_requested_at=None,
                    evidence_path=str(self.directory / 'search'), reason='not started'))

    @contextmanager
    def _locked(self):
        with self._lock:
            fd = os.open(self.directory / '.session.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
            try:
                fcntl.flock(fd, fcntl.LOCK_EX)
                yield
            finally:
                os.close(fd)

    def _load(self):
        if self._storage_failed and self.process is not None:
            return copy.deepcopy(self._memory)
        try:
            return json.loads((self.directory / 'session.json').read_text())
        except (OSError, ValueError):
            if self.process is None or self._memory is None:
                raise
            self._storage_failed = True
            return copy.deepcopy(self._memory)

    def _save(self, row):
        self._memory = copy.deepcopy(row)
        try:
            atomic_json(self.directory / 'session.json', row)
        except OSError:
            self._storage_failed = True
            raise

    def _persist_owned(self, row, *, final=False):
        try:
            if self._storage_failed:
                row.update(evidence_status='INCOMPLETE',
                           framework_error=row.get('framework_error') or 'HOST_STATE_IO_FAILED')
            self._save(row)
            if final:
                atomic_json(self.directory / 'result.json', row)
        except OSError:
            # Loss of host evidence must not prevent one SIGINT to our owned process.
            self._storage_failed = True
            row.update(evidence_status='INCOMPLETE',
                       framework_error=row.get('framework_error') or 'HOST_STATE_IO_FAILED')
            self._memory = copy.deepcopy(row)

    def start(self, target):
        if target != self.spec['target']:
            raise ValueError('target must equal fixed target verbatim')
        with self._locked():
            row = self._load()
            if row['consumed']:
                return self._refresh(row)
            if not self.spec['allow_motion']:
                raise ValueError('original --allow-motion is required')
            validate_inputs(self.spec)
            if (self.directory / 'search').exists():
                raise ValueError('search output directory already exists; refusing old evidence')
            if self._spawn is None:
                if self.spec.get('framework_config', {}).get('real_backend_enabled') is not True:
                    raise ValueError('real backend is disabled pending stage B/C authorization')
                from .runtime import verify_real_backend
                verify_real_backend(self.spec)
            argv = build_search_argv(self.spec)
            row.update(consumed=True, state='STARTING', started_at=time.time(), argv=argv,
                       active_process=None, stop_status='UNKNOWN', reason='start consumed before spawn')
            self._save(row)
            try:
                with (self.directory / 'executor.log').open('ab') as log:
                    self.process = (self._spawn or subprocess.Popen)(argv, cwd=self.spec['repo'],
                        env=dict(self.spec['search_env']), shell=False, start_new_session=True,
                        stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, close_fds=True)
            except OSError as error:
                row.update(state='FINISHED', active_process=False, stop_status='NOT_STARTED',
                           reason='SPAWN_FAILED:' + type(error).__name__)
                self._save(row)
                return row
            # A failure after Popen is ambiguous; the consumed record must survive it.
            row.update(state='RUNNING', active_process=True, reason='search running',
                       process_identity=self._identity(self.process.pid), pid=self.process.pid)
            self._persist_owned(row)
            return self._refresh(row)

    def _read_events(self):
        path = self.directory / 'search/events.jsonl'
        try:
            stat = path.stat()
            if ((self._event_inode is not None and self._event_inode != (stat.st_dev, stat.st_ino))
                    or stat.st_size < self._event_offset):
                self._event_error = True
                return
            self._event_inode = stat.st_dev, stat.st_ino
            with path.open('rb') as stream:
                stream.seek(self._event_offset)
                data = self._event_tail + stream.read()
                self._event_offset = stream.tell()
            lines = data.split(b'\n')
            self._event_tail = lines.pop()
            for line in lines:
                self._events.append(json.loads(line))
        except (OSError, ValueError):
            self._event_error = True

    def _finish(self, row):
        self._read_events()
        report = None
        try:
            report = json.loads((self.directory / 'search/result.json').read_text())
            if not isinstance(report, dict):
                report = None
        except (OSError, ValueError):
            pass
        report = report or {}
        for key in ('stop_confirmed', 'evidence_write_failed', 'vlm_evidence_pending'):
            row[key] = report.get(key)
        row['executor_result'] = report.get('result')
        report_valid = (report.get('target_id') == self.spec['target_id']
                        and report.get('mode') == 'local_ranger_explore')
        valid = not self._event_error and not self._event_tail and bool(self._events) and report_valid
        run_id, sequence = None, 0
        for event in self._events:
            if not isinstance(event, dict):
                valid = False
                continue
            run_id = run_id or event.get('run_id')
            seq = event.get('sequence')
            valid = valid and bool(run_id) and event.get('run_id') == run_id
            valid = valid and type(seq) is int and seq > sequence
            valid = valid and event.get('target_id') == self.spec['target_id']
            valid = valid and event.get('target') == self.spec['target']
            if type(seq) is int:
                sequence = seq
        end = self._events[-1] if self._events and isinstance(self._events[-1], dict) else {}
        valid = (valid and end.get('event') == 'SESSION_END'
                 and end.get('result') == report.get('result')
                 and end.get('stop_confirmed') is report.get('stop_confirmed'))
        row['run_id'] = run_id
        row['executor_success'] = bool(report_valid and row['process_exit_code'] == 0
            and report.get('result') == 'TARGET_APPROACHED' and report.get('stop_confirmed') is True)
        row['stop_status'] = 'CONFIRMED' if report_valid and report.get('stop_confirmed') is True else 'UNKNOWN'
        row['evidence_status'] = 'INCOMPLETE'
        if (valid and self._references_readable() and report.get('stop_confirmed') is not None
                and report.get('evidence_write_failed') is False and report.get('vlm_evidence_pending') is False):
            discarded = any(e.get('consumer_status') in ('TIMEOUT', 'CANCELED')
                            for e in self._events if isinstance(e, dict))
            row['evidence_status'] = 'UNKNOWN' if discarded else 'REPORTED_OK'
        row.update(state='FINISHED' if row['stop_status'] == 'CONFIRMED' else 'STOP_UNCONFIRMED',
                   reason='executor exited; evidence is reported, not independently certified')
        self._persist_owned(row, final=True)

    def _references_readable(self):
        root = (self.directory / 'search').resolve()
        def readable(value, *, input_asset=False):
            path = Path(value).resolve()
            if root not in path.parents:
                return (input_asset and str(path) in self.spec['config_digests']
                        and file_digest(path) == self.spec['config_digests'][str(path)])
            with path.open('rb') as stream:
                stream.read(1)
            return True
        try:
            for event in self._events:
                for key in ('evidence_dir', 'evidence_directory'):
                    if event.get(key):
                        directory = Path(event[key])
                        if not readable(directory / 'metadata.json'):
                            return False
                        data = json.loads((directory / 'metadata.json').read_text())
                        for name in ('input_file', 'raw_response_file', 'annotated_file'):
                            if data.get(name) and not readable(directory / data[name]):
                                return False
                images = event.get('images', [])
                if event.get('image') and not readable(event['image'], input_asset=True):
                    return False
                if any(not readable(p) for p in images):
                    return False
            return True
        except (OSError, ValueError, TypeError, AttributeError):
            return False

    def _refresh(self, row):
        if self.process is not None:
            code = self.process.poll()
            row['active_process'] = code is None
            row['process_exit_code'] = code
            if code is not None:
                self._finish(row)
            else:
                path = self.directory / 'search/events.jsonl'
                if path.exists():
                    self._read_events()
                if row.get('cancel_deadline') is not None and self._clock() >= row['cancel_deadline']:
                    row.update(state='STOP_UNCONFIRMED', stop_status='UNKNOWN', reason='cancel wait expired')
        elif row['consumed'] and row['state'] not in ('FINISHED',):
            identity = row.get('process_identity')
            current = self._identity(identity['pid']) if identity else None
            row.update(active_process=True if identity and current == identity else None,
                       stop_status='UNKNOWN', state='STOP_UNCONFIRMED',
                       reason='persisted session; this host does not own the previous process')
            return row  # A reader/restarted host must not overwrite the live owner's state.
        self._persist_owned(row) if self.process is not None else self._save(row)
        return row

    def status(self, task_id):
        if task_id != self.spec['task_id']:
            raise ValueError('unknown task_id')
        with self._locked():
            return self._refresh(self._load())

    def cancel(self, task_id):
        if task_id != self.spec['task_id']:
            raise ValueError('unknown task_id')
        with self._locked():
            row = self._refresh(self._load())
            if not row['consumed'] or row['active_process'] is False or row['cancel_requested_at'] is not None:
                return row
            expected = row.get('process_identity')
            if (self.process is None or expected is None or expected['pgid'] != self.process.pid
                    or self._identity(self.process.pid) != expected):
                row.update(state='STOP_UNCONFIRMED', stop_status='UNKNOWN',
                           reason='process identity unverified; no signal sent')
                if self.process is not None:
                    self._persist_owned(row)
                return row
            row.update(state='CANCEL_REQUESTED', cancel_requested_at=time.time(),
                       cancel_deadline=self._clock() + 30., reason='SIGINT requested; stop not yet confirmed')
            self._persist_owned(row)  # Memory also fences repeats if disk persistence fails.
            try:
                self._send_signal(expected['pgid'], signal.SIGINT)
            except OSError as error:
                row.update(state='STOP_UNCONFIRMED', reason='signal delivery unknown:' + type(error).__name__)
                self._persist_owned(row)
            return row

    def close(self):
        row = self.cancel(self.spec['task_id'])
        while row['active_process'] is True and row['state'] == 'CANCEL_REQUESTED':
            self._sleep(.1)
            row = self.status(self.spec['task_id'])
        with self._locked():
            row = self._refresh(self._load())
            if not row['consumed']:
                row.update(state='FINISHED', executor_result='AGENT_NO_EXECUTION', reason='agent did not start search')
                self._save(row)
            self._persist_owned(row, final=True)
        return row

    def framework_failed(self, reason):
        with self._locked():
            row = self._load()
            row['framework_error'] = reason
            self._persist_owned(row)
