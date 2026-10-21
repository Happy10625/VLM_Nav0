from concurrent.futures import Future
from types import SimpleNamespace as S

from multi_agent_nav import go2_check


class ParameterService:
    def __init__(self):
        self.ready = False
        self.requests = []
        self.pending = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        assert request.names == ['yaml_filename']
        future = Future()
        self.requests.append(request)
        self.pending.append(future)
        return future

    def remove_pending_request(self, future):
        self.pending.remove(future)


def test_map_request_waits_for_service_discovery():
    service = ParameterService()
    read = go2_check.MapParameterRead(service, S(names=['yaml_filename']))
    read.poll(0.)
    assert not service.requests and read.path is None
    service.ready = True
    read.poll(.5)
    service.pending[-1].set_result(S(values=[S(string_value='/tmp/field.yaml')]))
    read.poll(.6)
    assert read.path == '/tmp/field.yaml'
    assert read.attempts == 1


def test_lost_response_is_removed_then_retried_without_accepting_late_data():
    service = ParameterService()
    service.ready = True
    read = go2_check.MapParameterRead(service, S(names=['yaml_filename']))
    read.poll(0.)
    old = service.pending[0]
    read.poll(2.1)
    read.poll(3.)
    assert old not in service.pending and old.cancelled()
    service.pending[-1].set_result(S(values=[S(string_value='/tmp/current.yaml')]))
    read.poll(3.1)
    assert read.path == '/tmp/current.yaml' and read.attempts == 2


def test_empty_response_is_retried_and_success_is_not_requested_again():
    service = ParameterService()
    service.ready = True
    read = go2_check.MapParameterRead(service, S(names=['yaml_filename']))
    read.poll(0.)
    service.pending[-1].set_result(S(values=[]))
    read.poll(.1)
    assert read.path is None
    read.poll(1.)
    service.pending[-1].set_result(S(values=[S(string_value='/tmp/map.yaml')]))
    read.poll(1.1)
    read.poll(10.)
    assert read.path == '/tmp/map.yaml' and read.attempts == 2


def test_failed_parameter_response_is_retried():
    service = ParameterService()
    service.ready = True
    read = go2_check.MapParameterRead(service, S(names=['yaml_filename']))
    read.poll(0.)
    service.pending[-1].set_exception(RuntimeError('transport failed'))
    read.poll(.1)
    read.poll(1.)
    service.pending[-1].set_result(S(values=[S(string_value='/tmp/map.yaml')]))
    read.poll(1.1)
    assert read.path == '/tmp/map.yaml'
