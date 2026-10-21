from multi_agent_nav.resource_manager import ResourceManager
from multi_agent_nav.models import ResourceRequest, ResourceFeedback

def test_fifo_token_identity_and_no_timeout_release():
    m = ResourceManager()
    g = ResourceRequest('t','go2','deposit','basket')
    r = ResourceRequest('t','ranger','deposit2','basket')
    permit = m.request(g, 0)
    assert permit.granted and m.request(g, 1).token == permit.token
    assert not m.request(r, 0).granted
    assert not m.request(ResourceRequest('t','go2','door','door'), 0).granted
    assert not m.update(ResourceFeedback('t','go2','EXIT_NEW','basket',permit.token,'EXIT_CONFIRMED',True,True,True), 2)
    assert not m.update(ResourceFeedback('t','go2','deposit','basket',permit.token,'EXIT_CONFIRMED',True,True,True), 2)
    assert m.update(ResourceFeedback('t','go2','deposit','basket',permit.token,'ENTERED',True), 3)
    assert not m.update(ResourceFeedback('t','go2','deposit','basket',permit.token,'EXIT_CONFIRMED',True,False,True), 4)
    m.freeze('go2','lost')
    assert not m.request(r, 99999).granted
    assert 'basket' in m.held
    assert not m.update(ResourceFeedback('t','go2','deposit','basket',permit.token,'EXIT_CONFIRMED',True,True,True), 5)

def test_exit_after_entry_and_reserved_cancel_require_physical_stop():
    m = ResourceManager()
    req = ResourceRequest('t','go2','d','basket')
    p = m.request(req, 0)
    assert m.update(ResourceFeedback('t','go2','d','basket',p.token,'ENTERED',True), 1)
    assert m.update(ResourceFeedback('t','go2','d','basket',p.token,'EXIT_CONFIRMED',True,True,True), 2)
    p = m.request(ResourceRequest('t','ranger','r','basket'), 3)
    assert p.granted
    assert not m.update(ResourceFeedback('t','ranger','r','basket',p.token,'CANCEL_OUTSIDE',True,True,False), 4)
    assert m.update(ResourceFeedback('t','ranger','r','basket',p.token,'CANCEL_OUTSIDE',True,True,True), 4)
