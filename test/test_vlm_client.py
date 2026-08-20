import base64
import json
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import cv2

from vlm_nav.vlm_client import (
    OpenAICompatibleVLMClient,
    build_frontier_prompt,
    build_prompt,
    validate_frontier_decision,
    validate_vlm_result,
)
from vlm_nav.models import FrontierCandidate, Pixel


def valid_payload():
    return {
        "target_visible": True,
        "object_match": True,
        "qualifier_match": True,
        "relation_match": True,
        "confidence": 0.8,
        "target_pixel": {"u": 100, "v": 50},
        "evidence_pixel": {"u": 110, "v": 35},
    }


def test_target_response_contract_does_not_require_vlm_waypoints():
    payload = {
        "target_visible": True,
        "object_match": True,
        "qualifier_match": True,
        "relation_match": True,
        "confidence": 0.8,
        "target_pixel": {"u": 100, "v": 50},
        "evidence_pixel": {"u": 110, "v": 35},
    }

    result = validate_vlm_result(payload, width=160, height=120)

    assert result.target_pixel == Pixel(100, 50)
    assert result.evidence_pixel == Pixel(110, 35)
    assert not hasattr(result, "waypoints")


def test_valid_response_is_converted_to_immutable_contract():
    result = validate_vlm_result(valid_payload(), width=160, height=120)
    assert result.target_visible
    assert result.object_match
    assert result.qualifier_match
    assert result.relation_match
    assert result.target_pixel.u == 100
    assert result.evidence_pixel == Pixel(110, 35)


def test_two_element_pixel_arrays_are_parsed_safely():
    payload = valid_payload()
    payload["target_pixel"] = [100, 50]
    payload["evidence_pixel"] = [110, 35]

    result = validate_vlm_result(payload, width=160, height=120)

    assert result.target_pixel == Pixel(100, 50)
    assert result.evidence_pixel == Pixel(110, 35)


@pytest.mark.parametrize(
    "mutator",
    [
        lambda data: data.update(extra=True),
        lambda data: data.update(waypoints=[]),
        lambda data: data.update(confidence=1.1),
        lambda data: data.update(target_pixel={"u": 160, "v": 50}),
        lambda data: data.update(target_visible=False),
        lambda data: data.update(target_pixel={"u": True, "v": 5}),
        lambda data: data.update(target_pixel=[100, 50, 25]),
        lambda data: data.update(target_pixel=[100, "50"]),
    ],
)
def test_malformed_or_unsafe_response_is_rejected(mutator):
    payload = valid_payload()
    mutator(payload)
    with pytest.raises(ValueError):
        validate_vlm_result(payload, width=160, height=120)


def test_target_absent_requires_null_target_and_evidence_pixels():
    payload = {
        "target_visible": False,
        "object_match": True,
        "qualifier_match": False,
        "relation_match": False,
        "confidence": 0.2,
        "target_pixel": None,
        "evidence_pixel": None,
    }
    result = validate_vlm_result(payload, width=160, height=120)
    assert result.target_pixel is None


@pytest.mark.parametrize(
    "field", ["object_match", "qualifier_match", "relation_match"]
)
def test_visible_composite_target_requires_every_semantic_match(field):
    payload = valid_payload()
    payload[field] = False

    with pytest.raises(ValueError, match="semantic match fields"):
        validate_vlm_result(payload, width=160, height=120)


def test_visible_composite_target_requires_a_distinct_evidence_contract():
    payload = valid_payload()
    payload["evidence_pixel"] = None

    with pytest.raises(ValueError, match="evidence_pixel"):
        validate_vlm_result(payload, width=160, height=120)


def test_absent_target_cannot_claim_all_semantic_matches():
    payload = valid_payload()
    payload.update(
        target_visible=False,
        target_pixel=None,
        evidence_pixel=None,
    )

    with pytest.raises(ValueError, match="semantic match fields"):
        validate_vlm_result(payload, width=160, height=120)


def test_actual_pixel_coordinate_outside_image_is_rejected():
    payload = valid_payload()
    payload["target_pixel"] = {"u": 1280, "v": 500}
    with pytest.raises(ValueError, match="outside submitted image"):
        validate_vlm_result(payload, width=1280, height=720)


def test_qwen_normalized_coordinates_are_converted_to_real_image_pixels():
    payload = valid_payload()
    payload["target_pixel"] = {"u": 824, "v": 876}
    payload["evidence_pixel"] = {"u": 752, "v": 812}

    result = validate_vlm_result(
        payload,
        width=1280,
        height=720,
        coordinate_mode="normalized_1000",
    )

    assert result.target_pixel == Pixel(1054, 630)
    assert result.evidence_pixel == Pixel(962, 584)
    assert result.coordinate_mode == "normalized_1000"


def test_qwen_normalized_coordinate_outside_1000_is_rejected():
    payload = valid_payload()
    payload["target_pixel"] = {"u": 1001, "v": 500}

    with pytest.raises(ValueError, match="normalized coordinate"):
        validate_vlm_result(
            payload,
            width=1280,
            height=720,
            coordinate_mode="normalized_1000",
        )


def test_prompt_explicitly_requires_every_json_field():
    prompt = build_prompt("chair", 1280, 720)
    for field in (
        "target_visible",
        "object_match",
        "qualifier_match",
        "relation_match",
        "confidence",
        "target_pixel",
        "evidence_pixel",
    ):
        assert field in prompt
    assert "waypoint" not in prompt.lower()


def test_prompt_requires_complete_same_candidate_composite_evidence():
    prompt = build_prompt("放了可乐的椅子", 1280, 720)

    assert "logical AND" in prompt
    assert "same candidate combination" in prompt
    assert "may be on distinct objects and image regions" in prompt
    assert "one candidate combination with a qualifier from another" in prompt
    assert "if and only if all three are true" in prompt
    assert "complete target description" in prompt


def test_prompt_rejects_unreliable_qualifier_guessing_before_hard_negative():
    prompt = build_prompt("放了可乐的椅子", 1280, 720)

    general_rule = "cannot reliably identify the qualifier"
    hard_negative = "transparent kettle or ordinary water bottle"
    assert general_rule in prompt
    assert hard_negative in prompt
    assert prompt.index(general_rule) < prompt.index(hard_negative)


def test_prompt_requires_target_pixel_on_solid_material_not_cutout_background():
    prompt = build_prompt("chair with a cutout backrest", 1280, 720)

    assert "solid visible material" in prompt
    assert "holes, cutouts, gaps, or openings" in prompt
    assert "background visible through the target" in prompt


def test_qwen_prompt_explicitly_requests_normalized_output_coordinates():
    prompt = build_prompt("chair", 1280, 720, normalized_1000=True)
    assert "normalized 1000x1000 output coordinate system" in prompt
    assert "original 1280x720 image" in prompt
    assert "never an array" in prompt


def test_frontier_decision_accepts_only_offered_id_and_bounded_reason():
    result = validate_frontier_decision(
        {
            "selected_frontier_id": 2,
            "confidence": 0.82,
            "reason": "open and informative",
        },
        valid_ids=[1, 2, 4],
    )
    assert result.selected_frontier_id == 2
    with pytest.raises(ValueError):
        validate_frontier_decision(
            {
                "selected_frontier_id": 3,
                "confidence": 0.82,
                "reason": "not offered",
            },
            valid_ids=[1, 2, 4],
        )


def test_frontier_reason_over_limit_is_truncated_without_rejecting_decision():
    result = validate_frontier_decision(
        {
            "selected_frontier_id": 1,
            "confidence": 0.85,
            "reason": "x" * 260,
        },
        valid_ids=[1],
    )
    assert result.selected_frontier_id == 1
    assert len(result.reason) == 199


def test_frontier_prompt_lists_only_safe_candidates():
    candidates = [
        FrontierCandidate(1, 1.0, 0.0, 0.0, 1.0, 12),
        FrontierCandidate(4, 0.0, 2.0, 1.57, 2.0, 20),
    ]
    prompt = build_frontier_prompt("chair", candidates)
    assert "ID 1" in prompt and "ID 4" in prompt
    assert "ID 2" not in prompt


def test_dashscope_qwen_request_uses_non_thinking_json_mode(monkeypatch):
    captured = {}
    response_payload = {
        "target_visible": True,
        "object_match": True,
        "qualifier_match": True,
        "relation_match": True,
        "confidence": 0.9,
        "target_pixel": {"u": 824, "v": 876},
        "evidence_pixel": {"u": 752, "v": 812},
    }

    class FakeCompletions:
        def create(self, **kwargs):
            captured.update(kwargs)
            return SimpleNamespace(
                choices=[
                    SimpleNamespace(
                        message=SimpleNamespace(content=json.dumps(response_payload))
                    )
                ]
            )

    class FakeOpenAI:
        def __init__(self, **kwargs):
            captured["client"] = kwargs
            self.chat = SimpleNamespace(completions=FakeCompletions())

    monkeypatch.setitem(sys.modules, "openai", SimpleNamespace(OpenAI=FakeOpenAI))
    monkeypatch.setenv("DASHSCOPE_API_KEY", "test-key")
    monkeypatch.setenv("DASHSCOPE_WORKSPACE_ID", "ws-test")
    monkeypatch.delenv("DASHSCOPE_BASE_URL", raising=False)
    monkeypatch.setenv("OPENAI_API_KEY", "stale-key")
    monkeypatch.setenv("OPENAI_BASE_URL", "https://old-gateway.example/v1")
    monkeypatch.setenv("OPENAI_MODEL", "old-model")
    monkeypatch.delenv("DASHSCOPE_MODEL", raising=False)
    monkeypatch.delenv("HTTPS_PROXY", raising=False)
    monkeypatch.delenv("https_proxy", raising=False)

    client = OpenAICompatibleVLMClient()
    result = client.infer(np.zeros((720, 1280, 3), dtype=np.uint8), "chair")

    assert result.target_pixel == Pixel(1054, 630)
    assert result.evidence_pixel == Pixel(962, 584)
    assert result.coordinate_mode == "normalized_1000"
    assert client.model == "qwen3-vl-flash"
    assert captured["client"]["base_url"] == (
        "https://ws-test.cn-beijing.maas.aliyuncs.com/compatible-mode/v1"
    )
    assert captured["response_format"] == {"type": "json_object"}
    assert captured["extra_body"] == {"enable_thinking": False}
    assert json.loads(client.last_raw_response) == response_payload
    image_url = captured["messages"][1]["content"][1]["image_url"]
    assert image_url["url"].startswith("data:image/jpeg;base64,")
    assert "detail" not in image_url
    prompt = captured["messages"][1]["content"][0]["text"]
    assert "normalized 1000x1000 output coordinate system" in prompt
    assert "original 1280x720 image" in prompt


def test_dashscope_qwen_request_sends_image_without_coordinate_grid(monkeypatch):
    captured = {}
    response_payload = {
        "target_visible": False,
        "object_match": False,
        "qualifier_match": True,
        "relation_match": True,
        "confidence": 0.1,
        "target_pixel": None,
        "evidence_pixel": None,
    }

    class FakeCompletions:
        def create(self, **kwargs):
            captured.update(kwargs)
            return SimpleNamespace(
                choices=[
                    SimpleNamespace(
                        message=SimpleNamespace(content=json.dumps(response_payload))
                    )
                ]
            )

    class FakeOpenAI:
        def __init__(self, **_kwargs):
            self.chat = SimpleNamespace(completions=FakeCompletions())

    monkeypatch.setitem(sys.modules, "openai", SimpleNamespace(OpenAI=FakeOpenAI))
    monkeypatch.setenv("DASHSCOPE_API_KEY", "test-key")
    monkeypatch.setenv("DASHSCOPE_WORKSPACE_ID", "ws-test")
    monkeypatch.delenv("HTTPS_PROXY", raising=False)
    monkeypatch.delenv("https_proxy", raising=False)
    client = OpenAICompatibleVLMClient(jpeg_quality=100)

    client.infer(np.zeros((720, 1280, 3), dtype=np.uint8), "chair")

    data_url = captured["messages"][1]["content"][1]["image_url"]["url"]
    encoded = base64.b64decode(data_url.split(",", 1)[1])
    sent = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert sent.shape[:2] == (720, 1280)
    assert np.count_nonzero(sent) == 0


def test_qwen_frontier_request_sends_scene_and_map_images(monkeypatch):
    captured = {}
    payload = {
        "selected_frontier_id": 1,
        "confidence": 0.9,
        "reason": "largest visible open route",
    }

    class FakeCompletions:
        def create(self, **kwargs):
            captured.update(kwargs)
            return SimpleNamespace(
                choices=[
                    SimpleNamespace(
                        message=SimpleNamespace(content=json.dumps(payload))
                    )
                ]
            )

    class FakeOpenAI:
        def __init__(self, **_kwargs):
            self.chat = SimpleNamespace(completions=FakeCompletions())

    monkeypatch.setitem(sys.modules, "openai", SimpleNamespace(OpenAI=FakeOpenAI))
    monkeypatch.setenv("DASHSCOPE_API_KEY", "test-key")
    monkeypatch.setenv("DASHSCOPE_WORKSPACE_ID", "ws-test")
    monkeypatch.delenv("HTTPS_PROXY", raising=False)
    monkeypatch.delenv("https_proxy", raising=False)
    client = OpenAICompatibleVLMClient()
    candidates = [FrontierCandidate(1, 1.0, 0.0, 0.0, 1.0, 12)]

    result = client.infer_frontier(
        np.zeros((360, 1280, 3), dtype=np.uint8),
        np.zeros((720, 720, 3), dtype=np.uint8),
        candidates,
        "chair",
    )

    assert result.selected_frontier_id == 1
    content = captured["messages"][1]["content"]
    assert len([item for item in content if item["type"] == "image_url"]) == 2
    assert captured["extra_body"] == {"enable_thinking": False}
