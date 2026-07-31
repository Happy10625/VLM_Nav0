import copy
import json
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from vlm_nav.vlm_client import (
    OpenAICompatibleVLMClient,
    build_frontier_prompt,
    build_prompt,
    overlay_coordinate_grid,
    validate_frontier_decision,
    validate_vlm_result,
)
from vlm_nav.models import FrontierCandidate, Pixel


def valid_payload():
    return {
        "target_visible": True,
        "confidence": 0.8,
        "target_pixel": {"u": 100, "v": 50},
        "waypoints": [{"u": 80, "v": 100}, {"u": 90, "v": 75}],
    }


def test_valid_response_is_converted_to_immutable_contract():
    result = validate_vlm_result(valid_payload(), width=160, height=120)
    assert result.target_visible
    assert result.target_pixel.u == 100
    assert [(item.u, item.v) for item in result.waypoints] == [(80, 100), (90, 75)]


@pytest.mark.parametrize(
    "mutator",
    [
        lambda data: data.update(extra=True),
        lambda data: data.update(confidence=1.1),
        lambda data: data.update(target_pixel={"u": 160, "v": 50}),
        lambda data: data.update(waypoints=[{"u": 1, "v": 1}] * 4),
        lambda data: data.update(target_visible=False),
        lambda data: data.update(target_pixel={"u": True, "v": 5}),
    ],
)
def test_malformed_or_unsafe_response_is_rejected(mutator):
    payload = valid_payload()
    mutator(payload)
    with pytest.raises(ValueError):
        validate_vlm_result(payload, width=160, height=120)


def test_target_absent_requires_null_target_and_empty_waypoints():
    payload = {
        "target_visible": False,
        "confidence": 0.2,
        "target_pixel": None,
        "waypoints": [],
    }
    result = validate_vlm_result(payload, width=160, height=120)
    assert result.target_pixel is None
    assert not result.waypoints


def test_qwen_normalized_coordinates_are_scaled_to_real_image_pixels():
    payload = {
        "target_visible": True,
        "confidence": 0.95,
        "target_pixel": {"u": 680, "v": 780},
        "waypoints": [{"u": 640, "v": 820}, {"u": 1000, "v": 1000}],
    }
    result = validate_vlm_result(
        payload,
        width=1280,
        height=720,
        coordinate_mode="normalized_1000",
    )
    assert result.target_pixel == Pixel(870, 561)
    assert result.waypoints[-1] == Pixel(1279, 719)
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


def test_grid_overlay_does_not_modify_camera_frame():
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    original = copy.deepcopy(image)
    annotated = overlay_coordinate_grid(image, spacing=40)
    assert annotated.shape == image.shape
    assert np.array_equal(image, original)
    assert np.count_nonzero(annotated) > 0


def test_prompt_explicitly_requires_every_json_field():
    prompt = build_prompt("chair", 1280, 720)
    for field in ("target_visible", "confidence", "target_pixel", "waypoints"):
        assert field in prompt


def test_qwen_prompt_explicitly_uses_normalized_1000_coordinates():
    prompt = build_prompt("chair", 1280, 720, normalized_1000=True)
    assert "normalized 1000x1000 grid" in prompt
    assert "(1000,1000) is bottom-right" in prompt


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
        "target_visible": False,
        "confidence": 0.1,
        "target_pixel": None,
        "waypoints": [],
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
    result = client.infer(np.zeros((120, 160, 3), dtype=np.uint8), "chair")

    assert not result.target_visible
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
