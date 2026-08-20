"""OpenAI-compatible vision client with strict local result validation."""

import base64
import json
import math
import os
from typing import Any, Dict, Iterable

import cv2
import httpx
import numpy as np

from .models import FrontierDecision, Pixel, VLMResult


DEFAULT_DASHSCOPE_MODEL = "qwen3-vl-flash"


RESPONSE_SCHEMA: Dict[str, Any] = {
    "name": "vlm_navigation_grounding",
    "strict": True,
    "schema": {
        "type": "object",
        "properties": {
            "target_visible": {"type": "boolean"},
            "object_match": {"type": "boolean"},
            "qualifier_match": {"type": "boolean"},
            "relation_match": {"type": "boolean"},
            "confidence": {"type": "number", "minimum": 0.0, "maximum": 1.0},
            "target_pixel": {
                "anyOf": [
                    {
                        "type": "object",
                        "properties": {
                            "u": {"type": "integer"},
                            "v": {"type": "integer"},
                        },
                        "required": ["u", "v"],
                        "additionalProperties": False,
                    },
                    {"type": "null"},
                ]
            },
            "evidence_pixel": {
                "anyOf": [
                    {
                        "type": "object",
                        "properties": {
                            "u": {"type": "integer"},
                            "v": {"type": "integer"},
                        },
                        "required": ["u", "v"],
                        "additionalProperties": False,
                    },
                    {"type": "null"},
                ]
            },
        },
        "required": [
            "target_visible",
            "object_match",
            "qualifier_match",
            "relation_match",
            "confidence",
            "target_pixel",
            "evidence_pixel",
        ],
        "additionalProperties": False,
    },
}


def validate_vlm_result(
    payload: Any,
    width: int,
    height: int,
    coordinate_mode: str = "pixels",
) -> VLMResult:
    if not isinstance(payload, dict) or set(payload) != {
        "target_visible",
        "object_match",
        "qualifier_match",
        "relation_match",
        "confidence",
        "target_pixel",
        "evidence_pixel",
    }:
        raise ValueError("response must contain exactly the documented fields")
    if not isinstance(payload["target_visible"], bool):
        raise ValueError("target_visible must be boolean")
    semantic_fields = (
        "object_match",
        "qualifier_match",
        "relation_match",
    )
    if any(not isinstance(payload[field], bool) for field in semantic_fields):
        raise ValueError("semantic match fields must be boolean")
    semantic_match = all(payload[field] for field in semantic_fields)
    if payload["target_visible"] != semantic_match:
        raise ValueError(
            "target_visible must agree with all semantic match fields"
        )
    confidence = payload["confidence"]
    if isinstance(confidence, bool) or not isinstance(confidence, (int, float)):
        raise ValueError("confidence must be numeric")
    confidence = float(confidence)
    if not 0.0 <= confidence <= 1.0:
        raise ValueError("confidence outside [0,1]")

    if coordinate_mode not in ("pixels", "normalized_1000"):
        raise ValueError(f"unsupported coordinate mode: {coordinate_mode}")

    def parse_pixel(value: Any) -> Pixel:
        if isinstance(value, dict) and set(value) == {"u", "v"}:
            u, v = value["u"], value["v"]
        elif isinstance(value, (list, tuple)) and len(value) == 2:
            u, v = value
        else:
            raise ValueError("pixel must contain exactly u and v")
        if isinstance(u, bool) or isinstance(v, bool) or not isinstance(u, int) or not isinstance(v, int):
            raise ValueError("pixel coordinates must be integers")
        if coordinate_mode == "normalized_1000":
            if not (0 <= u <= 1000 and 0 <= v <= 1000):
                raise ValueError("normalized coordinate outside [0,1000]")
            return Pixel(
                u=int(round(u * max(0, width - 1) / 1000.0)),
                v=int(round(v * max(0, height - 1) / 1000.0)),
            )
        if not (0 <= u < width and 0 <= v < height):
            raise ValueError("pixel outside submitted image")
        return Pixel(u=u, v=v)

    raw_target = payload["target_pixel"]
    target = None if raw_target is None else parse_pixel(raw_target)
    if payload["target_visible"] != (target is not None):
        raise ValueError("target_visible and target_pixel disagree")
    raw_evidence = payload["evidence_pixel"]
    evidence = None if raw_evidence is None else parse_pixel(raw_evidence)
    if payload["target_visible"] != (evidence is not None):
        raise ValueError("target_visible and evidence_pixel disagree")
    return VLMResult(
        target_visible=payload["target_visible"],
        object_match=payload["object_match"],
        qualifier_match=payload["qualifier_match"],
        relation_match=payload["relation_match"],
        confidence=confidence,
        target_pixel=target,
        evidence_pixel=evidence,
        coordinate_mode=coordinate_mode,
    )


def validate_frontier_decision(payload: Any, valid_ids: Iterable[int]) -> FrontierDecision:
    if not isinstance(payload, dict) or set(payload) != {
        "selected_frontier_id",
        "confidence",
        "reason",
    }:
        raise ValueError("frontier response must contain exactly the documented fields")
    selected = payload["selected_frontier_id"]
    if isinstance(selected, bool) or not isinstance(selected, int):
        raise ValueError("selected_frontier_id must be an integer")
    if selected not in set(int(item) for item in valid_ids):
        raise ValueError("selected_frontier_id is not an offered candidate")
    confidence = payload["confidence"]
    if isinstance(confidence, bool) or not isinstance(confidence, (int, float)):
        raise ValueError("frontier confidence must be numeric")
    confidence = float(confidence)
    if not 0.0 <= confidence <= 1.0:
        raise ValueError("frontier confidence outside [0,1]")
    reason = payload["reason"]
    if not isinstance(reason, str) or not reason.strip():
        raise ValueError("frontier reason must be a non-empty string")
    # The reason is diagnostic text, not a motion-safety field. Providers can
    # exceed a requested character limit by a few characters, so keep the
    # valid frontier decision and truncate the display/log text locally.
    return FrontierDecision(selected, confidence, reason.strip()[:199])


def build_prompt(
    target: str, width: int, height: int, normalized_1000: bool = False
) -> str:
    coordinate_instruction = (
        "Use the normalized 1000x1000 output coordinate system for every "
        "target_pixel and evidence_pixel while preserving the original "
        f"{width}x{height} image and its aspect ratio. (0,0) is top-left and "
        "(1000,1000) is bottom-right. "
        if normalized_1000
        else (
            f"Use actual {width}x{height} image pixel coordinates with origin (0,0) "
            f"at top-left: u must be in [0,{max(0, width - 1)}] and v must be in "
            f"[0,{max(0, height - 1)}]. "
        )
    )
    target_lower = target.lower()
    cola_hard_negative = (
        "For a cola qualifier, cola means a visually identifiable cola drink such as "
        "Coca-Cola or Pepsi; a transparent kettle or ordinary water bottle on a chair "
        "is a hard negative, not a cola-chair match. "
        if any(token in target_lower for token in ("可乐", "cola", "coke"))
        else ""
    )
    return (
        f"Find this static navigation target: {target!r}. The submitted RGB image is "
        f"{width}x{height} pixels. {coordinate_instruction}Interpret every explicit object, "
        "attribute, associated object, identity, and spatial relation in the target description "
        "as constraints joined by logical AND. Examine every plausible main-object candidate; "
        "do not stop at the first object of the right category. Set object_match=true only when "
        "the main object category matches. Set qualifier_match=true only when every required "
        "attribute, identity, or associated object is clearly and specifically visible; set it "
        "to true when the description has no qualifier. Set relation_match=true only when every "
        "required relation is visibly satisfied; set it to true when no relation is required. "
        "If you cannot reliably identify the qualifier because it is small, occluded, ambiguous, "
        "or unreadable, set qualifier_match=false. Never guess from a similar container, color, "
        "shape, or scene context. target_pixel and evidence_pixel must belong to the same candidate "
        "combination and visibly satisfy the required relation. They may be on distinct objects and image regions "
        "when those exact instances visibly satisfy the requested relation. Never pair the main object from "
        "one candidate combination with a qualifier from another. "
        f"{cola_hard_negative}target_visible must equal the logical AND of object_match, "
        "qualifier_match, and relation_match: set target_visible=true if and only if all three are true. "
        "confidence is the confidence that the complete target "
        "description is simultaneously satisfied, not confidence in the main object category alone. "
        "target_pixel must be on solid visible material of the main navigation object. evidence_pixel "
        "must be on solid visible material of the most discriminative required qualifier, or on the "
        "main object when no qualifier exists. Never place either pixel in holes, cutouts, gaps, or openings, "
        "or on background visible through the target. "
        "The JSON object must contain exactly these seven fields: target_visible (boolean), "
        "object_match (boolean), qualifier_match (boolean), relation_match (boolean), confidence "
        "(number from 0.0 to 1.0), target_pixel ({u: integer, v: integer} or null), evidence_pixel "
        "({u: integer, v: integer} or null). Each non-null pixel must be a JSON object "
        "with exactly the keys u and v, never an array. When not visible return target_pixel=null and "
        "evidence_pixel=null. Never output velocities, "
        "steering commands, prose, or fields outside the schema. Return exactly one valid JSON "
        "object matching the requested fields."
    )


def build_frontier_prompt(target: str, candidates) -> str:
    lines = [
        (
            f"The robot is searching for {target!r}, but it was not found in the latest scan. "
            "Image 1 is the current camera view or an eight-view scan montage. Image 2 is a "
            "north-up occupancy map: white is known free space, black is occupied, gray is "
            "unknown, the orange arrow is the robot, and blue numbered circles are safe "
            "reachable frontier candidates. Select the one candidate that is most useful for "
            "visually searching for the target while preferring open, informative space. "
            "Do not invent coordinates or select an ID outside the list."
        ),
        "Candidates:",
    ]
    for item in candidates:
        lines.append(
            f"- ID {item.candidate_id}: distance={item.distance:.2f}m, "
            f"bearing={math.degrees(item.bearing):.0f}deg, frontier_cells={item.cell_count}"
        )
    lines.append(
        "Return exactly one JSON object with selected_frontier_id (integer), confidence "
        "(0.0 to 1.0), and reason. reason must be one short sentence strictly under "
        "200 characters; do not repeat all candidate statistics."
    )
    return "\n".join(lines)


class OpenAICompatibleVLMClient:
    def __init__(self, timeout_s: float = 4.0, image_detail: str = "high", jpeg_quality: int = 85):
        try:
            from openai import OpenAI
        except ImportError as error:
            raise RuntimeError("Python package 'openai' is required for VLM requests") from error

        # Prefer the provider-specific variables so a stale OPENAI_API_KEY or
        # OPENAI_BASE_URL cannot silently route Qwen images to the old gateway.
        dashscope_key = os.getenv("DASHSCOPE_API_KEY")
        api_key = dashscope_key or os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise RuntimeError("DASHSCOPE_API_KEY (or OPENAI_API_KEY) is not set")

        if dashscope_key:
            workspace_id = os.getenv("DASHSCOPE_WORKSPACE_ID")
            workspace_url = (
                f"https://{workspace_id}.cn-beijing.maas.aliyuncs.com/compatible-mode/v1"
                if workspace_id
                else None
            )
            base_url = os.getenv("DASHSCOPE_BASE_URL") or workspace_url
            if not base_url:
                raise RuntimeError(
                    "DASHSCOPE_BASE_URL or DASHSCOPE_WORKSPACE_ID is not set"
                )
            self.model = os.getenv("DASHSCOPE_MODEL") or DEFAULT_DASHSCOPE_MODEL
        else:
            base_url = (
                os.getenv("OPENAI_BASE_URL")
                or "https://api.chatanywhere.tech/v1"
            )
            self.model = os.getenv("OPENAI_MODEL", "gpt-4o-mini")
        self._is_qwen = (
            bool(dashscope_key)
            or "aliyuncs.com" in base_url.lower()
            or self.model.lower().startswith("qwen")
        )
        proxy_url = os.getenv("HTTPS_PROXY") or os.getenv("https_proxy") or None
        http_client = httpx.Client(
            proxy=proxy_url,
            timeout=float(timeout_s),
            trust_env=False,
        )
        # This node already retries across fresh camera frames and stops after
        # api_failure_limit consecutive failures. SDK-level retries would keep
        # an old frame in flight for several timeout periods.
        self.client = OpenAI(
            api_key=api_key,
            base_url=base_url,
            timeout=float(timeout_s),
            max_retries=0,
            http_client=http_client,
        )
        self.image_detail = image_detail
        self.jpeg_quality = int(jpeg_quality)
        self.last_raw_response = None
        # DashScope documents json_object, but not OpenAI's json_schema form.
        self._structured_output_supported = not self._is_qwen

    def _image_content(self, rgb: np.ndarray):
        image = np.ascontiguousarray(rgb)
        ok, encoded = cv2.imencode(
            ".jpg",
            cv2.cvtColor(image, cv2.COLOR_RGB2BGR),
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        )
        if not ok:
            raise RuntimeError("JPEG encoding failed")
        content = {
            "type": "image_url",
            "image_url": {
                "url": "data:image/jpeg;base64,"
                + base64.b64encode(encoded.tobytes()).decode("ascii")
            },
        }
        if not self._is_qwen:
            content["image_url"]["detail"] = self.image_detail
        return content

    def _completion(self, messages, max_tokens: int, schema=None):
        kwargs = {
            "model": self.model,
            "messages": messages,
            "temperature": 0.0,
            "max_tokens": int(max_tokens),
        }
        if self._is_qwen:
            kwargs["response_format"] = {"type": "json_object"}
            kwargs["extra_body"] = {"enable_thinking": False}
        if schema is not None and self._structured_output_supported:
            kwargs["response_format"] = {"type": "json_schema", "json_schema": schema}
            try:
                return self.client.chat.completions.create(**kwargs)
            except Exception as error:
                message = str(error).lower()
                if not any(
                    marker in message
                    for marker in ("response_format", "json_schema", "unsupported", "not support")
                ):
                    raise
                self._structured_output_supported = False
                kwargs["response_format"] = {"type": "json_object"}
        else:
            kwargs.setdefault("response_format", {"type": "json_object"})
        return self.client.chat.completions.create(**kwargs)

    def infer(self, rgb: np.ndarray, target: str) -> VLMResult:
        self.last_raw_response = None
        height, width = rgb.shape[:2]
        messages = [
            {
                "role": "system",
                "content": "You are a conservative visual grounding component for a mobile robot.",
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": build_prompt(
                            target, width, height, normalized_1000=self._is_qwen
                        ),
                    },
                    self._image_content(rgb),
                ],
            },
        ]
        response = self._completion(messages, 180, RESPONSE_SCHEMA)
        content = response.choices[0].message.content
        if not content:
            raise ValueError("VLM returned empty content")
        self.last_raw_response = content
        return validate_vlm_result(
            json.loads(content),
            width,
            height,
            coordinate_mode=(
                "normalized_1000" if self._is_qwen else "pixels"
            ),
        )

    def infer_frontier(self, scene_rgb, map_rgb, candidates, target):
        self.last_raw_response = None
        messages = [
            {
                "role": "system",
                "content": "You are a conservative semantic frontier selector for a mobile robot.",
            },
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": build_frontier_prompt(target, candidates)},
                    self._image_content(scene_rgb),
                    self._image_content(map_rgb),
                ],
            },
        ]
        response = self._completion(messages, 220)
        content = response.choices[0].message.content
        if not content:
            raise ValueError("VLM returned empty frontier content")
        self.last_raw_response = content
        return validate_frontier_decision(
            json.loads(content), [item.candidate_id for item in candidates]
        )
