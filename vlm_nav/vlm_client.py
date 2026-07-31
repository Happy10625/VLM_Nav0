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
            "waypoints": {
                "type": "array",
                "minItems": 0,
                "maxItems": 3,
                "items": {
                    "type": "object",
                    "properties": {
                        "u": {"type": "integer"},
                        "v": {"type": "integer"},
                    },
                    "required": ["u", "v"],
                    "additionalProperties": False,
                },
            },
        },
        "required": ["target_visible", "confidence", "target_pixel", "waypoints"],
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
        "confidence",
        "target_pixel",
        "waypoints",
    }:
        raise ValueError("response must contain exactly the documented fields")
    if not isinstance(payload["target_visible"], bool):
        raise ValueError("target_visible must be boolean")
    confidence = payload["confidence"]
    if isinstance(confidence, bool) or not isinstance(confidence, (int, float)):
        raise ValueError("confidence must be numeric")
    confidence = float(confidence)
    if not 0.0 <= confidence <= 1.0:
        raise ValueError("confidence outside [0,1]")

    if coordinate_mode not in ("pixels", "normalized_1000"):
        raise ValueError(f"unsupported coordinate mode: {coordinate_mode}")

    def parse_pixel(value: Any) -> Pixel:
        if not isinstance(value, dict) or set(value) != {"u", "v"}:
            raise ValueError("pixel must contain exactly u and v")
        u, v = value["u"], value["v"]
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
    raw_waypoints = payload["waypoints"]
    if not isinstance(raw_waypoints, list) or len(raw_waypoints) > 3:
        raise ValueError("waypoints must be an array with at most three entries")
    waypoints = tuple(parse_pixel(item) for item in raw_waypoints)
    if not payload["target_visible"] and waypoints:
        raise ValueError("waypoints must be empty when target is not visible")
    return VLMResult(
        target_visible=payload["target_visible"],
        confidence=confidence,
        target_pixel=target,
        waypoints=waypoints,
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


def overlay_coordinate_grid(
    rgb: np.ndarray, spacing: int = 80, normalized_1000: bool = False
) -> np.ndarray:
    """Draw a sparse coordinate grid on a copy, retaining original dimensions."""
    image = np.ascontiguousarray(rgb.copy())
    height, width = image.shape[:2]
    color = (255, 215, 0)
    if normalized_1000:
        for value in range(0, 1001, 100):
            x = int(round(value * max(0, width - 1) / 1000.0))
            y = int(round(value * max(0, height - 1) / 1000.0))
            cv2.line(image, (x, 0), (x, height - 1), color, 1, cv2.LINE_AA)
            cv2.putText(
                image,
                str(value),
                (min(x + 2, max(0, width - 38)), 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.40,
                color,
                1,
            )
            cv2.line(image, (0, y), (width - 1, y), color, 1, cv2.LINE_AA)
            cv2.putText(
                image,
                str(value),
                (2, min(y + 15, max(0, height - 3))),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.40,
                color,
                1,
            )
    else:
        for x in range(0, width, max(20, spacing)):
            cv2.line(image, (x, 0), (x, height - 1), color, 1, cv2.LINE_AA)
            cv2.putText(image, str(x), (min(x + 2, width - 35), 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)
        for y in range(0, height, max(20, spacing)):
            cv2.line(image, (0, y), (width - 1, y), color, 1, cv2.LINE_AA)
            cv2.putText(image, str(y), (2, min(y + 15, height - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)
    return image


def build_prompt(
    target: str, width: int, height: int, normalized_1000: bool = False
) -> str:
    coordinate_instruction = (
        "Qwen coordinate mode is required: express every target_pixel and waypoint on a "
        "normalized 1000x1000 grid with integer u,v in [0,1000], regardless of the "
        f"actual {width}x{height} image size. (0,0) is top-left and (1000,1000) is "
        "bottom-right. The drawn grid labels use this normalized coordinate system. "
        if normalized_1000
        else (
            f"Use actual {width}x{height} image pixel coordinates with origin (0,0) "
            "at top-left. "
        )
    )
    return (
        f"Find this static navigation target: {target!r}. The submitted RGB image is "
        f"{width}x{height} pixels. {coordinate_instruction}Only set target_visible=true "
        "when the target is clearly visible. target_pixel must be a pixel on the visible target. "
        "The JSON object must contain exactly these four fields: target_visible (boolean), "
        "confidence (number from 0.0 to 1.0), target_pixel ({u: integer, v: integer} or null), "
        "and waypoints (an array of {u: integer, v: integer}). "
        "When visible, return 1 to 3 traversable ground pixels leading toward it, ordered nearest "
        "to farthest. Choose clear floor, not walls, objects, stairs, ledges, or the target itself. "
        "When not visible return target_pixel=null and waypoints=[]. Never output velocities, "
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

    def _image_content(
        self,
        rgb: np.ndarray,
        add_grid: bool = False,
        normalized_grid: bool = False,
    ):
        image = (
            overlay_coordinate_grid(rgb, normalized_1000=normalized_grid)
            if add_grid
            else np.ascontiguousarray(rgb)
        )
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
                    self._image_content(
                        rgb, add_grid=True, normalized_grid=self._is_qwen
                    ),
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
