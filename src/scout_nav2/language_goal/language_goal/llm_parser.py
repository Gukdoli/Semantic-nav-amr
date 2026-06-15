"""LLM-backed command parsing (M5) with strict JSON output (Google Gemini).

ROS-free so it can be unit tested with a fake client (see test/test_llm_parser.py).
The node tries this first and falls back to the keyword parser
(command_parser.parse) on ANY failure: no API key, missing SDK, timeout, API
error, or output that doesn't validate. The LLM is used for parsing only.

The API key is read only from the GEMINI_API_KEY environment variable -- never
hard-coded or stored in params (the repo is public). Get a free key at
https://aistudio.google.com/apikey . M5 uses target_label + selector
(nearest/farthest); relation (near/behind/between) is Future Work.
"""

from __future__ import annotations

import json
import os
from typing import Callable, Optional, Sequence

from language_goal.command_parser import ParsedCommand


def _build_prompt(command: str, allowed_labels: Sequence[str]) -> str:
    labels = ", ".join(f"'{label}'" for label in allowed_labels)
    return (
        "Parse a robot navigation command into a JSON object with keys "
        '"target_label" and "selector".\n'
        f"- target_label: one of [{labels}], or \"none\" if the command refers "
        "to no allowed object. Map any synonym or language (incl. Korean).\n"
        '- selector: "nearest" (closest, the default), "farthest", or null if '
        "unspecified.\n"
        'Respond with ONLY the JSON object, e.g. {"target_label": "fire '
        'extinguisher", "selector": "farthest"}.\n\n'
        f"Command: {command!r}"
    )


def _default_client(timeout_s: float):
    """Build a Gemini client from GEMINI_API_KEY, or None if unavailable."""
    if not os.environ.get("GEMINI_API_KEY"):
        return None
    try:
        from google import genai
        from google.genai import types
    except ImportError:
        return None
    # Gemini rejects a request deadline under 10s with 400 INVALID_ARGUMENT, so
    # never send less even if a shorter timeout is configured. Unit: milliseconds.
    deadline_ms = int(max(timeout_s, 10.0) * 1000)
    kwargs = {"api_key": os.environ["GEMINI_API_KEY"]}
    try:  # http_options/timeout shape varies across SDK versions; best-effort.
        kwargs["http_options"] = types.HttpOptions(timeout=deadline_ms)
    except Exception:  # noqa: BLE001
        pass
    return genai.Client(**kwargs)


def _log(logger: Optional[Callable[[str], None]], msg: str) -> None:
    if logger is not None:
        logger(msg)


def parse_with_llm(
    command: str,
    allowed_labels: Sequence[str],
    *,
    model: str,
    timeout_s: float = 5.0,
    max_tokens: int = 256,
    client=None,
    client_factory: Optional[Callable[[float], object]] = None,
    logger: Optional[Callable[[str], None]] = None,
) -> Optional[ParsedCommand]:
    """Parse a free-text command with an LLM, or return None to trigger fallback.

    Returns a ParsedCommand on success, or None on ANY failure (no key, missing
    SDK, API/timeout error, malformed output, or a label outside
    `allowed_labels`) so the caller falls back to the keyword parser. When
    `logger` is given, each failure branch reports its reason through it.
    """
    if not command or not command.strip():
        return None
    if client is None:
        factory = client_factory or _default_client
        client = factory(timeout_s)
    if client is None:
        _log(logger, "no Gemini client (GEMINI_API_KEY unset or google-genai missing)")
        return None

    try:
        config = {
            "response_mime_type": "application/json",
            "max_output_tokens": max_tokens,
        }
        # Disable "thinking" on Gemini 2.5 models (only they support it): this is
        # a trivial extraction, and thinking tokens otherwise eat the output
        # budget so no JSON comes back. 2.0 models reject thinking_config.
        if "2.5" in model:
            config["thinking_config"] = {"thinking_budget": 0}
        resp = client.models.generate_content(
            model=model,
            contents=_build_prompt(command, allowed_labels),
            config=config,
        )
    except Exception as exc:  # noqa: BLE001 -- API/network/timeout error -> fallback
        _log(logger, f"Gemini API error ({model}): {type(exc).__name__}: {exc}")
        return None

    payload = _extract_json(resp)
    if payload is None:
        _log(logger, f"Gemini returned empty/unparseable output: {_resp_text(resp)!r}")
        return None
    parsed = _to_parsed_command(payload, allowed_labels)
    if parsed is None:
        _log(logger, f"Gemini label not in allowed set: payload={payload}")
    return parsed


def _resp_text(resp) -> Optional[str]:
    """Best-effort raw text of a response, for diagnostics (never raises)."""
    try:
        return resp.text
    except Exception:  # noqa: BLE001
        return None


def _extract_json(resp) -> Optional[dict]:
    """Pull the JSON object out of a generate_content response, tolerating fences."""
    text = _resp_text(resp)
    if not text:
        return None
    text = text.strip()
    if text.startswith("```"):  # strip a ```json ... ``` markdown fence
        text = text.strip("`").strip()
        if text.lower().startswith("json"):
            text = text[4:].strip()
    try:
        data = json.loads(text)
    except (ValueError, TypeError):
        return None
    return data if isinstance(data, dict) else None


def _to_parsed_command(
    payload: dict, allowed_labels: Sequence[str]
) -> Optional[ParsedCommand]:
    """Validate the JSON payload and build a ParsedCommand, or None to fall back."""
    label = payload.get("target_label")
    if not isinstance(label, str):
        return None
    # Validate against allowed labels (case-insensitive); 'none'/unknown -> fallback.
    lookup = {lab.lower(): lab for lab in allowed_labels}
    canonical = lookup.get(label.strip().lower())
    if canonical is None:
        return None
    selector = payload.get("selector")
    if selector not in ("nearest", "farthest"):
        selector = None
    relation = payload.get("relation")  # captured but unused (Future Work)
    if relation not in ("near", "behind", "front", "between"):
        relation = None
    return ParsedCommand(
        target_label=canonical, relation=relation, selector=selector
    )
