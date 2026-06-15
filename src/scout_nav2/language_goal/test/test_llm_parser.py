"""Unit tests for the LLM command parser (no network; fake Gemini client)."""

from language_goal.llm_parser import parse_with_llm

LABELS = ["fire extinguisher"]


class _Resp:
    def __init__(self, text):
        self.text = text


class _Models:
    def __init__(self, resp=None, raise_exc=None):
        self._resp = resp
        self._raise = raise_exc
        self.kwargs = None

    def generate_content(self, **kwargs):
        self.kwargs = kwargs
        if self._raise is not None:
            raise self._raise
        return self._resp


class _Client:
    def __init__(self, text=None, raise_exc=None):
        self.models = _Models(_Resp(text) if text is not None else None, raise_exc)


def test_parse_returns_label_and_selector():
    client = _Client('{"target_label": "fire extinguisher", "selector": "farthest"}')
    parsed = parse_with_llm("go to the far extinguisher", LABELS, model="m", client=client)
    assert parsed is not None
    assert parsed.target_label == "fire extinguisher"
    assert parsed.selector == "farthest"


def test_label_is_validated_case_insensitively():
    client = _Client('{"target_label": "Fire Extinguisher"}')
    parsed = parse_with_llm("go", LABELS, model="m", client=client)
    assert parsed is not None
    assert parsed.target_label == "fire extinguisher"
    assert parsed.selector is None


def test_json_in_markdown_fence_is_parsed():
    client = _Client('```json\n{"target_label": "fire extinguisher"}\n```')
    parsed = parse_with_llm("go", LABELS, model="m", client=client)
    assert parsed is not None
    assert parsed.target_label == "fire extinguisher"


def test_unknown_label_returns_none():
    client = _Client('{"target_label": "none"}')
    assert parse_with_llm("dance", LABELS, model="m", client=client) is None


def test_label_outside_allowed_returns_none():
    client = _Client('{"target_label": "chair"}')
    assert parse_with_llm("go to chair", LABELS, model="m", client=client) is None


def test_invalid_selector_is_dropped():
    client = _Client('{"target_label": "fire extinguisher", "selector": "weird"}')
    parsed = parse_with_llm("go", LABELS, model="m", client=client)
    assert parsed is not None
    assert parsed.selector is None


def test_malformed_json_returns_none():
    client = _Client("not json at all")
    assert parse_with_llm("go", LABELS, model="m", client=client) is None


def test_api_error_returns_none():
    client = _Client(raise_exc=RuntimeError("boom"))
    assert parse_with_llm("go", LABELS, model="m", client=client) is None


def test_empty_response_returns_none():
    client = _Client(text="")
    assert parse_with_llm("go", LABELS, model="m", client=client) is None


def test_missing_client_returns_none():
    # client_factory yields no client (e.g. no API key / SDK) -> fallback signal.
    assert parse_with_llm("go", LABELS, model="m", client_factory=lambda t: None) is None


def test_empty_command_returns_none():
    client = _Client('{"target_label": "fire extinguisher"}')
    assert parse_with_llm("   ", LABELS, model="m", client=client) is None
