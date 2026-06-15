"""Unit tests for the keyword command parser (no ROS)."""

from language_goal.command_parser import (
    DEFAULT_LABEL_SYNONYMS,
    parse,
)

SYN = {"fire extinguisher": ["fire extinguisher", "extinguisher", "소화기"]}


def test_parse_canonical_phrase():
    parsed = parse("go to the fire extinguisher", SYN)
    assert parsed is not None
    assert parsed.target_label == "fire extinguisher"


def test_parse_is_case_insensitive():
    parsed = parse("GO TO Fire Extinguisher", SYN)
    assert parsed is not None
    assert parsed.target_label == "fire extinguisher"


def test_parse_short_synonym():
    parsed = parse("navigate to the extinguisher please", SYN)
    assert parsed is not None
    assert parsed.target_label == "fire extinguisher"


def test_parse_korean_synonym():
    parsed = parse("소화기 옆으로 가", SYN)
    assert parsed is not None
    assert parsed.target_label == "fire extinguisher"


def test_parse_unknown_returns_none():
    assert parse("go to the chair", SYN) is None


def test_parse_empty_returns_none():
    assert parse("", SYN) is None


def test_default_synonyms_cover_extinguisher():
    assert "fire extinguisher" in DEFAULT_LABEL_SYNONYMS
    parsed = parse("approach the fire extinguisher", DEFAULT_LABEL_SYNONYMS)
    assert parsed is not None
    assert parsed.target_label == "fire extinguisher"


def test_longer_synonym_wins_when_multiple_match():
    # Two labels whose forms both appear; the longer matched form should win.
    syn = {
        "extinguisher": ["extinguisher"],
        "fire extinguisher": ["fire extinguisher"],
    }
    parsed = parse("go to the fire extinguisher", syn)
    assert parsed is not None
    assert parsed.target_label == "fire extinguisher"
