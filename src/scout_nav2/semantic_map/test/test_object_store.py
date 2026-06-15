"""Unit tests for ObjectStore data association (no ROS)."""

import pytest

from semantic_map.object_store import ObjectStore


def test_new_object_added_when_no_match():
    store = ObjectStore(merge_distance=0.5)
    store.update("chair", (1.0, 1.0, 0.0), 0.9, 10.0)
    assert len(store.all()) == 1


def test_close_same_label_merges_with_ema():
    store = ObjectStore(merge_distance=0.5, ema_alpha=0.5)
    store.update("chair", (1.0, 0.0, 0.0), 0.8, 1.0)
    store.update("chair", (1.2, 0.0, 0.0), 0.9, 2.0)
    assert len(store.all()) == 1
    obj = store.all()[0]
    # EMA with alpha=0.5: 0.5*1.0 + 0.5*1.2 = 1.1
    assert obj.x == pytest.approx(1.1)
    assert obj.confidence == pytest.approx(0.9)
    assert obj.last_seen == pytest.approx(2.0)
    assert obj.count == 2


def test_far_same_label_creates_new_object():
    store = ObjectStore(merge_distance=0.5)
    store.update("chair", (0.0, 0.0, 0.0), 0.8, 1.0)
    store.update("chair", (5.0, 0.0, 0.0), 0.8, 2.0)
    assert len(store.all()) == 2


def test_same_position_different_label_not_merged():
    store = ObjectStore(merge_distance=0.5)
    store.update("chair", (0.0, 0.0, 0.0), 0.8, 1.0)
    store.update("fire extinguisher", (0.0, 0.0, 0.0), 0.8, 2.0)
    assert len(store.all()) == 2


def test_find_returns_all_instances_sorted_by_confidence():
    store = ObjectStore(merge_distance=0.5)
    store.update("chair", (0.0, 0.0, 0.0), 0.6, 1.0)
    store.update("chair", (5.0, 0.0, 0.0), 0.95, 2.0)  # separate object (5m away)
    matches = store.find("chair")
    assert len(matches) == 2
    assert matches[0].confidence == pytest.approx(0.95)  # highest first
    assert matches[1].confidence == pytest.approx(0.6)


def test_find_missing_label_returns_empty():
    store = ObjectStore()
    assert store.find("nonexistent") == []


def test_find_min_count_filters_unconfirmed():
    store = ObjectStore(merge_distance=0.5)
    # Seen 3 times -> confirmed.
    for t in range(3):
        store.update("chair", (0.0, 0.0, 0.0), 0.8, float(t))
    # Seen once -> unconfirmed (e.g. a one-off false positive 5m away).
    store.update("chair", (5.0, 0.0, 0.0), 0.8, 9.0)
    assert len(store.find("chair", min_count=1)) == 2
    confirmed = store.find("chair", min_count=3)
    assert len(confirmed) == 1
    assert confirmed[0].count == 3


def test_save_load_roundtrip(tmp_path):
    path = str(tmp_path / "map.json")
    store = ObjectStore(merge_distance=0.5)
    for t in range(3):
        store.update("fire extinguisher", (0.67, 5.77, 0.0), 0.8, float(t))
    store.update("fire extinguisher", (0.67, 1.77, 0.0), 0.7, 9.0)  # separate
    assert store.save(path) == 2

    restored = ObjectStore(merge_distance=0.5)
    assert restored.load(path) == 2
    matches = restored.find("fire extinguisher", min_count=1)
    assert len(matches) == 2
    confirmed = restored.find("fire extinguisher", min_count=3)
    assert len(confirmed) == 1  # count survives the round-trip
    assert confirmed[0].x == pytest.approx(0.67)
    assert confirmed[0].y == pytest.approx(5.77)


def test_load_missing_file_returns_zero(tmp_path):
    store = ObjectStore()
    assert store.load(str(tmp_path / "nope.json")) == 0
    assert store.all() == []


def test_two_far_same_label_both_confirmable():
    """Instance separation: two fire extinguishers >3m apart stay distinct."""
    store = ObjectStore(merge_distance=0.5)
    for t in range(3):
        store.update("fire extinguisher", (0.0, 0.0, 0.0), 0.7, float(t))
        store.update("fire extinguisher", (3.5, 0.0, 0.0), 0.7, float(t))
    matches = store.find("fire extinguisher", min_count=3)
    assert len(matches) == 2
    xs = sorted(round(m.x, 1) for m in matches)
    assert xs == [0.0, 3.5]
