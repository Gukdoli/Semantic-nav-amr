"""Pure-function natural-language command parsing.

Deliberately free of ROS imports so the parser can be unit tested standalone
(see test/test_command_parser.py). The node layer (goal_commander_node.py)
feeds `parse` with a label->synonyms mapping and dispatches on the result.

`parse` here is the keyword matcher: it is both the M4 baseline and the M5
fallback when the LLM parser (llm_parser.py) is disabled or fails. The LLM
parser produces the same ParsedCommand, so the node treats both uniformly.

ParsedCommand fields:
  - target_label: canonical label to navigate to (set by both parsers).
  - selector: which instance to pick when several share the label
    ("nearest"/"farthest"); set by the LLM parser, None for keyword matching.
  - relation: spatial relation (near/behind/between) -- captured in the LLM
    schema but unused (Future Work); always None from the keyword parser.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence


# Default label -> surface-form synonyms. Kept here (not in the parser body and
# not in a ROS param) so M4 stays simple for the single demo class; the node
# injects this into `parse`. A proper per-label synonym param structure is M5.
DEFAULT_LABEL_SYNONYMS: Dict[str, List[str]] = {
    "fire extinguisher": ["fire extinguisher", "extinguisher", "소화기"],
}


@dataclass
class ParsedCommand:
    target_label: str
    relation: Optional[str] = None  # near/behind/between; unused (Future Work).
    selector: Optional[str] = None  # "nearest"/"farthest" instance selection.


def _normalize(text: str) -> str:
    return " ".join(text.lower().split())


def parse(
    command: str,
    label_synonyms: Dict[str, Sequence[str]],
) -> Optional[ParsedCommand]:
    """Resolve a free-text command to a target label by synonym matching.

    `label_synonyms` maps a canonical label to its surface forms; the command
    is matched case-insensitively by substring against every form. Returns a
    ParsedCommand with the canonical label, or None if no known label is found
    (caller reports this as "could not understand the command").

    When several labels match, the one whose matched synonym is longest wins
    (more specific phrase), so e.g. "fire extinguisher" beats a bare token.
    """
    text = _normalize(command)
    best_label: Optional[str] = None
    best_len = -1
    for label, forms in label_synonyms.items():
        for form in forms:
            f = _normalize(form)
            if f and f in text and len(f) > best_len:
                best_label = label
                best_len = len(f)
    if best_label is None:
        return None
    return ParsedCommand(target_label=best_label)
