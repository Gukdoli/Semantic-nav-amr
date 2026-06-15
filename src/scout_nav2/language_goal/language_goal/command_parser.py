"""Pure-function natural-language command parsing (M4: keyword matching).

Deliberately free of ROS imports so the parser can be unit tested standalone
(see test/test_command_parser.py). The node layer (goal_commander_node.py)
feeds `parse` with a label->synonyms mapping and dispatches on the result.

M4 is keyword matching only (single demo class, the fire extinguisher). The
`relation` field is parsed-but-unused scaffolding for M5 (spatial relations
near/behind/between) and the planned LLM parser.
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
    relation: Optional[str] = None  # M5: near/behind/between; unused in M4.


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
