"""Stable identifier validation shared by public Blue Drake configuration."""

from __future__ import annotations

import re

_IDENTIFIER = re.compile(r"[A-Za-z][A-Za-z0-9_-]{0,63}")


def validate_identifier(name: str, value: str) -> str:
    """Return a safe stable identifier or raise ``ValueError``."""

    if not isinstance(value, str) or _IDENTIFIER.fullmatch(value) is None:
        raise ValueError(
            f"{name} must start with a letter and contain at most 64 "
            "letters, digits, underscores, or hyphens"
        )
    return value
