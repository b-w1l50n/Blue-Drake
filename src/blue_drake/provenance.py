"""Shared parameter-provenance vocabulary for public model configuration."""

from __future__ import annotations

from enum import StrEnum


class ParameterProvenance(StrEnum):
    """Declared origin of behavior-defining model parameters."""

    PUBLISHED = "published"
    MEASURED = "measured"
    FITTED = "fitted"
    ASSUMED = "assumed"


def validate_parameter_provenance(
    provenance: ParameterProvenance | str,
    source_urls: tuple[str, ...],
) -> tuple[ParameterProvenance, tuple[str, ...]]:
    """Normalize provenance and require traceable non-assumed sources."""

    try:
        normalized = ParameterProvenance(provenance)
    except ValueError as exc:
        raise ValueError(
            f"unsupported parameter provenance: {provenance}"
        ) from exc
    urls = tuple(str(url).strip() for url in source_urls)
    if any(not url.startswith("https://") for url in urls):
        raise ValueError("parameter source URLs must use https")
    if normalized is not ParameterProvenance.ASSUMED and not urls:
        raise ValueError(f"{normalized} parameters require a source URL")
    return normalized, urls
