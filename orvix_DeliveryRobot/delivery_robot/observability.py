"""Structured logging + log replay.

Two outputs by design:
- Console (human-readable text, INFO+) — for operator-facing terminal.
- JSONL file (one JSON object per line, all levels) — for machine
  post-processing and incident replay.

`setup_logging()` is idempotent — call once at program start.

Each module uses standard `logging.getLogger(__name__)`. To attach
structured payloads to a log record, pass `extra={"event": {...}}`. The
JSON formatter merges that into the emitted line.

Example:
    log.info("decision",
             extra={"event": {"state": "walking", "progress_m": 42}})

Replay:
    for record in replay_jsonl("logs/2026-04-26.jsonl"):
        if record.get("event", {}).get("state") == "off_route":
            ...
"""
from __future__ import annotations

import json
import logging
import sys
import time
from pathlib import Path
from typing import Any, Iterator, Optional, Union


_CONFIGURED = False


class JsonLineFormatter(logging.Formatter):
    """One JSON object per log line. Keys: ts, level, logger, msg, event?, exc?"""

    def format(self, record: logging.LogRecord) -> str:
        payload: dict[str, Any] = {
            "ts": record.created,
            "level": record.levelname,
            "logger": record.name,
            "msg": record.getMessage(),
        }
        event = getattr(record, "event", None)
        if event is not None:
            payload["event"] = event
        if record.exc_info:
            payload["exc"] = self.formatException(record.exc_info)
        return json.dumps(payload, default=_json_default, ensure_ascii=False)


class HumanFormatter(logging.Formatter):
    """Console formatter: timestamp + level + logger + msg + key=value extras."""

    def format(self, record: logging.LogRecord) -> str:
        ts = time.strftime("%H:%M:%S", time.localtime(record.created))
        head = f"{ts} {record.levelname:5} {record.name:36} {record.getMessage()}"
        event = getattr(record, "event", None)
        if isinstance(event, dict) and event:
            tail = " ".join(f"{k}={_compact(v)}" for k, v in event.items())
            head = f"{head}  [{tail}]"
        if record.exc_info:
            head = f"{head}\n{self.formatException(record.exc_info)}"
        return head


def _json_default(o: Any) -> Any:
    """Best-effort JSON serializer for dataclasses, enums, points, etc."""
    if hasattr(o, "value"):  # Enum
        return o.value
    if hasattr(o, "__dict__"):
        return {k: v for k, v in vars(o).items() if not k.startswith("_")}
    return repr(o)


def _compact(v: Any) -> str:
    if isinstance(v, float):
        return f"{v:.2f}"
    if isinstance(v, str):
        return v
    return repr(v)


def setup_logging(
    json_path: Optional[Union[str, Path]] = None,
    console_level: str = "INFO",
    file_level: str = "DEBUG",
    quiet_libraries: bool = True,
) -> None:
    """Configure root logging. Safe to call multiple times — only acts once.

    Parameters
    ----------
    json_path : path or None
        File to receive JSONL records. None disables file logging.
    console_level / file_level : "DEBUG"/"INFO"/"WARNING"/"ERROR"
        Independent thresholds.
    quiet_libraries : bool
        Suppress noisy 3rd-party loggers (urllib3, ultralytics, matplotlib).
    """
    global _CONFIGURED
    if _CONFIGURED:
        return

    root = logging.getLogger()
    root.setLevel(logging.DEBUG)

    console = logging.StreamHandler(sys.stderr)
    console.setLevel(getattr(logging, console_level.upper()))
    console.setFormatter(HumanFormatter())
    root.addHandler(console)

    if json_path is not None:
        Path(json_path).parent.mkdir(parents=True, exist_ok=True)
        fh = logging.FileHandler(json_path, encoding="utf-8")
        fh.setLevel(getattr(logging, file_level.upper()))
        fh.setFormatter(JsonLineFormatter())
        root.addHandler(fh)

    if quiet_libraries:
        for noisy in ("urllib3", "ultralytics", "matplotlib", "PIL"):
            logging.getLogger(noisy).setLevel(logging.WARNING)

    _CONFIGURED = True


def log_event(logger: logging.Logger, level: str, message: str, **fields: Any) -> None:
    """Convenience: emit a record with a structured `event` payload."""
    logger.log(getattr(logging, level.upper()), message, extra={"event": fields})


def replay_jsonl(path: Union[str, Path]) -> Iterator[dict]:
    """Yield each JSON record from a JSONL log file. Skips malformed lines."""
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue
