"""Logging + replay tests."""
import json
import logging
from pathlib import Path

import pytest

from delivery_robot.observability import (
    JsonLineFormatter,
    log_event,
    replay_jsonl,
)


def test_json_formatter_basic():
    fmt = JsonLineFormatter()
    record = logging.LogRecord(
        name="test", level=logging.INFO, pathname="x", lineno=1,
        msg="hello", args=(), exc_info=None,
    )
    out = fmt.format(record)
    obj = json.loads(out)
    assert obj["msg"] == "hello"
    assert obj["level"] == "INFO"
    assert obj["logger"] == "test"


def test_json_formatter_with_event():
    fmt = JsonLineFormatter()
    record = logging.LogRecord(
        name="test", level=logging.INFO, pathname="x", lineno=1,
        msg="decision", args=(), exc_info=None,
    )
    record.event = {"state": "walking", "progress_m": 42.5}
    out = fmt.format(record)
    obj = json.loads(out)
    assert obj["event"]["state"] == "walking"
    assert obj["event"]["progress_m"] == 42.5


def test_replay_skips_malformed_lines(tmp_path: Path):
    p = tmp_path / "log.jsonl"
    p.write_text(
        '{"msg": "first"}\n'
        'not valid json\n'
        '\n'
        '{"msg": "second", "event": {"a": 1}}\n'
    )
    records = list(replay_jsonl(p))
    assert len(records) == 2
    assert records[0]["msg"] == "first"
    assert records[1]["event"]["a"] == 1


def test_log_event_helper(caplog):
    logger = logging.getLogger("test_helper")
    logger.setLevel(logging.DEBUG)
    with caplog.at_level(logging.INFO):
        log_event(logger, "info", "hello", state="walking", progress=42)
    assert len(caplog.records) == 1
    assert caplog.records[0].event == {"state": "walking", "progress": 42}
