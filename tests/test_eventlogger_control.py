from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
import importlib.util
import json
import threading
import urllib.error
import urllib.request


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / "raspberry_pi" / "eventlogger" / "eventlogger_control.py"


def load_module():
    spec = importlib.util.spec_from_file_location("eventlogger_control", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def make_request(url: str, method: str = "GET", payload: dict | None = None) -> dict | str:
    data = None
    headers = {}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    request = urllib.request.Request(url, method=method, data=data, headers=headers)
    with urllib.request.urlopen(request, timeout=5) as response:
        body = response.read().decode("utf-8")
        content_type = response.headers.get("Content-Type", "")
        if "application/json" in content_type:
            return json.loads(body)
        return body


def make_journal(root_dir: Path, name: str, rows: list[str]) -> None:
    journal_dir = root_dir / "journal"
    journal_dir.mkdir(parents=True, exist_ok=True)
    header = "utc_time\trealtime_ns\tmonotonic_ns\tinput\tedge\n"
    (journal_dir / name).write_text(header + "".join(rows))


def make_input_state(root_dir: Path) -> None:
    control_dir = root_dir / "control"
    control_dir.mkdir(parents=True, exist_ok=True)
    state = {
        "generated_utc": "2026-06-06T15:00:00.000000000Z",
        "inputs": [
            {
                "name": "DIN1",
                "gpio": 5,
                "enabled": True,
                "current_level": 0,
                "last_edge": "falling",
                "last_edge_utc": "2026-06-06T15:00:00.000000000Z",
                "rising_count": 2,
                "falling_count": 2,
            },
            {
                "name": "DIN2",
                "gpio": 6,
                "enabled": False,
                "current_level": 0,
                "last_edge": None,
                "last_edge_utc": None,
                "rising_count": 0,
                "falling_count": 0,
            },
        ],
    }
    (control_dir / "inputs.json").write_text(json.dumps(state))


def test_control_module_exists_and_normalizes_empty_basename():
    module = load_module()

    assert module.normalize_basename("") == "events"
    assert module.normalize_basename("   ") == "events"
    assert module.normalize_basename("mouse123") == "mouse123"
    assert module.normalize_basename("mouse-123_a") == "mouse-123_a"


def test_invalid_basenames_are_rejected():
    module = load_module()

    for invalid in (".hidden", "-bad", "_bad", "bad/name", "bad\\name", "bad:name", "bad*name", "bad?name"):
        try:
            module.validate_basename(invalid)
        except module.ControlError:
            continue
        raise AssertionError(f"expected invalid basename to fail: {invalid}")


def test_export_recording_slices_journal_and_writes_minimal_yaml(tmp_path: Path):
    module = load_module()
    root_dir = tmp_path / "eventlogger"
    make_journal(
        root_dir,
        "events_2026-06-06.tsv",
        [
            "2026-06-06T14:57:06.200002831Z\t1\t1\tDIN1\tfalling\n",
            "2026-06-06T14:57:07.000004223Z\t2\t2\tDIN1\trising\n",
            "2026-06-06T14:57:07.200001113Z\t3\t3\tDIN1\tfalling\n",
            "2026-06-06T14:57:08.000003466Z\t4\t4\tDIN2\trising\n",
        ],
    )

    recording = {
        "basename": "events",
        "start_utc": "2026-06-06T14:57:07.000000000Z",
        "stop_utc": "2026-06-06T14:57:08.000003466Z",
        "start_local": "2026-06-06T10:57:07",
        "timezone": "America/New_York",
        "exported_inputs": ["DIN1", "DIN2"],
        "interrupted": False,
    }

    result = module.export_recording(root_dir, recording, hostname="testpi")

    exported_tsv = Path(result["tsv_path"])
    exported_yaml = Path(result["yaml_path"])
    assert exported_tsv.exists()
    assert exported_yaml.exists()
    assert exported_tsv.name == "events_2026-06-06_105707.tsv"
    exported_lines = exported_tsv.read_text().splitlines()
    assert exported_lines[0] == "UTC_time\tinput\tedge"
    assert exported_lines[1:] == [
        "2026-06-06T14:57:07.000004223Z\tDIN1\trising",
        "2026-06-06T14:57:07.200001113Z\tDIN1\tfalling",
        "2026-06-06T14:57:08.000003466Z\tDIN2\trising",
    ]
    yaml_text = exported_yaml.read_text()
    assert "basename: events" in yaml_text
    assert "hostname: testpi" in yaml_text
    assert "interrupted: false" in yaml_text


def test_export_collision_only_shifts_filename(tmp_path: Path):
    module = load_module()
    root_dir = tmp_path / "eventlogger"
    recordings_dir = root_dir / "recordings"
    recordings_dir.mkdir(parents=True, exist_ok=True)
    (recordings_dir / "events_2026-06-06_105707.tsv").write_text("existing\n")
    (recordings_dir / "events_2026-06-06_105707.yaml").write_text("existing\n")
    make_journal(
        root_dir,
        "events_2026-06-06.tsv",
        ["2026-06-06T14:57:07.000004223Z\t2\t2\tDIN1\trising\n"],
    )

    recording = {
        "basename": "events",
        "start_utc": "2026-06-06T14:57:07.000000000Z",
        "stop_utc": "2026-06-06T14:57:07.000004223Z",
        "start_local": "2026-06-06T10:57:07",
        "timezone": "America/New_York",
        "exported_inputs": ["DIN1"],
        "interrupted": False,
    }

    result = module.export_recording(root_dir, recording, hostname="testpi")
    assert Path(result["tsv_path"]).name == "events_2026-06-06_105708.tsv"
    assert result["recording"]["start_local"] == "2026-06-06T10:57:07"


def test_export_tsv_drops_raw_nanosecond_columns(tmp_path: Path):
    module = load_module()
    root_dir = tmp_path / "eventlogger"
    make_journal(
        root_dir,
        "events_2026-06-06.tsv",
        [
            "2026-06-06T14:57:07.000004223Z\t123456789\t987654321\tDIN1\trising\n",
        ],
    )

    recording = {
        "basename": "events",
        "start_utc": "2026-06-06T14:57:07.000000000Z",
        "stop_utc": "2026-06-06T14:57:07.000004223Z",
        "start_local": "2026-06-06T10:57:07",
        "timezone": "America/New_York",
        "exported_inputs": ["DIN1"],
        "interrupted": False,
    }

    result = module.export_recording(root_dir, recording, hostname="testpi")
    exported_text = Path(result["tsv_path"]).read_text()

    assert "realtime_ns" not in exported_text
    assert "monotonic_ns" not in exported_text
    assert "123456789" not in exported_text
    assert "987654321" not in exported_text


def test_recover_interrupted_recording_creates_interrupted_export(tmp_path: Path):
    module = load_module()
    root_dir = tmp_path / "eventlogger"
    make_journal(
        root_dir,
        "events_2026-06-06.tsv",
        ["2026-06-06T14:57:07.000004223Z\t2\t2\tDIN1\trising\n"],
    )
    control_dir = root_dir / "control"
    control_dir.mkdir(parents=True, exist_ok=True)
    active_state = {
        "basename": "events",
        "start_utc": "2026-06-06T14:57:07.000000000Z",
        "start_local": "2026-06-06T10:57:07",
        "timezone": "America/New_York",
    }
    active_path = control_dir / "active_recording.json"
    active_path.write_text(json.dumps(active_state))

    recovery_time = datetime(2026, 6, 6, 15, 0, 0, tzinfo=timezone.utc)
    result = module.recover_interrupted_recording(root_dir, recovery_time, hostname="testpi")

    assert result is not None
    assert result["recording"]["interrupted"] is True
    assert not active_path.exists()


def test_status_payload_reads_input_state_and_active_recording(tmp_path: Path):
    module = load_module()
    root_dir = tmp_path / "eventlogger"
    make_input_state(root_dir)
    control_dir = root_dir / "control"
    active = {
        "basename": "events",
        "export_stem": "events_2026-06-06_105707",
        "input_rising_baselines": {"DIN1": 1, "DIN2": 0},
        "start_utc": "2026-06-06T14:57:07.000000000Z",
        "start_local": "2026-06-06T10:57:07",
        "timezone": "America/New_York",
    }
    (control_dir / "active_recording.json").write_text(json.dumps(active))

    app = module.EventLoggerControlApp(root_dir=root_dir, hostname="testpi")
    status = app.build_status()

    assert status["active_recording"]["active"] is True
    assert status["active_recording"]["basename"] == "events"
    assert status["active_recording"]["filename"] == "events_2026-06-06_105707.tsv"
    assert status["active_recording"]["input_rising_baselines"]["DIN1"] == 1
    assert status["inputs"][0]["rising_count"] == 2
    assert status["inputs"][0]["current_level"] == 0


def test_http_server_serves_page_and_recording_workflow(tmp_path: Path):
    module = load_module()
    root_dir = tmp_path / "eventlogger"
    make_journal(
        root_dir,
        "events_2026-06-06.tsv",
        [
            "2026-06-06T14:57:07.000004223Z\t2\t2\tDIN1\trising\n",
            "2026-06-06T14:57:07.200001113Z\t3\t3\tDIN1\tfalling\n",
        ],
    )
    make_input_state(root_dir)

    clock_times = iter(
        [
            datetime(2026, 6, 6, 14, 57, 7, tzinfo=timezone.utc),
            datetime(2026, 6, 6, 14, 57, 8, tzinfo=timezone.utc),
        ]
    )

    app = module.EventLoggerControlApp(
        root_dir=root_dir,
        hostname="testpi",
        now_utc=lambda: next(clock_times),
        local_timezone_name="America/New_York",
        local_timezone=timezone.utc,
    )
    httpd = module.make_http_server("127.0.0.1", 0, app)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{httpd.server_address[1]}"

    try:
        page = make_request(base_url + "/")
        assert "Event Logger Test UI" in page
        assert "250 ms" in page

        status = make_request(base_url + "/v1/status")
        assert status["active_recording"]["active"] is False

        started = make_request(base_url + "/v1/recordings/start", method="POST", payload={"basename": "   "})
        assert started["recording"]["basename"] == "events"

        stopped = make_request(base_url + "/v1/recordings/stop", method="POST")
        assert stopped["recording"]["basename"] == "events"
        assert Path(stopped["tsv_path"]).exists()
    finally:
        httpd.shutdown()
        thread.join(timeout=5)


def test_start_rejects_when_recording_already_active(tmp_path: Path):
    module = load_module()
    root_dir = tmp_path / "eventlogger"
    make_input_state(root_dir)
    app = module.EventLoggerControlApp(
        root_dir=root_dir,
        hostname="testpi",
        now_utc=lambda: datetime(2026, 6, 6, 14, 57, 7, tzinfo=timezone.utc),
        local_timezone_name="America/New_York",
        local_timezone=timezone.utc,
    )
    app.start_recording("events")

    try:
        app.start_recording("second")
    except module.ControlError as exc:
        assert "already active" in str(exc)
    else:
        raise AssertionError("expected ControlError")


def test_start_recording_reserves_export_filename_immediately(tmp_path: Path):
    module = load_module()
    root_dir = tmp_path / "eventlogger"
    make_input_state(root_dir)
    recordings_dir = root_dir / "recordings"
    recordings_dir.mkdir(parents=True, exist_ok=True)
    (recordings_dir / "events_2026-06-06_145707.tsv").write_text("existing\n")
    (recordings_dir / "events_2026-06-06_145707.yaml").write_text("existing\n")

    app = module.EventLoggerControlApp(
        root_dir=root_dir,
        hostname="testpi",
        now_utc=lambda: datetime(2026, 6, 6, 14, 57, 7, tzinfo=timezone.utc),
        local_timezone_name="America/New_York",
        local_timezone=timezone.utc,
    )

    recording = app.start_recording("events")
    status = app.build_status()

    assert recording["export_stem"] == "events_2026-06-06_145708"
    assert recording["input_rising_baselines"]["DIN1"] == 2
    assert status["active_recording"]["filename"] == "events_2026-06-06_145708.tsv"


def test_file_contracts_for_control_service_and_ui():
    service = (REPO_ROOT / "raspberry_pi" / "eventlogger" / "eventlogger-control.service").read_text()
    install_script = (REPO_ROOT / "raspberry_pi" / "eventlogger" / "install_eventlogger.sh").read_text()
    ui = (REPO_ROOT / "raspberry_pi" / "eventlogger" / "web" / "index.html").read_text()

    assert "ExecStart=/usr/bin/python3 /usr/local/lib/neurokairos-eventlogger/eventlogger_control.py" in service
    assert "neurokairos-eventlogger-control.service" in install_script
    assert "avahi-publish" in install_script
    assert "setInterval(() => {" in ui
    assert "Digital inputs:" in ui
    assert "Pulses recorded" in ui
    assert "Filename:" in ui
    assert "record-light" in ui
    assert "control-button record-button" in ui
    assert "control-button stop-button" in ui
