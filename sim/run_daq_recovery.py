"""Qualify real DAQ CAN loss/rejoin without changing normal linked gates."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

DURATION_MS = 40_000
SAMPLES = 20
COMMAND_SAMPLES = (1, 2, *range(6, SAMPLES))
CONTROL_ACK_MS = 2_500
MAX_DATA_GAP_MS = 500


def configure_recovery(topology: dict, daq: dict) -> None:
    """Scope only the injected DAQ outage and two recovery observations."""
    if (topology.get("virtual_time_ms") != DURATION_MS
            or topology.get("sample_count") != SAMPLES
            or topology.get("reboots") or topology.get("can_ack_events")
            or topology.get("can_link_events")):
        raise ValueError("DAQ recovery requires its isolated 40-second, 20-sample scene")
    topology["can_link_events"] = [
        dict(node="daq", peripheral="fdcan1", link="fill_can", after_sample=3, connected=False),
        dict(node="daq", peripheral="fdcan1", link="fill_can", after_sample=5, connected=True),
    ]
    # The Bay topology overrides the standalone node run duration. Keep the
    # finite offer budget tied to this actual 40-second scene.
    daq["execution"]["virtual_time_ms"] = DURATION_MS
    probes = {probe["name"]: probe for probe in daq["execution"]["memory_probes"]}
    scoped = ("analog_report_failures", "network_publication_failures", "kg50_network_failures",
              "loadcell_network_failures", "publication_io_refusals", "can_tx_expired")
    for name in scoped:
        if probes[name].get("maximum") != 0:
            raise ValueError(f"Normal DAQ {name} must have its zero bound")
        probes[name].pop("maximum")
        # Off after sample3/on after5 affects samples3,4. Samples5,6 include
        # reattachment/draining. No later interval may add failures or expire.
        for sample in (0, 1, 2, *range(7, SAMPLES)):
            check = dict(name=f"DAQ {name} healthy interval {sample}", node="daq", probe=name)
            if sample == 0:
                check.update(sample=0, maximum=0)
            else:
                check.update(from_sample=sample - 1, to_sample=sample, maximum_gain=0)
            topology["assertions"].append(check)
    for name in ("publication_other_refusals", "sdk_snapshot_failures", "sdk_admission_drops"):
        if probes[name].get("maximum") != 0:
            raise ValueError(f"DAQ {name} must remain zero even during the outage")
    topology["assertions"] += [
        dict(name="DAQ publisher experienced the actual outage", node="daq",
             probe="publication_io_refusals", minimum=1),
        dict(name="DAQ CAN controller could not transmit during loss", node="daq",
             probe="can_tx_rejected", minimum_gain=1, from_sample=2, to_sample=4),
    ]
    for name in ("daq_samples", "sd_raw_records_written", "sd_csv_rows_written",
                 "fdcan_tx_ok", "loadcell_network_rows", "kg50_network_rows"):
        for sample in range(7, SAMPLES):
            topology["assertions"].append(dict(
                name=f"DAQ {name} recovery progress {sample}", node="daq", probe=name,
                minimum_gain=1, from_sample=sample - 1, to_sample=sample))
    host = next(host for host in topology["host_nodes"] if host["name"] == "groundstation")
    host["env"].update(
        GS_SIM_VALIDATE_SOAK_COMMANDS="1",
        GS_SIM_SOAK_COMMAND_SAMPLES=",".join(map(str, COMMAND_SAMPLES)),
        GS_SIM_SOAK_COMMAND_ACK_MAX_LATENCY_MS=str(CONTROL_ACK_MS),
        GS_SIM_VALIDATE_FILL_DELIVERY="1",
    )
    for board in ("VB", "AB"):
        topology["host_log_assertions"].append(dict(
            name=f"Every {board} recovery control round returns its ACK", node="groundstation",
            contains=f"full-bay soak valve command acknowledged: board={board},",
            minimum_occurrences=len(COMMAND_SAMPLES)))


def verify_host_recovery(log: str, loadcell_hz: int) -> dict:
    """Verify accepted GS data and exact control rounds, not producer counters."""
    acknowledgements = {}
    for board in ("VB", "AB"):
        observed = {}
        pattern = (r"full-bay soak valve command acknowledged: board=" + board
                   + r",.*?sample (\d+), latency (\d+) ms")
        for line in log.splitlines():
            match = re.search(pattern, line)
            if match:
                sample, latency = map(int, match.groups())
                if sample in observed or latency > CONTROL_ACK_MS:
                    raise ValueError(f"Duplicate or late {board} control ACK at sample {sample}")
                observed[sample] = latency
        if set(observed) != set(COMMAND_SAMPLES):
            raise ValueError(f"Missing actual {board} recovery command acknowledgements")
        acknowledgements[board] = observed
    windows = []
    for line in log.splitlines():
        match = re.search(r"sustained fill delivery (?:passed|failed) (\{.*\})", line)
        if not match:
            continue
        window = json.loads(match.group(1))
        if window["start_ms"] < 20_000:
            continue
        elapsed = window["elapsed_ms"]
        if elapsed < 10_000:
            raise ValueError("Recovery data observation window was incomplete")
        # This existing observer exports unique SDK-accepted packet counts.
        # Its old 250Hz PASS flag is not a rate oracle for a configured 40Hz
        # publisher. Require >=95% of the actual configured observer cadence.
        for channel in (0, 1):
            if (window["received"][channel] * 1_000_000 < loadcell_hz * 950 * elapsed
                    or window["max_gap_ms"][channel] > MAX_DATA_GAP_MS):
                raise ValueError("Accepted DAQ data did not recover its rate and freshness")
        windows.append(window)
    if not windows:
        raise ValueError("No actual GS accepted-data window after CAN recovery")
    return dict(acknowledgements=acknowledgements, windows=windows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--build-subdir", default="Release_Script")
    args = parser.parse_args()
    root = args.repo_root.resolve()
    sys.path.insert(0, str(root))
    from sim import run_full
    if json.loads((root / "sim/board.json").read_text())["name"] != "DAQ-Board":
        raise ValueError("DAQ recovery must run from the DAQ repository")
    observer_hz = int(run_full.daq_loadcell_observer_config()["environment"]["GS_SIM_DAQ_LOADCELL_HZ"])
    suite = os.environ.get("SEDS_FIRMWARE_SIM_SUITE_ROOT", str(root / "build/sim-full-bay"))
    os.environ.update(SEDS_FIRMWARE_SIM_SUITE_ROOT=suite, SEDS_FIRMWARE_SIM_SKIP_BUILD="1",
                      SEDS_FIRMWARE_SIM_NETWORK_TIME_MS=str(DURATION_MS),
                      SEDS_FIRMWARE_SIM_NETWORK_SAMPLES=str(SAMPLES))
    (root / "build").mkdir(exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix="daq-can-recovery-", dir=root / "build"))
    original_run = run_full.run_live

    def run_recovery(command, label):
        if "bay" not in command:
            return original_run(command, label)
        mount = next(command[i + 1] for i, item in enumerate(command[:-1])
                     if item == "-v" and command[i + 1].endswith(":/simulation:ro"))
        fixture = Path(mount.removesuffix(":/simulation:ro"))
        topology = json.loads((fixture / "topology.json").read_text())
        daq = json.loads((fixture / "daq.json").read_text())
        configure_recovery(topology, daq)
        # Apply the actual build's aggregate submission bound to this fault
        # layout only, retaining every per-interval healthy zero assertion.
        run_full.scope_daq_disconnected_publication_probes(daq, root, args.build_subdir)
        (fixture / "topology.json").write_text(json.dumps(topology, indent=2))
        (fixture / "daq.json").write_text(json.dumps(daq, indent=2))
        for path in fixture.iterdir():
            if path.is_file():
                shutil.copy2(path, output / path.name)
        diagnostics = output / "diagnostics"
        diagnostics.mkdir()
        diagnostics.chmod(0o777)
        command = command[:]
        command[2:2] = ["--group-add", str(os.getgid()), "-e",
                        "FIRMWARE_SIM_DIAGNOSTICS_DIR=/diagnostics",
                        "-v", str(diagnostics) + ":/diagnostics"]
        print(f"[SIM] DAQ CAN recovery evidence: {output}", flush=True)
        with (output / "stdout.log").open("w") as stdout, (output / "stderr.log").open("w") as stderr:
            process = subprocess.Popen(command, stdout=stdout, stderr=stderr)
            try:
                process.wait(timeout=1800)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                raise
        if process.returncode:
            raise subprocess.CalledProcessError(process.returncode, command)
        host_logs = list(diagnostics.rglob("groundstation-host.log"))
        if len(host_logs) != 1:
            raise ValueError("Missing unique actual GroundStation recovery log")
        proof = verify_host_recovery(host_logs[0].read_text(), observer_hz)
        (output / "recovery-proof.json").write_text(json.dumps(proof, indent=2))
        print("[SIM] DAQ CAN recovery preserved fresh data, storage and control ACK latency", flush=True)

    class UI:
        def say(self, kind, message):
            print(f"[{kind}] {message}", flush=True)

    run_full.run_live = run_recovery
    run_full.run_network_simulation(UI(), root, "stm32u5", args.build_subdir)


if __name__ == "__main__":
    main()
