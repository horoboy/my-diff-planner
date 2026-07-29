#!/usr/bin/env python3

import argparse
import copy
import csv
import datetime
import json
import math
import os
from pathlib import Path
import re
import socket
import subprocess
import sys

import yaml


TARGET_STAGES = ("early", "middle", "late")


def port_available(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        try:
            probe.bind(("127.0.0.1", port))
        except OSError:
            return False
    return True


def next_available_port(start, reserved):
    port = start
    while port <= 65535:
        if port not in reserved and port_available(port):
            return port
        port += 1
    raise RuntimeError("no available TCP port at or above %d" % start)


def parse_csv_values(raw, cast, name):
    values = []
    for item in raw.split(","):
        item = item.strip()
        if not item:
            continue
        try:
            values.append(cast(item))
        except ValueError as error:
            raise argparse.ArgumentTypeError(
                "invalid %s value %r" % (name, item)
            ) from error
    if not values:
        raise argparse.ArgumentTypeError("%s must not be empty" % name)
    return values


def load_yaml(path):
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError("base configuration must contain a YAML mapping")
    return data


def target_for_stage(config, stage, edge_margin):
    center = config["roi_center"]
    radius = float(config["roi_radius"])
    target = config.get("target_position", [center[0], center[1], 0.0])
    z = float(target[2]) if len(target) >= 3 else 0.0
    offset = max(0.0, radius - edge_margin)
    positions = {
        "early": [float(center[0]), float(center[1]) - offset, z],
        "middle": [float(center[0]), float(center[1]), z],
        "late": [float(center[0]), float(center[1]) + offset, z],
    }
    return positions[stage]


def finite_number(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def extract_log_metrics(path):
    metrics = {
        "search_goal_skips": 0,
        "max_consecutive_search_goal_skips": 0,
        "planner_failure_events": 0,
        "planner_goals_issued": 0,
        "route_repaired_segments": 0,
        "route_detour_goals": 0,
        "transit_recoveries": 0,
        "motion_recoveries": 0,
    }
    if not path.is_file():
        return metrics

    text = path.read_text(encoding="utf-8", errors="replace")
    consecutive = [
        int(value)
        for value in re.findall(
            r"Skipping search goal .*? \((\d+) consecutive\)", text
        )
    ]
    metrics["search_goal_skips"] = len(consecutive)
    metrics["max_consecutive_search_goal_skips"] = max(
        consecutive, default=0
    )
    metrics["planner_failure_events"] = len(
        re.findall(r"^Success=no$", text, flags=re.MULTILINE)
    )
    metrics["planner_goals_issued"] = len(
        re.findall(
            r"\[competition\] Goal \d+ \(planner_goal_\d+_of_\d+\)",
            text,
        )
    )
    route_repair = re.search(
        r"Search route repaired (\d+) unsafe segments with "
        r"(\d+) detour goals",
        text,
    )
    if route_repair:
        metrics["route_repaired_segments"] = int(route_repair.group(1))
        metrics["route_detour_goals"] = int(route_repair.group(2))
    metrics["transit_recoveries"] = len(
        re.findall(r"Transit recovery \d+/\d+ goal republished", text)
    )
    metrics["motion_recoveries"] = len(
        re.findall(
            r"Motion recovery in \S+ \d+/\d+ goal republished", text
        )
    )
    return metrics


def evaluate_result(result, return_code, criteria, metrics):
    failures = []
    if result is None:
        failures.append("result_missing")
        if return_code:
            failures.append("runner_exit_%d" % return_code)
        return failures

    if not result.get("success", False):
        failures.append("mission_failed:%s" % result.get("reason", "unknown"))
    if result.get("final_state") != "COMPLETE":
        failures.append("final_state:%s" % result.get("final_state", "missing"))

    duration = result.get("duration")
    if not finite_number(duration):
        failures.append("duration_invalid")
    elif float(duration) > criteria["max_duration"]:
        failures.append("duration_exceeded")

    collisions = result.get("collision_count")
    if not isinstance(collisions, int):
        failures.append("collision_count_invalid")
    elif collisions > criteria["max_collisions"]:
        failures.append("collision_limit_exceeded")

    altitude_violations = result.get("altitude_violation_count")
    if not isinstance(altitude_violations, int):
        failures.append("altitude_violation_count_invalid")
    elif altitude_violations > criteria["max_altitude_violations"]:
        failures.append("altitude_violation_limit_exceeded")

    planner_recoveries = result.get("planner_recovery_count")
    if not isinstance(planner_recoveries, int):
        failures.append("planner_recovery_count_invalid")
    elif planner_recoveries > criteria["max_planner_recoveries"]:
        failures.append("planner_recovery_limit_exceeded")

    clearance = result.get("min_obstacle_clearance")
    if not finite_number(clearance):
        failures.append("clearance_invalid")
    elif float(clearance) < criteria["min_clearance"]:
        failures.append("clearance_below_limit")

    drop_error = result.get("drop_error")
    if not finite_number(drop_error):
        failures.append("drop_error_invalid")
    elif float(drop_error) > criteria["max_drop_error"]:
        failures.append("drop_error_exceeded")

    if (
        metrics["max_consecutive_search_goal_skips"]
        > criteria["max_consecutive_search_goal_skips"]
    ):
        failures.append("consecutive_search_goal_skips_exceeded")

    if return_code and not failures:
        failures.append("runner_exit_%d" % return_code)
    return failures


def write_json(path, data):
    with path.open("w", encoding="utf-8") as stream:
        json.dump(data, stream, indent=2, sort_keys=True)
        stream.write("\n")


def write_csv(path, cases):
    fields = (
        "name",
        "seed",
        "tree_count",
        "target_stage",
        "target_x",
        "target_y",
        "target_z",
        "ros_port",
        "passed",
        "failures",
        "runner_return_code",
        "mission_success",
        "reason",
        "final_state",
        "duration",
        "goal_sequence",
        "collision_count",
        "min_altitude",
        "max_altitude",
        "altitude_violation_count",
        "planner_recovery_count",
        "min_obstacle_clearance",
        "drop_error",
        "search_goal_skips",
        "max_consecutive_search_goal_skips",
        "planner_failure_events",
        "planner_goals_issued",
        "route_repaired_segments",
        "route_detour_goals",
        "transit_recoveries",
        "motion_recoveries",
        "min_clearance_x",
        "min_clearance_y",
        "min_clearance_z",
        "min_clearance_tree_id",
        "min_clearance_state",
        "config_file",
        "result_file",
        "log_file",
        "runner_log",
    )
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for case in cases:
            result = case.get("result") or {}
            target = case["target_position"]
            clearance_position = result.get("min_clearance_position") or [
                None,
                None,
                None,
            ]
            writer.writerow(
                {
                    "name": case["name"],
                    "seed": case["seed"],
                    "tree_count": case["tree_count"],
                    "target_stage": case["target_stage"],
                    "target_x": target[0],
                    "target_y": target[1],
                    "target_z": target[2],
                    "ros_port": case["ros_port"],
                    "passed": case.get("passed"),
                    "failures": ";".join(case.get("failures", [])),
                    "runner_return_code": case.get("runner_return_code"),
                    "mission_success": result.get("success"),
                    "reason": result.get("reason"),
                    "final_state": result.get("final_state"),
                    "duration": result.get("duration"),
                    "goal_sequence": result.get("goal_sequence"),
                    "collision_count": result.get("collision_count"),
                    "min_altitude": result.get("min_altitude"),
                    "max_altitude": result.get("max_altitude"),
                    "altitude_violation_count": result.get(
                        "altitude_violation_count"
                    ),
                    "planner_recovery_count": result.get(
                        "planner_recovery_count"
                    ),
                    "min_obstacle_clearance": result.get(
                        "min_obstacle_clearance"
                    ),
                    "drop_error": result.get("drop_error"),
                    "search_goal_skips": case["metrics"][
                        "search_goal_skips"
                    ],
                    "max_consecutive_search_goal_skips": case["metrics"][
                        "max_consecutive_search_goal_skips"
                    ],
                    "planner_failure_events": case["metrics"][
                        "planner_failure_events"
                    ],
                    "planner_goals_issued": case["metrics"][
                        "planner_goals_issued"
                    ],
                    "route_repaired_segments": case["metrics"][
                        "route_repaired_segments"
                    ],
                    "route_detour_goals": case["metrics"][
                        "route_detour_goals"
                    ],
                    "transit_recoveries": case["metrics"][
                        "transit_recoveries"
                    ],
                    "motion_recoveries": case["metrics"][
                        "motion_recoveries"
                    ],
                    "min_clearance_x": clearance_position[0],
                    "min_clearance_y": clearance_position[1],
                    "min_clearance_z": clearance_position[2],
                    "min_clearance_tree_id": result.get(
                        "min_clearance_tree_id"
                    ),
                    "min_clearance_state": result.get(
                        "min_clearance_state"
                    ),
                    "config_file": case["config_file"],
                    "result_file": case["result_file"],
                    "log_file": case["log_file"],
                    "runner_log": case["runner_log"],
                }
            )


def print_case_result(index, total, case):
    result = case.get("result") or {}
    status = "PASS" if case.get("passed") else "FAIL"
    print(
        "[%d/%d] %s %s duration=%s clearance=%s altitude=[%s,%s] drop=%s"
        % (
            index,
            total,
            status,
            case["name"],
            result.get("duration", "-"),
            result.get("min_obstacle_clearance", "-"),
            result.get("min_altitude", "-"),
            result.get("max_altitude", "-"),
            result.get("drop_error", "-"),
        ),
        flush=True,
    )
    if case.get("failures"):
        print("        %s" % ", ".join(case["failures"]), flush=True)


def build_parser(package_dir):
    timestamp = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
    parser = argparse.ArgumentParser(
        description=(
            "Generate and run a deterministic single-UAV competition "
            "simulation matrix."
        )
    )
    parser.add_argument(
        "--base-config",
        type=Path,
        default=package_dir / "config" / "single_uav.yaml",
        help="base YAML configuration",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("/tmp/competition_batch_%s" % timestamp),
        help="new directory for generated configs, logs, and reports",
    )
    parser.add_argument(
        "--seeds",
        default="10,14,20,30,40",
        help="comma-separated forest seeds",
    )
    parser.add_argument(
        "--tree-counts",
        default="",
        help="comma-separated tree counts; default uses the base config",
    )
    parser.add_argument(
        "--target-stages",
        default="early,middle,late",
        help="comma-separated stages: early,middle,late",
    )
    parser.add_argument(
        "--target-edge-margin",
        type=float,
        default=0.2,
        help="distance from early/late targets to the ROI boundary",
    )
    parser.add_argument("--ros-port-base", type=int, default=11400)
    parser.add_argument("--mission-timeout", type=int, default=230)
    parser.add_argument("--search-backend", default="coverage")
    parser.add_argument("--obstacles-inflation", type=float, default=0.40)
    parser.add_argument("--obstacle-clearance", type=float, default=0.20)
    parser.add_argument(
        "--obstacle-clearance-soft", type=float, default=0.50
    )
    parser.add_argument(
        "--max-duration",
        type=float,
        default=180.0,
        help=(
            "maximum complete mission duration in seconds, including "
            "takeoff, transit, search, drop, return, and landing"
        ),
    )
    parser.add_argument("--min-clearance", type=float, default=0.20)
    parser.add_argument("--max-drop-error", type=float, default=0.45)
    parser.add_argument("--max-collisions", type=int, default=0)
    parser.add_argument("--max-altitude-violations", type=int, default=0)
    parser.add_argument("--max-planner-recoveries", type=int, default=3)
    parser.add_argument(
        "--max-consecutive-search-goal-skips",
        type=int,
        default=2,
    )
    parser.add_argument(
        "--stop-on-failure",
        action="store_true",
        help="stop after the first failed case",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="only generate configurations and manifest",
    )
    return parser


def main():
    script_path = Path(__file__).resolve()
    package_dir = script_path.parent.parent
    single_runner = script_path.parent / "run_single_uav_validation.sh"
    args = build_parser(package_dir).parse_args()

    base_config = args.base_config.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    if not base_config.is_file():
        raise SystemExit("base config not found: %s" % base_config)
    if not single_runner.is_file():
        raise SystemExit("single-run validator not found: %s" % single_runner)
    if output_dir.exists() and any(output_dir.iterdir()):
        raise SystemExit("output directory is not empty: %s" % output_dir)
    if args.ros_port_base <= 1024:
        raise SystemExit("--ros-port-base must be greater than 1024")
    if args.mission_timeout <= 0:
        raise SystemExit("--mission-timeout must be positive")
    if args.target_edge_margin < 0.0:
        raise SystemExit("--target-edge-margin must not be negative")
    if args.search_backend not in ("coverage", "fuel"):
        raise SystemExit("--search-backend must be coverage or fuel")
    if args.obstacles_inflation < 0.0:
        raise SystemExit("--obstacles-inflation must not be negative")
    if args.obstacle_clearance < 0.0:
        raise SystemExit("--obstacle-clearance must not be negative")
    if args.obstacle_clearance_soft < 0.0:
        raise SystemExit("--obstacle-clearance-soft must not be negative")
    if args.obstacle_clearance_soft < args.obstacle_clearance:
        raise SystemExit(
            "--obstacle-clearance-soft must be at least "
            "--obstacle-clearance"
        )
    if args.max_duration <= 0.0:
        raise SystemExit("--max-duration must be positive")
    if args.min_clearance < 0.0:
        raise SystemExit("--min-clearance must not be negative")
    if args.max_drop_error < 0.0:
        raise SystemExit("--max-drop-error must not be negative")
    if args.max_collisions < 0:
        raise SystemExit("--max-collisions must not be negative")
    if args.max_altitude_violations < 0:
        raise SystemExit("--max-altitude-violations must not be negative")
    if args.max_planner_recoveries < 0:
        raise SystemExit("--max-planner-recoveries must not be negative")
    if args.max_consecutive_search_goal_skips < 0:
        raise SystemExit(
            "--max-consecutive-search-goal-skips must not be negative"
        )

    seeds = parse_csv_values(args.seeds, int, "seed")
    stages = parse_csv_values(args.target_stages, str, "target stage")
    invalid_stages = [stage for stage in stages if stage not in TARGET_STAGES]
    if invalid_stages:
        raise SystemExit(
            "invalid target stages: %s" % ", ".join(invalid_stages)
        )

    base = load_yaml(base_config)
    base_tree_count = int(base["map"]["tree_count"])
    tree_counts = (
        parse_csv_values(args.tree_counts, int, "tree count")
        if args.tree_counts.strip()
        else [base_tree_count]
    )
    if any(tree_count <= 0 for tree_count in tree_counts):
        raise SystemExit("tree counts must be positive")
    for name, values in (
        ("seeds", seeds),
        ("tree counts", tree_counts),
        ("target stages", stages),
    ):
        if len(values) != len(set(values)):
            raise SystemExit("%s must not contain duplicates" % name)

    total = len(seeds) * len(tree_counts) * len(stages)
    if args.ros_port_base > 65535:
        raise SystemExit("--ros-port-base must not exceed 65535")

    criteria = {
        "max_duration": args.max_duration,
        "min_clearance": args.min_clearance,
        "max_drop_error": args.max_drop_error,
        "max_collisions": args.max_collisions,
        "max_altitude_violations": args.max_altitude_violations,
        "max_planner_recoveries": args.max_planner_recoveries,
        "max_consecutive_search_goal_skips": (
            args.max_consecutive_search_goal_skips
        ),
    }
    for directory in (
        output_dir,
        output_dir / "configs",
        output_dir / "logs",
        output_dir / "results",
        output_dir / "ros_home",
        output_dir / "ros_logs",
    ):
        directory.mkdir(parents=True, exist_ok=True)

    cases = []
    case_index = 0
    reserved_ports = set()
    port_cursor = args.ros_port_base
    for tree_count in tree_counts:
        for seed in seeds:
            for stage in stages:
                case_index += 1
                name = "seed%03d_trees%03d_target_%s" % (
                    seed,
                    tree_count,
                    stage,
                )
                config = copy.deepcopy(base)
                config["map"]["seed"] = seed
                config["map"]["tree_count"] = tree_count
                target_position = target_for_stage(
                    config, stage, args.target_edge_margin
                )
                config["target_position"] = target_position

                config_file = output_dir / "configs" / ("%s.yaml" % name)
                result_file = output_dir / "results" / ("%s.json" % name)
                log_file = output_dir / "logs" / ("%s.ros.log" % name)
                runner_log = output_dir / "logs" / ("%s.runner.log" % name)
                with config_file.open("w", encoding="utf-8") as stream:
                    yaml.safe_dump(config, stream, sort_keys=False)

                try:
                    ros_port = next_available_port(
                        port_cursor, reserved_ports
                    )
                except RuntimeError as error:
                    raise SystemExit(str(error)) from error
                if ros_port != port_cursor:
                    print(
                        "Skipping occupied TCP ports %d-%d"
                        % (port_cursor, ros_port - 1),
                        flush=True,
                    )
                reserved_ports.add(ros_port)
                port_cursor = ros_port + 1
                cases.append(
                    {
                        "name": name,
                        "seed": seed,
                        "tree_count": tree_count,
                        "target_stage": stage,
                        "target_position": target_position,
                        "ros_port": ros_port,
                        "config_file": str(config_file),
                        "result_file": str(result_file),
                        "log_file": str(log_file),
                        "runner_log": str(runner_log),
                    }
                )

    manifest = {
        "base_config": str(base_config),
        "criteria": criteria,
        "dry_run": args.dry_run,
        "mission_timeout": args.mission_timeout,
        "obstacles_inflation": args.obstacles_inflation,
        "obstacle_clearance": args.obstacle_clearance,
        "obstacle_clearance_soft": args.obstacle_clearance_soft,
        "output_dir": str(output_dir),
        "search_backend": args.search_backend,
        "total_cases": len(cases),
        "cases": cases,
    }
    write_json(output_dir / "manifest.json", manifest)

    print("Competition batch")
    print("  output: %s" % output_dir)
    print("  cases:  %d" % len(cases))
    print("  mode:   %s" % ("dry-run" if args.dry_run else "sequential"))
    if args.dry_run:
        for index, case in enumerate(cases, 1):
            print(
                "[%d/%d] PLAN %s port=%d target=%s"
                % (
                    index,
                    len(cases),
                    case["name"],
                    case["ros_port"],
                    case["target_position"],
                )
            )
        return 0

    completed_cases = []
    runtime_port_cursor = max(reserved_ports, default=args.ros_port_base) + 1
    for index, case in enumerate(cases, 1):
        if not port_available(case["ros_port"]):
            previous_port = case["ros_port"]
            try:
                case["ros_port"] = next_available_port(
                    runtime_port_cursor, reserved_ports
                )
            except RuntimeError as error:
                raise SystemExit(str(error)) from error
            reserved_ports.add(case["ros_port"])
            runtime_port_cursor = case["ros_port"] + 1
            case["ros_port_reassigned_from"] = previous_port
            write_json(output_dir / "manifest.json", manifest)
            print(
                "[%d/%d] port %d became occupied; reassigned to %d"
                % (
                    index,
                    len(cases),
                    previous_port,
                    case["ros_port"],
                ),
                flush=True,
            )
        print(
            "[%d/%d] RUN  %s port=%d"
            % (index, len(cases), case["name"], case["ros_port"]),
            flush=True,
        )
        environment = os.environ.copy()
        environment.update(
            {
                "CONFIG_FILE": case["config_file"],
                "RESULT_FILE": case["result_file"],
                "LOG_FILE": case["log_file"],
                "ROS_PORT": str(case["ros_port"]),
                "MISSION_TIMEOUT": str(args.mission_timeout),
                "SEARCH_BACKEND": args.search_backend,
                "OBSTACLES_INFLATION": str(args.obstacles_inflation),
                "OBSTACLE_CLEARANCE": str(args.obstacle_clearance),
                "OBSTACLE_CLEARANCE_SOFT": str(
                    args.obstacle_clearance_soft
                ),
                "ROS_HOME": str(
                    output_dir / "ros_home" / case["name"]
                ),
                "ROS_LOG_DIR": str(
                    output_dir / "ros_logs" / case["name"]
                ),
            }
        )
        with Path(case["runner_log"]).open("w", encoding="utf-8") as stream:
            process = subprocess.run(
                [str(single_runner)],
                env=environment,
                stdout=stream,
                stderr=subprocess.STDOUT,
                check=False,
            )

        result_path = Path(case["result_file"])
        result = None
        if result_path.is_file():
            try:
                with result_path.open("r", encoding="utf-8") as stream:
                    result = json.load(stream)
            except (json.JSONDecodeError, OSError) as error:
                case["result_read_error"] = str(error)

        case["runner_return_code"] = process.returncode
        case["result"] = result
        case["metrics"] = extract_log_metrics(Path(case["log_file"]))
        case["failures"] = evaluate_result(
            result, process.returncode, criteria, case["metrics"]
        )
        case["passed"] = not case["failures"]
        completed_cases.append(case)
        print_case_result(index, len(cases), case)
        if args.stop_on_failure and not case["passed"]:
            break

    passed = sum(1 for case in completed_cases if case["passed"])
    summary = {
        "base_config": str(base_config),
        "criteria": criteria,
        "finished_at": datetime.datetime.now().isoformat(),
        "obstacles_inflation": args.obstacles_inflation,
        "obstacle_clearance": args.obstacle_clearance,
        "obstacle_clearance_soft": args.obstacle_clearance_soft,
        "output_dir": str(output_dir),
        "planned_cases": len(cases),
        "completed_cases": len(completed_cases),
        "passed_cases": passed,
        "failed_cases": len(completed_cases) - passed,
        "all_passed": passed == len(cases),
        "cases": completed_cases,
    }
    write_json(output_dir / "summary.json", summary)
    write_csv(output_dir / "summary.csv", completed_cases)

    print("Batch result")
    print("  passed: %d" % passed)
    print("  failed: %d" % (len(completed_cases) - passed))
    print("  JSON:   %s" % (output_dir / "summary.json"))
    print("  CSV:    %s" % (output_dir / "summary.csv"))
    return 0 if summary["all_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
