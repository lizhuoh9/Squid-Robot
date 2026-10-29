"""Compile the sources of a configured ESP-IDF project's main component, one by one, with the exact
commands CMake recorded in compile_commands.json.

ninja would first build the libraries main depends on, and with an ESP-IDF older than the project
needs, one of them fails (the bundled esp-dl uses MALLOC_CAP_SIMD, which v5.3.2 lacks), so main never
compiles. This runs main's compile commands directly. No image is made.

Run in an ESP-IDF shell after `idf.py reconfigure`:  python compile_main_sources.py <build folder>
Writes <build folder>/compile_check.txt and exits non-zero if a source fails, takes longer than
TIMEOUT_S, or compile_commands.json lists a different number of main sources than the folder holds.
"""
import json
import logging
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

log = logging.getLogger("compile_main_sources")

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
DIAGNOSTIC = re.compile(r"^(?P<file>.+?):\d+:\d+: (?P<kind>warning|error):")
SOURCE_SUFFIXES = (".c", ".cc", ".cpp")
TIMEOUT_S = 600


def normalized(path: str, base: str = ".") -> str:
    return os.path.normcase(os.path.abspath(os.path.join(base, path))).replace("\\", "/")


def compile_one(entry: dict) -> tuple[str, int, list[str]]:
    try:
        result = subprocess.run(entry["command"], cwd=entry["directory"], capture_output=True, timeout=TIMEOUT_S)
    except subprocess.TimeoutExpired:
        return entry["file"], -1, [f"no result after {TIMEOUT_S} s"]
    text = (result.stdout + result.stderr).decode("utf-8", errors="replace")
    return entry["file"], result.returncode, [ANSI.sub("", line) for line in text.splitlines()]


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    build = Path(sys.argv[1]).resolve()
    project = json.loads((build / "project_description.json").read_text(encoding="utf-8"))["project_path"]
    main_dir = normalized(os.path.join(project, "main")) + "/"
    # main and the model folder it includes (../../artifacts/hybrid_mpc_v1 from main)
    own_dirs = (main_dir, normalized(os.path.join(project, "..", "artifacts", "hybrid_mpc_v1")) + "/")
    entries = [e for e in json.loads((build / "compile_commands.json").read_text(encoding="utf-8"))
               if normalized(e["file"], e["directory"]).startswith(main_dir)]
    sources = [p for p in Path(project, "main").iterdir() if p.suffix in SOURCE_SUFFIXES]
    if not entries or len(entries) != len(sources):
        log.error("compile_commands.json lists %d sources of %s, the folder holds %d",
                  len(entries), main_dir, len(sources))
        return 2

    with ThreadPoolExecutor(max_workers=os.cpu_count()) as pool:
        results = sorted(pool.map(compile_one, entries))

    report, failed, own, other = [], 0, 0, 0
    for (file, code, lines), entry in zip(results, sorted(entries, key=lambda e: e["file"])):
        kinds = [(normalized(m["file"], entry["directory"]), m["kind"]) for m in map(DIAGNOSTIC.match, lines) if m]
        mine = sum(1 for path, kind in kinds if kind == "warning" and path.startswith(own_dirs))
        own += mine
        other += sum(1 for _, kind in kinds if kind == "warning") - mine
        failed += code != 0
        status = "FAILED" if code else "ok"
        report.append(f"{status:<6} {Path(file).name}  (warnings in own code: {mine})")
        report.extend(f"    {line}" for line in lines)
    summary = (f"{len(results)} sources of main compiled, {failed} failed; warnings: {own} in main and the model "
               f"headers, {other} in other headers")
    (build / "compile_check.txt").write_text("\n".join([summary, "", *report, ""]), encoding="utf-8")
    for line in report:
        if not line.startswith("    "):
            log.info(line)
    log.info(summary)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
