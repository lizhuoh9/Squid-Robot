"""Was C0 (the image deployed on 2026-10-27) built from the sources at the pinned commit?

Reads C0's image, ELF and the sources from git at commit deef5845, whatever is checked out. From the
ELF's debug information it takes the values of the depth_cfg constants and kEnabled, and the
declaration line of every function, variable and member declared in firmware/main and the model
headers (both folders are flat). A declaration whose name is not on that line of the source means the
file differs from the one C0 was built from. Lambda captures are named with a leading "__" by GCC, so
that prefix is ignored; an implicit capture of `this` has no name on its line and is only reported.

Exits with 1 if the image or the ELF is not C0's, a constant differs from the sources, a declaration
other than an implicit `this` capture is off its line, or the number of compilation units is not 22.

Run with a python that has pyelftools (the ESP-IDF venv has it); the repository defaults to the one
this file is in:
  python -I check_c0_sources.py [repository]
"""
import hashlib
import io
import logging
import struct
import subprocess
import sys
from pathlib import Path
from typing import NamedTuple

from elftools.elf.elffile import ELFFile

log = logging.getLogger("check_c0_sources")

REPO = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).resolve().parents[2]
COMMIT = "deef58455f6f395e1b4b07fdaa39611760143435"
FOLDER = "7_ProposalA_Learning"
IMAGE = f"{FOLDER}/firmware/build-learning/squid_robot_a_recorder"
C0_SHA256 = "b825bc7c7de97209c242a05475a0b77f3c60de5631a76bca056a830b0c6f1ac9"
SOURCE_DIRS = (f"{FOLDER}/firmware/main", f"{FOLDER}/artifacts/hybrid_mpc_v1")
SOURCE_SUFFIXES = (".c", ".cpp", ".h", ".hpp")
CU_FOLDER = f"/{FOLDER}/firmware/main/"
COMPILATION_UNITS = 22  # SRCS in firmware/main/CMakeLists.txt
# The values in firmware/main/DepthController.h and artifacts/hybrid_mpc_v1/ProposalStudentConfig.h at
# COMMIT, as float32 rounded to 7 decimals (DUTY_MIN = 80/255).
CONSTANTS = {"KP": 0.081, "KD": 0.54, "KI": 0.009, "I_LIMIT": 1.5, "G_SINK": 0.75, "G_RISE": 4.5,
             "A_REST_REF": 1.1, "DEPTH_REF": 40.0, "ALPHA": 0.03, "DUTY_MIN": 0.3137255, "MOD_WINDOW_MS": 1000,
             "FLIP_MIN_DUTY": 0.05, "kEnabled": 1}
DECLARATIONS = ("DW_TAG_subprogram", "DW_TAG_variable", "DW_TAG_member")
IMPLICIT_THIS = "__this"


class Mismatch(NamedTuple):
    file: str
    line: int
    tag: str
    name: str
    text: str

    def __str__(self) -> str:
        return f"{self.file}:{self.line} {self.tag} {self.name!r}: {self.text.strip()[:80]!r}"


def git(*args: str) -> bytes:
    return subprocess.run(["git", "-C", str(REPO), *args], capture_output=True, check=True).stdout


def source_lines() -> dict[str, list[str]]:
    lines = {}
    for folder in SOURCE_DIRS:
        for path in git("ls-tree", "--name-only", COMMIT, folder + "/").decode().splitlines():
            if path.endswith(SOURCE_SUFFIXES):
                text = git("show", f"{COMMIT}:{path}").decode("utf-8", errors="replace")
                lines[path.rsplit("/", 1)[-1]] = text.splitlines()
    return lines


def const_value(die) -> object:
    attr = die.attributes["DW_AT_const_value"]
    if attr.form.startswith("DW_FORM_block"):
        raw = bytes(attr.value)
        return round(struct.unpack("<f", raw)[0], 7) if len(raw) == 4 else raw.hex()
    return attr.value


def check_cu(dwarf, cu, lines: dict[str, list[str]]) -> tuple[dict, int, list[Mismatch]]:
    files = dwarf.line_program_for_CU(cu).header["file_entry"]
    version = cu.header["version"]
    constants, checked, mismatches = {}, 0, []
    for die in cu.iter_DIEs():
        attr = die.attributes.get("DW_AT_name")
        if attr is None:
            continue
        name = attr.value.decode(errors="replace")
        if die.tag == "DW_TAG_variable" and name in CONSTANTS and "DW_AT_const_value" in die.attributes:
            constants[name] = const_value(die)
        if die.tag not in DECLARATIONS or "DW_AT_decl_line" not in die.attributes:
            continue
        index = die.attributes["DW_AT_decl_file"].value
        entry = files[index if version >= 5 else index - 1]
        fname = entry.name.decode(errors="replace").replace("\\", "/").rsplit("/", 1)[-1]
        if fname not in lines:
            continue
        line_no = die.attributes["DW_AT_decl_line"].value
        text = lines[fname][line_no - 1] if 0 < line_no <= len(lines[fname]) else ""
        checked += 1
        if name.lstrip("_") not in text:
            mismatches.append(Mismatch(fname, line_no, die.tag, name, text))
    return constants, checked, mismatches


def main() -> int:
    sys.stdout.reconfigure(errors="backslashreplace")  # source lines quote Chinese comments
    logging.basicConfig(level=logging.INFO, format="%(message)s", stream=sys.stdout)
    image, elf = git("show", f"{COMMIT}:{IMAGE}.bin"), git("show", f"{COMMIT}:{IMAGE}.elf")
    image_ok = hashlib.sha256(image).hexdigest() == C0_SHA256
    elf_ok = hashlib.sha256(elf).digest()[:8] == image[32 + 144: 32 + 152]  # esp_app_desc_t.app_elf_sha256
    log.info("commit %s: C0 image SHA-256 %s (expected %s); ELF matches the image: %s",
             COMMIT[:8], hashlib.sha256(image).hexdigest()[:8], C0_SHA256[:8], elf_ok)
    lines = source_lines()
    total = units = 0
    unexpected, wrong_constants, seen = [], set(), set()
    dwarf = ELFFile(io.BytesIO(elf)).get_dwarf_info()
    for cu in dwarf.iter_CUs():
        cu_name = cu.get_top_DIE().attributes["DW_AT_name"].value.decode(errors="replace").replace("\\", "/")
        if CU_FOLDER not in cu_name:
            continue
        constants, checked, mismatches = check_cu(dwarf, cu, lines)
        units += 1
        total += checked
        log.info("%-24s %4d declarations, %d not on their line", cu_name.rsplit("/", 1)[-1], checked, len(mismatches))
        for item in mismatches[:5]:
            log.info("    %s", item)
        unexpected += [m for m in mismatches if m.name != IMPLICIT_THIS]
        if constants:
            log.info("    %s", ", ".join(f"{k} = {v}" for k, v in constants.items()))
            wrong_constants |= {k for k, v in constants.items() if v != CONSTANTS[k]}
            seen |= constants.keys()
    log.info("%d compilation units, %d declarations checked", units, total)
    failures = [text for failed, text in (
        (not image_ok, "the image is not C0"), (not elf_ok, "the ELF does not belong to the image"),
        (bool(wrong_constants), f"constants differ from the sources: {sorted(wrong_constants)}"),
        (seen != set(CONSTANTS), f"constants missing from the debug information: {sorted(set(CONSTANTS) - seen)}"),
        (bool(unexpected), f"{len(unexpected)} declarations off their line"),
        (units != COMPILATION_UNITS, f"{units} compilation units, expected {COMPILATION_UNITS}")) if failed]
    log.info("result: %s", "; ".join(failures) if failures else "C0 matches the sources at the commit")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
