"""Copy the project and installed dependency notices into a frozen distribution.

Run after PyInstaller, using the same Python environment as the build. This
collects the environment's installed packages as a conservative superset; the
inventory is not a claim that every package is embedded in the executable.
"""

from __future__ import annotations

import argparse
import importlib.metadata as metadata
import json
from pathlib import Path
import re
import shutil
import sys


PACKAGING_DIR = Path(__file__).resolve().parent
REPOSITORY_DIR = PACKAGING_DIR.parent.parent
RUNTIME_PACKAGES = (
    "customtkinter", "numpy", "scipy", "pandas", "matplotlib", "pillow",
    "openpyxl", "pyserial",
)
NOTICE_NAME = re.compile(r"(?:licen[sc]e|copying|copyright|notice)", re.IGNORECASE)


def safe_component(value: str) -> str:
    """Keep metadata and installed file paths inside the notices directory."""
    return re.sub(r"[^A-Za-z0-9._-]", "_", value).strip(".") or "unnamed"


def notice_destination(relative_path: Path) -> Path:
    return Path(*(safe_component(part) for part in relative_path.parts))


def collect_distribution(distribution: metadata.Distribution, target: Path) -> dict:
    package = distribution.metadata
    result = {
        "name": package.get("Name", "unknown"),
        "version": distribution.version,
        "license_expression": package.get("License-Expression"),
        "license_metadata": package.get("License"),
        "license_classifiers": [
            item for item in package.get_all("Classifier", [])
            if item.startswith("License ::")
        ],
        "project_urls": package.get_all("Project-URL", []),
        "homepage": package.get("Home-page"),
        "requires": distribution.requires or [],
        "notice_files": [],
    }
    target.mkdir(parents=True, exist_ok=True)
    for relative_file in sorted(distribution.files or [], key=str):
        # Include complete license directories as well as individual LICENSE,
        # COPYING, NOTICE, and copyright files, including bundled font notices.
        if not any(NOTICE_NAME.search(part) for part in relative_file.parts):
            continue
        source = Path(distribution.locate_file(relative_file))
        if not source.is_file():
            raise RuntimeError(f"Recorded notice file is missing: {source}")
        relative_target = notice_destination(Path(str(relative_file)))
        destination = target / relative_target
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        result["notice_files"].append(relative_target.as_posix())
    # Retain exact metadata even when a wheel supplies no separate notice file.
    (target / "metadata.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return result


def collect(output: Path, required_packages: list[str]) -> dict:
    for package in required_packages:
        try:
            distribution = metadata.distribution(package)
        except metadata.PackageNotFoundError as error:
            raise RuntimeError(
                f"Required build dependency {package!r} is not installed. "
                "Use the same environment that runs PyInstaller."
            ) from error
        if not any(
            any(NOTICE_NAME.search(part) for part in file.parts)
            for file in distribution.files or []
        ):
            raise RuntimeError(
                f"Required runtime dependency {package!r} supplies no recorded "
                "license/notice files. Review its distribution before releasing."
            )

    project_license = REPOSITORY_DIR / "LICENSE"
    project_notices = REPOSITORY_DIR / "THIRD_PARTY_NOTICES.md"
    python_license = Path(sys.base_prefix) / "LICENSE.txt"
    for source in (project_license, project_notices, python_license):
        if not source.is_file():
            raise RuntimeError(f"Required notice file is missing: {source}")

    output = output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    shutil.copy2(project_license, output / "LICENSE")
    shutil.copy2(project_notices, output / "THIRD_PARTY_NOTICES.md")
    licenses = output / "licenses"
    runtime = licenses / "python-runtime"
    runtime.mkdir(parents=True, exist_ok=True)
    shutil.copy2(python_license, runtime / "LICENSE.txt")
    runtime_files = ["LICENSE.txt"]
    tcl_root = Path(sys.base_prefix) / "tcl"
    for source in sorted(tcl_root.rglob("*")) if tcl_root.is_dir() else []:
        if source.is_file() and NOTICE_NAME.search(source.name):
            relative_target = Path("tcl-tk") / source.relative_to(tcl_root)
            destination = runtime / relative_target
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, destination)
            runtime_files.append(relative_target.as_posix())

    inventory = {
        "scope": "Installed build environment; a superset of frozen application dependencies",
        "python_version": sys.version,
        "python_runtime_notices": runtime_files,
        "packages": [],
        "packages_without_separate_notice_files": [],
    }
    distributions = sorted(
        metadata.distributions(),
        key=lambda item: (item.metadata.get("Name", "").lower(), item.version),
    )
    used_names: dict[str, int] = {}
    for distribution in distributions:
        name = distribution.metadata.get("Name", "unknown")
        base = safe_component(f"{name}-{distribution.version}")
        used_names[base] = used_names.get(base, 0) + 1
        folder = base if used_names[base] == 1 else f"{base}-{used_names[base]}"
        record = collect_distribution(distribution, licenses / "packages" / folder)
        record["directory"] = f"packages/{folder}"
        inventory["packages"].append(record)
        if not record["notice_files"]:
            inventory["packages_without_separate_notice_files"].append(
                f"{name}=={distribution.version}"
            )

    (licenses / "inventory.json").write_text(
        json.dumps(inventory, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    (licenses / "README.txt").write_text(
        "The project license is ../LICENSE. Repository reference and asset\n"
        "information is ../THIRD_PARTY_NOTICES.md.\n\n"
        "inventory.json records exact package versions from the build environment,\n"
        "including tools and packages that may not be embedded in the executable.\n"
        "packages/ retains license and notice files supplied by those installed\n"
        "distributions. python-runtime/ retains the Python distribution license\n"
        "and available Tcl/Tk license files.\n\n"
        "Review packages_without_separate_notice_files in the inventory, upstream\n"
        "asset notices, and any separately added native redistributables before\n"
        "publishing a release. Collection does not determine permission to reuse\n"
        "third-party material or establish that all release obligations are met.\n",
        encoding="utf-8",
    )
    return inventory


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=PACKAGING_DIR / "dist" / "BHO")
    parser.add_argument(
        "--required-packages", nargs="+", default=list(RUNTIME_PACKAGES),
        help="Required installed runtime dependencies (override for a focused collection check).",
    )
    arguments = parser.parse_args()
    try:
        inventory = collect(arguments.output, arguments.required_packages)
    except (OSError, RuntimeError) as error:
        print(f"License collection failed: {error}", file=sys.stderr)
        return 1
    print(f"Collected notices for {len(inventory['packages'])} installed distributions.")
    missing = inventory["packages_without_separate_notice_files"]
    if missing:
        print("Review packages with metadata but no separate notice files: " + ", ".join(missing))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
