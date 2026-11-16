"""Build e verifica isolata della consegna (solo libreria standard nell'host).

Uso: python3 scripts/verify_clean_environment.py
Richiede accesso a PyPI o cache pip completa. Non modifica il venv del progetto.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


def verify(output: Path, constraints: Path | None = None) -> None:
    root = Path(__file__).resolve().parents[1]
    output.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    environment.pop("PYTHONHOME", None)
    environment.update(PYTHONDONTWRITEBYTECODE="1", MPLBACKEND="Agg", PIP_DISABLE_PIP_VERSION_CHECK="1")
    commands = []
    inputs = [root / "pyproject.toml", root / "main.py", Path(__file__).resolve()]
    inputs += sorted((root / "src" / "mas_formation").rglob("*.py"))
    inputs += sorted((root / "tests").rglob("*.py"))
    status = {"status": "running", "python": platform.python_version(),
              "platform": platform.platform(), "commands": commands,
              "source_sha256": {file.relative_to(root).as_posix(): hashlib.sha256(file.read_bytes()).hexdigest()
                                for file in inputs}}
    with (output / "commands.log").open("w", encoding="utf-8") as log:
        def run(arguments: list[str], cwd: Path) -> str:
            log.write("\n$ " + " ".join(arguments) + "\n")
            log.flush()
            result = subprocess.run(arguments, cwd=cwd, env=environment,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            log.write(result.stdout)
            log.flush()
            commands.append({"arguments": arguments, "cwd": str(cwd), "returncode": result.returncode})
            print(result.stdout, end="", flush=True)
            result.check_returncode()
            return result.stdout

        try:
            with tempfile.TemporaryDirectory(prefix="mas-delivery-") as temporary:
                work = Path(temporary)
                source, execution = work / "source", work / "run"
                source.mkdir()
                execution.mkdir()
                for file in ("pyproject.toml", "README.md", "main.py"):
                    shutil.copy2(root / file, source / file)
                shutil.copytree(root / "src" / "mas_formation", source / "src" / "mas_formation",
                                ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
                # Non copiare pyproject.toml nella directory di esecuzione:
                # il suo pythonpath=src non deve mascherare la wheel installata.
                shutil.copytree(root / "tests", execution / "tests",
                                ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
                run([sys.executable, "-m", "venv", str(work / "venv")], work)
                python = str(work / "venv" / ("Scripts/python.exe" if os.name == "nt" else "bin/python"))
                constraint_args = ["--constraint", str(constraints.resolve())] if constraints else []
                run([python, "-m", "pip", "wheel", "--no-deps", "--wheel-dir", str(work / "wheels"),
                     str(source)], work)
                wheel, = (work / "wheels").glob("mas_formation-*.whl")
                run([python, "-m", "pip", "install", *constraint_args, f"{wheel}[dev]"], execution)
                run([python, "-m", "pip", "check"], execution)
                run([python, "-I", "-c",
                     "import mas_formation; from pathlib import Path; import sys; "
                     "p=Path(mas_formation.__file__).resolve(); print(p); "
                     "assert Path(sys.prefix).resolve() in p.parents"], execution)
                status["pytest_output"] = run(
                    [python, "-m", "pytest", "-q", "--import-mode=importlib", "tests"], execution).strip()
                run([python, str(source / "main.py"), "--help"], execution)
                cases = [
                    ("formation", 3, 15), ("formation-goal", 3, 25),
                    ("static-obstacles", 3, 25), ("moving-obstacles", 3, 30),
                    ("dynamic-navigation", 3, 30), ("dynamic-navigation", 4, 30),
                    ("moving-obstacles", 5, 30), ("formation-goal", 6, 25), ("formation", 8, 15),
                ]
                for scenario, count, duration in cases:
                    run([python, str(source / "main.py"), "--experiment", scenario,
                         "--agent-count", str(count), "--seed", "42", "--duration", str(duration),
                         "--moving-obstacle-count", "2", "--no-plot", "--export-html",
                         str(execution / f"{scenario}_{count}.html")], execution)
                run([python, "-m", "mas_formation.experiments.compare_avoidance", "--output-dir",
                     str(execution / "comparison")], execution)
                status["comparison"] = json.loads(
                    (execution / "comparison" / "metrics.json").read_text(encoding="utf-8"))
                # Blocca le sole dipendenze; evita riferimenti alla wheel temporanea.
                versions = json.loads(run([python, "-c",
                    "import importlib.metadata as m, json; "
                    "print(json.dumps({d.metadata['Name']: d.version for d in m.distributions()}))"], execution))
                status["versions"] = versions
                (output / "requirements.txt").write_text(
                    "# Dipendenze dell'ambiente verificato; il progetto si installa separatamente.\n"
                    + "\n".join(f"{name}=={version}" for name, version in sorted(versions.items())
                                if name.lower() != "mas-formation") + "\n", encoding="utf-8")
                status["smoke_cases"] = [dict(experiment=scenario, agent_count=count, duration_s=duration)
                                         for scenario, count, duration in cases]
                status["status"] = "passed"
        except Exception as error:
            status.update(status="failed", error=str(error))
            raise
        finally:
            (output / "summary.json").write_text(json.dumps(status, indent=2) + "\n", encoding="utf-8")
    print(f"Verifica completata: {output / 'summary.json'}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("docs/results/validation"))
    parser.add_argument("--constraints", type=Path, help="Versioni bloccate opzionali per pip install.")
    args = parser.parse_args()
    verify(args.output_dir.resolve(), args.constraints)


if __name__ == "__main__":
    main()
