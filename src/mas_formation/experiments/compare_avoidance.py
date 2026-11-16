"""Confronto riproducibile reattivo–predittivo, senza finestre grafiche.

Eseguire con ``python -m mas_formation.experiments.compare_avoidance``.
Le metriche sono campionate: non certificano sicurezza o convergenza globale.
"""

from __future__ import annotations

import argparse
import csv
import json
import platform
from pathlib import Path

import numpy as np

from .basic_formation import (
    DynamicNavigationExperimentResult,
    MovingObstacleExperimentResult,
    run_dynamic_navigation_experiment,
    run_moving_obstacle_experiment,
)

DynamicResult = MovingObstacleExperimentResult | DynamicNavigationExperimentResult
AGENT_RADIUS = 0.15  # Raggio usato da entrambi gli scenari in basic_formation.py.
GOAL_TOLERANCE = 0.1


def measure_result(result: DynamicResult) -> tuple[dict, dict[str, np.ndarray]]:
    """Misura clearance reale, errore sugli archi e arrivo del baricentro.

    I centri reali registrati (non quelli predetti) determinano la clearance.
    Un frame di contatto può coinvolgere più coppie: non è un evento distinto.
    """
    radii = np.array([obstacle.radius for obstacle in result.obstacles])
    displacement = result.positions[:, :, None, :] - result.obstacle_positions[:, None, :, :]
    clearance = np.linalg.norm(displacement, axis=-1) - AGENT_RADIUS - radii
    edge_errors = np.column_stack([
        np.linalg.norm(
            result.positions[:, result.agent_ids.index(edge.first_agent)]
            - result.positions[:, result.agent_ids.index(edge.second_agent)], axis=-1,
        ) - edge.desired_distance
        for edge in result.formation.edges
    ])
    active = np.flatnonzero(result.obstacle_potentials > 0.0)
    metrics = {
        "minimum_real_clearance_m": float(clearance.min()),
        "contact_frames": int(np.count_nonzero(np.any(clearance <= 0.0, axis=(1, 2)))),
        "first_repulsion_time_s": float(result.times[active[0]]) if active.size else None,
        "maximum_absolute_formation_error_m": float(np.abs(edge_errors).max()),
        "rms_formation_error_m": float(np.sqrt(np.mean(edge_errors**2))),
        "final_maximum_formation_error_m": float(np.abs(edge_errors[-1]).max()),
        "navigation_start_time_s": None,
        "first_goal_arrival_time_s": None,
        "final_goal_error_m": None,
        "goal_reached_at_end": None,
    }
    curves = {
        "time_s": result.times,
        "minimum_real_clearance_m": clearance.min(axis=(1, 2)),
        "maximum_absolute_formation_error_m": np.abs(edge_errors).max(axis=1),
        "goal_error_m": np.full_like(result.times, np.nan),
    }
    if isinstance(result, DynamicNavigationExperimentResult):
        error = np.linalg.norm(result.positions.mean(axis=1) - result.goal, axis=1)
        active_navigation = result.times >= result.navigation_start_time
        arrivals = np.flatnonzero(active_navigation & (error <= GOAL_TOLERANCE))
        metrics.update(
            navigation_start_time_s=float(result.navigation_start_time),
            first_goal_arrival_time_s=float(result.times[arrivals[0]]) if arrivals.size else None,
            final_goal_error_m=float(error[-1]),
            goal_reached_at_end=bool(error[-1] <= GOAL_TOLERANCE),
        )
        curves["goal_error_m"] = np.where(active_navigation, error, np.nan)
    return metrics, curves


def _save_plots(pairs: dict, path: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    for scenario, runs in pairs.items():
        figure, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
        for axis, (mode, (result, _)) in zip(axes, runs.items(), strict=True):
            for index, identifier in enumerate(result.agent_ids):
                trajectory = result.positions[:, index]
                axis.plot(*trajectory.T, label=f"Agente {identifier}")
                axis.scatter(*trajectory[0], marker="x", s=25)
            for index in range(len(result.obstacles)):
                axis.plot(*result.obstacle_positions[:, index].T, "--", color="gray", alpha=0.7)
            for edge in result.formation.edges:
                endpoints = result.final_positions[[result.agent_ids.index(edge.first_agent),
                                                    result.agent_ids.index(edge.second_agent)]]
                axis.plot(*endpoints.T, "k--", alpha=0.35)
            if isinstance(result, DynamicNavigationExperimentResult):
                axis.scatter(*result.goal, marker="*", color="red", s=100, label="Goal")
            axis.set(title=mode, xlabel="x [m]", ylabel="y [m]", aspect="equal")
            axis.grid(alpha=0.3)
            axis.legend(fontsize="small")
        # Stessa scala spaziale per entrambi i controllori.
        limits = [axis.axis() for axis in axes]
        for axis in axes:
            axis.set_xlim(min(limit[0] for limit in limits), max(limit[1] for limit in limits))
            axis.set_ylim(min(limit[2] for limit in limits), max(limit[3] for limit in limits))
        figure.suptitle(scenario)
        figure.savefig(path / f"{scenario}_trajectories.png", dpi=150)
        plt.close(figure)

        figure, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True, constrained_layout=True)
        for mode, (_, curves) in runs.items():
            for axis, key in zip(axes, (
                "minimum_real_clearance_m", "maximum_absolute_formation_error_m", "goal_error_m",
            ), strict=True):
                axis.plot(curves["time_s"], curves[key], label=mode)
        for axis, title in zip(axes, ("Clearance reale minima [m]", "Massimo errore di formazione [m]",
                                      "Distanza baricentro–goal [m]"), strict=True):
            axis.set_ylabel(title)
            axis.grid(alpha=0.3)
            axis.legend()
        axes[0].axhline(0.0, color="red", linestyle=":", label="contatto")
        if scenario == "moving-obstacles":
            axes[2].text(0.5, 0.5, "Scenario senza goal", transform=axes[2].transAxes, ha="center")
        else:
            axes[2].axhline(GOAL_TOLERANCE, color="gray", linestyle=":")
        axes[-1].set_xlabel("Tempo [s]")
        figure.suptitle(scenario)
        figure.savefig(path / f"{scenario}_metrics.png", dpi=150)
        plt.close(figure)


def run_comparison(
    output_dir: str | Path,
    seed: int = 42,
    agent_count: int = 3,
    duration: float = 30.0,
    dt: float = 0.01,
    obstacle_count: int = 2,
    prediction_horizon: float = 1.5,
    prediction_steps: int = 10,
) -> Path:
    """Salva configurazione, metriche, traiettorie complete e figure del confronto."""
    if not np.isfinite(prediction_horizon) or prediction_horizon <= 0:
        raise ValueError("Il confronto richiede prediction_horizon strettamente positivo.")
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    config = dict(seed=seed, agent_count=agent_count, duration=duration, dt=dt,
                  moving_obstacle_count=obstacle_count, prediction_steps=prediction_steps)
    records = []
    pairs = {}
    for scenario, runner in (("moving-obstacles", run_moving_obstacle_experiment),
                             ("dynamic-navigation", run_dynamic_navigation_experiment)):
        pairs[scenario] = {}
        for mode, horizon in (("reattivo", 0.0), ("predittivo", prediction_horizon)):
            result = runner(**config, prediction_horizon=horizon)
            metrics, curves = measure_result(result)
            records.append(dict(scenario=scenario, mode=mode, prediction_horizon_s=horizon, **metrics))
            pairs[scenario][mode] = (result, curves)
            data = dict(times=result.times, positions=result.positions, agent_ids=np.array(result.agent_ids),
                        obstacle_positions=result.obstacle_positions,
                        obstacle_radii=np.array([obstacle.radius for obstacle in result.obstacles]),
                        obstacle_velocities=np.array([obstacle.velocity for obstacle in result.obstacles]),
                        agent_radii=np.full(len(result.agent_ids), AGENT_RADIUS),
                        formation_potentials=result.potentials, obstacle_potentials=result.obstacle_potentials,
                        desired_distances=np.array([edge.desired_distance for edge in result.formation.edges]),
                        edges=np.array([(edge.first_agent, edge.second_agent) for edge in result.formation.edges]))
            if isinstance(result, DynamicNavigationExperimentResult):
                data.update(goal=result.goal, goal_potentials=result.goal_potentials,
                            navigation_start_time=np.array(result.navigation_start_time))
            np.savez_compressed(output / f"{scenario}_{mode}.npz", **data)
            with (output / f"{scenario}_{mode}.csv").open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(curves)
                writer.writerows(zip(*(np.where(np.isfinite(v), v, None) for v in curves.values()), strict=True))
        reactive, predictive = [value[0] for value in pairs[scenario].values()]
        # Nessun cambiamento casuale fra i due controllori: la sola traiettoria degli agenti differisce.
        np.testing.assert_array_equal(reactive.positions[0], predictive.positions[0])
        np.testing.assert_array_equal(reactive.obstacle_positions, predictive.obstacle_positions)

    import matplotlib
    payload = dict(configuration=dict(**config, predictive_horizon_s=prediction_horizon,
                                      goal_distance_m=8.0, agent_radius_m=AGENT_RADIUS,
                                      goal_tolerance_m=GOAL_TOLERANCE),
                   environment=dict(python=platform.python_version(), numpy=np.__version__,
                                    matplotlib=matplotlib.__version__), results=records)
    (output / "metrics.json").write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    with (output / "metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    _save_plots(pairs, output)

    def formatted(value):
        return "—" if value is None else f"{value:.4f}"

    lines = ["# Confronto reattivo–predittivo", "",
             f"Configurazione: seed `{seed}`, N={agent_count}, {obstacle_count} ostacoli, "
             f"durata {duration:g} s, dt={dt:g} s, orizzonte predittivo {prediction_horizon:g} s, "
             f"{prediction_steps} campioni futuri oltre all'istante corrente.", "",
             "Le posizioni iniziali degli agenti e le intere traiettorie degli ostacoli sono identiche "
             "fra i due controllori (uguaglianza verificata automaticamente). Il goal è generato "
             "all'attivazione: il suo angolo e la distanza iniziale sono identici, ma il baricentro "
             "di attivazione può differire fra le esecuzioni.", "",
             "| Scenario | Controllo | Clearance minima [m] | Prima repulsione [s] | "
             "Errore formazione RMS [m] | Errore finale goal [m] | Primo arrivo [s] | Frame di contatto |",
             "|---|---|---:|---:|---:|---:|---:|---:|"]
    for row in records:
        lines.append(f"| {row['scenario']} | {row['mode']} | " + " | ".join(formatted(row[key]) for key in (
            "minimum_real_clearance_m", "first_repulsion_time_s", "rms_formation_error_m",
            "final_goal_error_m", "first_goal_arrival_time_s")) + f" | {row['contact_frames']} |")
    for scenario in pairs:
        lines.extend(["", f"## {scenario}", "",
                      f"![Traiettorie]({scenario}_trajectories.png)", "",
                      f"![Metriche nel tempo]({scenario}_metrics.png)"])
    lines.extend(["", "## Interpretazione e riproducibilità", "",
                  "- Clearance e contatti usano le posizioni **reali**, non quelle predette.",
                  "- Un frame di contatto ha almeno una clearance non positiva; non conta eventi distinti di collisione.",
                  "- Le metriche sono campionate: non escludono collisioni tra timestep, "
                  "né comprendono le collisioni agente–agente.",
                  "- Il primo arrivo non implica permanenza nel goal; `goal_reached_at_end` "
                  "nel JSON indica anche il rispetto finale della tolleranza.",
                  "- Il RMS di formazione include tutti i timestep e tutti gli archi, anche la fase iniziale.",
                  "- Un singolo seed illustra un caso: non dimostra superiorità generale o garanzie di sicurezza.",
                  "- `metrics.json` contiene configurazione, versioni e indicatori; `metrics.csv` la tabella; "
                  "gli altri CSV le curve e i file NPZ le traiettorie complete.", "",
                  "```bash", ".venv/bin/python -m mas_formation.experiments.compare_avoidance \\",
                  f"  --seed {seed} --agent-count {agent_count} --moving-obstacle-count {obstacle_count} \\",
                  f"  --duration {duration:g} --dt {dt:g} --prediction-horizon {prediction_horizon:g} \\",
                  f"  --prediction-steps {prediction_steps} --output-dir {output.as_posix()}", "```", ""])
    (output / "report.md").write_text("\n".join(lines), encoding="utf-8")
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="docs/results/avoidance_comparison")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--agent-count", type=int, default=3)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--moving-obstacle-count", type=int, default=2)
    parser.add_argument("--prediction-horizon", type=float, default=1.5)
    parser.add_argument("--prediction-steps", type=int, default=10)
    args = parser.parse_args()
    output = run_comparison(args.output_dir, seed=args.seed, agent_count=args.agent_count,
                            duration=args.duration, dt=args.dt, obstacle_count=args.moving_obstacle_count,
                            prediction_horizon=args.prediction_horizon, prediction_steps=args.prediction_steps)
    print(f"Confronto salvato in: {output / 'report.md'}")


if __name__ == "__main__":
    main()
