"""Punto di ingresso per gli esperimenti del progetto."""

from __future__ import annotations

import argparse

from mas_formation.experiments import (
    FormationGoalExperimentResult,
    export_interactive_html,
    plot_basic_formation_result,
    run_basic_formation_experiment,
    run_dynamic_navigation_experiment,
    run_formation_goal_experiment,
    run_moving_obstacle_experiment,
    run_static_obstacle_experiment,
    show_interactive_simulation,
)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Esperimenti per il controllo di formazione MAS.")
    parser.add_argument("--dt", type=float, default=0.01, help="Passo di integrazione [s].")
    parser.add_argument("--duration", type=float, default=25.0, help="Durata della simulazione [s].")
    parser.add_argument(
        "--experiment", choices=("formation", "formation-goal", "static-obstacles", "moving-obstacles", "dynamic-navigation"),
        default="static-obstacles",
        help="Esperimento: formazione, navigazione, ostacoli fissi, oppure ostacoli mobili.",
    )
    parser.add_argument("--goal-distance", type=float, default=8.0, help="Distanza iniziale goal-baricentro [m].")
    parser.add_argument(
        "--obstacle-count", type=int, default=3,
        help="Numero di ostacoli fissi nell'esperimento static-obstacles (da 1 a 5)."
    )
    parser.add_argument(
        "--moving-obstacle-count", type=int, default=1,
        help="Numero di ostacoli mobili nell'esperimento moving-obstacles (da 1 a 3).",
    )
    parser.add_argument("--save", type=str, help="Percorso opzionale in cui salvare il grafico.")
    parser.add_argument("--export-html", type=str, help="Esporta una pagina HTML autonoma con slider e riproduzione.")
    parser.add_argument("--seed", type=int, help="Seed opzionale per rendere riproducibili le posizioni iniziali.")
    parser.add_argument("--no-plot", action="store_true", help="Esegue l'esperimento senza aprire il grafico.")
    parser.add_argument(
        "--interactive", action="store_true",
        help="Mostra un grafico interattivo con slider temporale invece dei grafici statici.",
    )
    return parser.parse_args()


def main() -> None:
    arguments = parse_arguments()
    if arguments.experiment == "formation":
        result = run_basic_formation_experiment(dt=arguments.dt, duration=arguments.duration, seed=arguments.seed)
    elif arguments.experiment == "formation-goal":
        result = run_formation_goal_experiment(
            dt=arguments.dt,
            duration=arguments.duration,
            seed=arguments.seed,
            goal_distance=arguments.goal_distance,
        )
    elif arguments.experiment == "dynamic-navigation":
        result = run_dynamic_navigation_experiment(
            dt=arguments.dt,
            duration=arguments.duration,
            seed=arguments.seed,
            goal_distance=arguments.goal_distance,
            moving_obstacle_count=arguments.moving_obstacle_count,
        )
    elif arguments.experiment == "static-obstacles":
        result = run_static_obstacle_experiment(
            dt=arguments.dt,
            duration=arguments.duration,
            seed=arguments.seed,
            goal_distance=arguments.goal_distance,
            obstacle_count=arguments.obstacle_count,
        )
    else:
        result = run_moving_obstacle_experiment(
            dt=arguments.dt,
            duration=arguments.duration,
            moving_obstacle_count=arguments.moving_obstacle_count,
            seed=arguments.seed,
        )
    final_positions = {
        identifier: result.final_positions[index]
        for index, identifier in enumerate(result.agent_ids)
    }
    errors = result.formation.distance_errors(final_positions)

    print("Esperimento completato.")
    print(f"Potenziale iniziale: {result.potentials[0]:.6f}")
    print(f"Potenziale finale:   {result.potentials[-1]:.6e}")
    print("Errori finali sulle distanze [m]:")
    for edge, error in errors.items():
        print(f"  {edge}: {error:+.6e}")
    if isinstance(result, FormationGoalExperimentResult):
        final_centroid = result.final_positions.mean(axis=0)
        print(f"Inizio navigazione: {result.navigation_start_time:.2f} s")
        print(f"Goal casuale: ({result.goal[0]:.3f}, {result.goal[1]:.3f})")
        print(f"Distanza finale del baricentro dal goal: {((final_centroid - result.goal) @ (final_centroid - result.goal)) ** 0.5:.6f} m")

    if arguments.export_html:
        output = export_interactive_html(result, arguments.export_html)
        print(f"Simulazione interattiva esportata in: {output}")

    if arguments.interactive:
        show_interactive_simulation(result)
    elif not arguments.no_plot:
        plot_basic_formation_result(result, save_path=arguments.save)


if __name__ == "__main__":
    main()
