"""Primo esperimento: convergenza al triangolo senza goal né ostacoli."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from mas_formation.controllers import (
    FormationController,
    FormationThenGoalController,
    FormationWithObstacleAvoidanceController,
    PositionBasedObstacleAvoidanceController,
    StaticObstacleAvoidanceController,
)
from mas_formation.models import Agent, DynamicObstacle, Environment, TriangularFormation


@dataclass(frozen=True)
class ExperimentResult:
    """Dati registrati durante un esperimento di formazione."""

    agent_ids: tuple[int, ...]
    times: NDArray[np.float64]
    positions: NDArray[np.float64]
    potentials: NDArray[np.float64]
    formation: TriangularFormation

    @property
    def final_positions(self) -> NDArray[np.float64]:
        return self.positions[-1]


@dataclass(frozen=True)
class FormationGoalExperimentResult(ExperimentResult):
    """Dati dell'esperimento sequenziale di formazione e navigazione."""

    goal: NDArray[np.float64]
    navigation_start_time: float
    goal_potentials: NDArray[np.float64]


@dataclass(frozen=True)
class StaticObstacleExperimentResult(FormationGoalExperimentResult):
    """Dati dell'esperimento con ostacoli circolari fissi."""

    obstacles: tuple[DynamicObstacle, ...]
    obstacle_potentials: NDArray[np.float64]


@dataclass(frozen=True)
class MovingObstacleExperimentResult(ExperimentResult):
    """Dati dell'esperimento di formazione perturbata da ostacoli mobili."""

    obstacles: tuple[DynamicObstacle, ...]
    obstacle_positions: NDArray[np.float64]
    obstacle_potentials: NDArray[np.float64]


@dataclass(frozen=True)
class DynamicNavigationExperimentResult(FormationGoalExperimentResult):
    """Dati della navigazione verso un goal in presenza di ostacoli mobili."""

    obstacles: tuple[DynamicObstacle, ...]
    obstacle_positions: NDArray[np.float64]
    obstacle_potentials: NDArray[np.float64]


def run_basic_formation_experiment(
    dt: float = 0.01, duration: float = 15.0, seed: int | None = None
) -> ExperimentResult:
    """Simula tre agenti casuali che raggiungono un triangolo equilatero.

    L'esperimento isola il controllore di formazione: non sono presenti né
    obiettivo di navigazione né ostacoli. ``seed`` rende riproducibile
    l'estrazione casuale delle condizioni iniziali; se è ``None``, ogni
    esecuzione usa condizioni iniziali diverse.
    """
    if dt <= 0 or not np.isfinite(dt):
        raise ValueError("dt deve essere finito e strettamente positivo.")
    if duration <= 0 or not np.isfinite(duration):
        raise ValueError("duration deve essere finita e strettamente positiva.")

    steps = round(duration / dt)
    if not np.isclose(steps * dt, duration):
        raise ValueError("duration deve essere un multiplo intero di dt.")

    formation = TriangularFormation.from_desired_distances(
        agent_ids=(0, 1, 2),
        desired_distances={(0, 1): 2.0, (0, 2): 2.0, (1, 2): 2.0},
    )
    controller = FormationController(formation=formation, gain=1.0)
    initial_positions = _random_initial_positions(np.random.default_rng(seed), count=len(formation.agent_ids))
    agents = [
        Agent(identifier=identifier, position=initial_positions[index], max_speed=3.0)
        for index, identifier in enumerate(formation.agent_ids)
    ]
    environment = Environment(agents=agents)

    times = np.linspace(0.0, duration, steps + 1)
    positions = np.empty((steps + 1, len(agents), 2), dtype=float)
    potentials = np.empty(steps + 1, dtype=float)

    def positions_by_id() -> dict[int, NDArray[np.float64]]:
        return {agent.identifier: agent.position.copy() for agent in environment.agents}

    current_positions = positions_by_id()
    positions[0] = [current_positions[identifier] for identifier in formation.agent_ids]
    potentials[0] = controller.potential(current_positions)

    for step in range(steps):
        controls = controller.controls(current_positions)
        environment.step(controls, dt)
        current_positions = positions_by_id()
        positions[step + 1] = [current_positions[identifier] for identifier in formation.agent_ids]
        potentials[step + 1] = controller.potential(current_positions)

    return ExperimentResult(
        agent_ids=tuple(formation.agent_ids),
        times=times,
        positions=positions,
        potentials=potentials,
        formation=formation,
    )


def run_formation_goal_experiment(
    dt: float = 0.01,
    duration: float = 25.0,
    seed: int | None = None,
    goal_distance: float = 8.0,
) -> FormationGoalExperimentResult:
    """Raggiunge la formazione e solo dopo trasla il MAS verso un goal casuale."""
    if dt <= 0 or not np.isfinite(dt):
        raise ValueError("dt deve essere finito e strettamente positivo.")
    if duration <= 0 or not np.isfinite(duration):
        raise ValueError("duration deve essere finita e strettamente positiva.")
    if goal_distance <= 0 or not np.isfinite(goal_distance):
        raise ValueError("goal_distance deve essere finita e strettamente positiva.")

    steps = round(duration / dt)
    if not np.isclose(steps * dt, duration):
        raise ValueError("duration deve essere un multiplo intero di dt.")

    formation = TriangularFormation.from_desired_distances(
        agent_ids=(0, 1, 2),
        desired_distances={(0, 1): 2.0, (0, 2): 2.0, (1, 2): 2.0},
    )
    initial_positions = _random_initial_positions(np.random.default_rng(seed), count=len(formation.agent_ids))
    environment = Environment([
        Agent(identifier=identifier, position=initial_positions[index], max_speed=3.0)
        for index, identifier in enumerate(formation.agent_ids)
    ])
    mission_controller = FormationThenGoalController(
        formation_controller=FormationController(formation=formation, gain=1.0),
        goal_distance=goal_distance,
        formation_tolerance=1e-3,
        navigation_gain=0.6,
        goal_tolerance=0.1,
        seed=seed,
    )

    times = np.linspace(0.0, duration, steps + 1)
    positions = np.empty((steps + 1, len(formation.agent_ids), 2), dtype=float)
    potentials = np.empty(steps + 1, dtype=float)
    goal_potentials = np.full(steps + 1, np.nan, dtype=float)
    navigation_start_time: float | None = None

    def positions_by_id() -> dict[int, NDArray[np.float64]]:
        return {agent.identifier: agent.position.copy() for agent in environment.agents}

    current_positions = positions_by_id()
    positions[0] = [current_positions[identifier] for identifier in formation.agent_ids]
    potentials[0] = mission_controller.formation_controller.potential(current_positions)

    for step in range(steps):
        controls = mission_controller.controls(current_positions)
        if mission_controller.navigation_active and navigation_start_time is None:
            navigation_start_time = times[step]
        current_goal_potential = mission_controller.goal_potential(current_positions)
        goal_potentials[step] = np.nan if current_goal_potential is None else current_goal_potential
        environment.step(controls, dt)
        current_positions = positions_by_id()
        positions[step + 1] = [current_positions[identifier] for identifier in formation.agent_ids]
        potentials[step + 1] = mission_controller.formation_controller.potential(current_positions)
        next_goal_potential = mission_controller.goal_potential(current_positions)
        goal_potentials[step + 1] = np.nan if next_goal_potential is None else next_goal_potential

    if mission_controller.goal is None or navigation_start_time is None:
        raise RuntimeError("La formazione non è stata raggiunta nella durata selezionata.")
    return FormationGoalExperimentResult(
        agent_ids=tuple(formation.agent_ids),
        times=times,
        positions=positions,
        potentials=potentials,
        formation=formation,
        goal=mission_controller.goal,
        navigation_start_time=navigation_start_time,
        goal_potentials=goal_potentials,
    )


def run_static_obstacle_experiment(
    dt: float = 0.01,
    duration: float = 25.0,
    seed: int | None = None,
    goal_distance: float = 8.0,
    obstacle_count: int = 3,
) -> StaticObstacleExperimentResult:
    """Esegue formazione, navigazione e evitamento di più ostacoli fissi."""
    if dt <= 0 or not np.isfinite(dt):
        raise ValueError("dt deve essere finito e strettamente positivo.")
    if duration <= 0 or not np.isfinite(duration):
        raise ValueError("duration deve essere finita e strettamente positiva.")
    if goal_distance <= 0 or not np.isfinite(goal_distance):
        raise ValueError("goal_distance deve essere finita e strettamente positiva.")
    if not isinstance(obstacle_count, int) or not 1 <= obstacle_count <= 5:
        raise ValueError("obstacle_count deve essere un intero compreso fra 1 e 5.")

    steps = round(duration / dt)
    if not np.isclose(steps * dt, duration):
        raise ValueError("duration deve essere un multiplo intero di dt.")

    formation = TriangularFormation.from_desired_distances(
        agent_ids=(0, 1, 2),
        desired_distances={(0, 1): 2.0, (0, 2): 2.0, (1, 2): 2.0},
    )
    generator = np.random.default_rng(seed)
    initial_positions = _random_initial_positions(generator, count=len(formation.agent_ids))
    initial_centroid = initial_positions.mean(axis=0)
    goal_angle = generator.uniform(0.0, 2.0 * np.pi)
    goal_direction = np.array((np.cos(goal_angle), np.sin(goal_angle)))
    obstacles = _generate_static_obstacles(
        initial_centroid=initial_centroid,
        goal_direction=goal_direction,
        goal_distance=goal_distance,
        count=obstacle_count,
    )
    agents = [
        Agent(identifier=identifier, position=initial_positions[index], radius=0.15, max_speed=3.0)
        for index, identifier in enumerate(formation.agent_ids)
    ]
    environment = Environment(agents=agents, obstacles=list(obstacles))
    obstacle_controller = StaticObstacleAvoidanceController(
        agent_radii={agent.identifier: agent.radius for agent in agents},
        obstacles=obstacles,
        gain=0.3,
        influence_distance=2.0,
        minimum_clearance=0.05,
    )
    mission_controller = FormationThenGoalController(
        formation_controller=FormationController(formation=formation, gain=1.0),
        goal_distance=goal_distance,
        formation_tolerance=1e-3,
        navigation_gain=0.6,
        goal_tolerance=0.1,
        goal_angle=goal_angle,
        obstacle_controller=obstacle_controller,
    )

    times = np.linspace(0.0, duration, steps + 1)
    positions = np.empty((steps + 1, len(formation.agent_ids), 2), dtype=float)
    potentials = np.empty(steps + 1, dtype=float)
    goal_potentials = np.full(steps + 1, np.nan, dtype=float)
    obstacle_potentials = np.empty(steps + 1, dtype=float)
    navigation_start_time: float | None = None

    def positions_by_id() -> dict[int, NDArray[np.float64]]:
        return {agent.identifier: agent.position.copy() for agent in environment.agents}

    current_positions = positions_by_id()
    positions[0] = [current_positions[identifier] for identifier in formation.agent_ids]
    potentials[0] = mission_controller.formation_controller.potential(current_positions)
    obstacle_potentials[0] = obstacle_controller.potential(current_positions)
    for step in range(steps):
        controls = mission_controller.controls(current_positions)
        if mission_controller.navigation_active and navigation_start_time is None:
            navigation_start_time = times[step]
        current_goal_potential = mission_controller.goal_potential(current_positions)
        goal_potentials[step] = np.nan if current_goal_potential is None else current_goal_potential
        environment.step(controls, dt)
        current_positions = positions_by_id()
        positions[step + 1] = [current_positions[identifier] for identifier in formation.agent_ids]
        potentials[step + 1] = mission_controller.formation_controller.potential(current_positions)
        next_goal_potential = mission_controller.goal_potential(current_positions)
        goal_potentials[step + 1] = np.nan if next_goal_potential is None else next_goal_potential
        obstacle_potentials[step + 1] = obstacle_controller.potential(current_positions)

    if mission_controller.goal is None or navigation_start_time is None:
        raise RuntimeError("La formazione non è stata raggiunta nella durata selezionata.")
    return StaticObstacleExperimentResult(
        agent_ids=tuple(formation.agent_ids),
        times=times,
        positions=positions,
        potentials=potentials,
        formation=formation,
        goal=mission_controller.goal,
        navigation_start_time=navigation_start_time,
        goal_potentials=goal_potentials,
        obstacles=obstacles,
        obstacle_potentials=obstacle_potentials,
    )


def run_moving_obstacle_experiment(
    dt: float = 0.01,
    duration: float = 20.0,
    moving_obstacle_count: int = 1,
    seed: int | None = None,
) -> MovingObstacleExperimentResult:
    """Mantiene una formazione immobile, perturbata da ostacoli mobili casuali.

    Non sono presenti goal né termine di navigazione. Gli agenti partono già
    nella formazione equilatera desiderata, perciò si muovono esclusivamente
    quando l'APF repulsivo degli ostacoli mobili entra nel raggio d'influenza.
    Ogni ostacolo ha una direzione, velocità e punto di attraversamento casuali
    ma costanti, quindi percorre una traiettoria rettilinea.
    """
    if dt <= 0 or not np.isfinite(dt):
        raise ValueError("dt deve essere finito e strettamente positivo.")
    if duration <= 0 or not np.isfinite(duration):
        raise ValueError("duration deve essere finita e strettamente positiva.")
    if not isinstance(moving_obstacle_count, int) or not 1 <= moving_obstacle_count <= 3:
        raise ValueError("moving_obstacle_count deve essere un intero compreso fra 1 e 3.")

    steps = round(duration / dt)
    if not np.isclose(steps * dt, duration):
        raise ValueError("duration deve essere un multiplo intero di dt.")

    formation = TriangularFormation.equilateral((0, 1, 2), side_length=2.0)
    sqrt3 = np.sqrt(3.0)
    agents = [
        Agent(identifier=0, position=(-1.0, -sqrt3 / 3.0), radius=0.15, max_speed=3.0),
        Agent(identifier=1, position=(1.0, -sqrt3 / 3.0), radius=0.15, max_speed=3.0),
        Agent(identifier=2, position=(0.0, 2.0 * sqrt3 / 3.0), radius=0.15, max_speed=3.0),
    ]
    generator = np.random.default_rng(seed)
    obstacles = _generate_moving_obstacles(generator, moving_obstacle_count)
    environment = Environment(agents=agents, obstacles=list(obstacles))
    obstacle_controller = PositionBasedObstacleAvoidanceController(
        agent_radii={agent.identifier: agent.radius for agent in agents},
        obstacles=obstacles,
        gain=0.3,
        influence_distance=2.0,
        minimum_clearance=0.05,
    )
    controller = FormationWithObstacleAvoidanceController(
        formation_controller=FormationController(formation=formation, gain=1.0),
        obstacle_controller=obstacle_controller,
    )

    times = np.linspace(0.0, duration, steps + 1)
    positions = np.empty((steps + 1, len(agents), 2), dtype=float)
    obstacle_positions = np.empty((steps + 1, len(obstacles), 2), dtype=float)
    potentials = np.empty(steps + 1, dtype=float)
    obstacle_potentials = np.empty(steps + 1, dtype=float)

    def agent_positions_by_id() -> dict[int, NDArray[np.float64]]:
        return {agent.identifier: agent.position.copy() for agent in environment.agents}

    current_positions = agent_positions_by_id()
    positions[0] = [current_positions[identifier] for identifier in formation.agent_ids]
    obstacle_positions[0] = [obstacle.position for obstacle in environment.obstacles]
    potentials[0] = controller.formation_controller.potential(current_positions)
    obstacle_potentials[0] = obstacle_controller.potential(current_positions)

    for step in range(steps):
        environment.step(controller.controls(current_positions), dt)
        current_positions = agent_positions_by_id()
        positions[step + 1] = [current_positions[identifier] for identifier in formation.agent_ids]
        obstacle_positions[step + 1] = [obstacle.position for obstacle in environment.obstacles]
        potentials[step + 1] = controller.formation_controller.potential(current_positions)
        obstacle_potentials[step + 1] = obstacle_controller.potential(current_positions)

    return MovingObstacleExperimentResult(
        agent_ids=tuple(formation.agent_ids),
        times=times,
        positions=positions,
        potentials=potentials,
        formation=formation,
        obstacles=obstacles,
        obstacle_positions=obstacle_positions,
        obstacle_potentials=obstacle_potentials,
    )


def run_dynamic_navigation_experiment(
    dt: float = 0.01,
    duration: float = 30.0,
    seed: int | None = None,
    goal_distance: float = 8.0,
    moving_obstacle_count: int = 2,
) -> DynamicNavigationExperimentResult:
    """Naviga verso un goal casuale in presenza di ostacoli mobili casuali.

    Il MAS raggiunge prima la formazione. Goal e ostacoli sono generati in modo
    riproducibile con ``seed``; gli ostacoli percorrono rette casuali che
    attraversano il corridoio previsto fra il baricentro iniziale e il goal.
    """
    if dt <= 0 or not np.isfinite(dt):
        raise ValueError("dt deve essere finito e strettamente positivo.")
    if duration <= 0 or not np.isfinite(duration):
        raise ValueError("duration deve essere finita e strettamente positiva.")
    if goal_distance <= 0 or not np.isfinite(goal_distance):
        raise ValueError("goal_distance deve essere finita e strettamente positiva.")
    if not isinstance(moving_obstacle_count, int) or not 1 <= moving_obstacle_count <= 3:
        raise ValueError("moving_obstacle_count deve essere un intero compreso fra 1 e 3.")

    steps = round(duration / dt)
    if not np.isclose(steps * dt, duration):
        raise ValueError("duration deve essere un multiplo intero di dt.")

    formation = TriangularFormation.equilateral((0, 1, 2), side_length=2.0)
    generator = np.random.default_rng(seed)
    initial_positions = _random_initial_positions(generator, count=len(formation.agent_ids))
    initial_centroid = initial_positions.mean(axis=0)
    goal_angle = generator.uniform(0.0, 2.0 * np.pi)
    goal_direction = np.array((np.cos(goal_angle), np.sin(goal_angle)))
    obstacles = _generate_route_crossing_moving_obstacles(
        generator=generator,
        count=moving_obstacle_count,
        initial_agent_positions=initial_positions,
        initial_centroid=initial_centroid,
        goal_direction=goal_direction,
        goal_distance=goal_distance,
    )
    agents = [
        Agent(identifier=identifier, position=initial_positions[index], radius=0.15, max_speed=3.0)
        for index, identifier in enumerate(formation.agent_ids)
    ]
    environment = Environment(agents=agents, obstacles=list(obstacles))
    obstacle_controller = PositionBasedObstacleAvoidanceController(
        agent_radii={agent.identifier: agent.radius for agent in agents},
        obstacles=obstacles,
        gain=0.3,
        influence_distance=2.0,
        minimum_clearance=0.05,
    )
    mission_controller = FormationThenGoalController(
        formation_controller=FormationController(formation=formation, gain=1.0),
        goal_distance=goal_distance,
        formation_tolerance=1e-3,
        navigation_gain=0.6,
        goal_tolerance=0.1,
        goal_angle=goal_angle,
        obstacle_controller=obstacle_controller,
    )

    times = np.linspace(0.0, duration, steps + 1)
    positions = np.empty((steps + 1, len(agents), 2), dtype=float)
    obstacle_positions = np.empty((steps + 1, len(obstacles), 2), dtype=float)
    potentials = np.empty(steps + 1, dtype=float)
    goal_potentials = np.full(steps + 1, np.nan, dtype=float)
    obstacle_potentials = np.empty(steps + 1, dtype=float)
    navigation_start_time: float | None = None

    def positions_by_id() -> dict[int, NDArray[np.float64]]:
        return {agent.identifier: agent.position.copy() for agent in environment.agents}

    current_positions = positions_by_id()
    positions[0] = [current_positions[identifier] for identifier in formation.agent_ids]
    obstacle_positions[0] = [obstacle.position for obstacle in environment.obstacles]
    potentials[0] = mission_controller.formation_controller.potential(current_positions)
    obstacle_potentials[0] = obstacle_controller.potential(current_positions)

    for step in range(steps):
        controls = mission_controller.controls(current_positions)
        if mission_controller.navigation_active and navigation_start_time is None:
            navigation_start_time = times[step]
        goal_potential = mission_controller.goal_potential(current_positions)
        goal_potentials[step] = np.nan if goal_potential is None else goal_potential
        environment.step(controls, dt)
        current_positions = positions_by_id()
        positions[step + 1] = [current_positions[identifier] for identifier in formation.agent_ids]
        obstacle_positions[step + 1] = [obstacle.position for obstacle in environment.obstacles]
        potentials[step + 1] = mission_controller.formation_controller.potential(current_positions)
        next_goal_potential = mission_controller.goal_potential(current_positions)
        goal_potentials[step + 1] = np.nan if next_goal_potential is None else next_goal_potential
        obstacle_potentials[step + 1] = obstacle_controller.potential(current_positions)

    if mission_controller.goal is None or navigation_start_time is None:
        raise RuntimeError("La formazione non è stata raggiunta nella durata selezionata.")
    return DynamicNavigationExperimentResult(
        agent_ids=tuple(formation.agent_ids),
        times=times,
        positions=positions,
        potentials=potentials,
        formation=formation,
        goal=mission_controller.goal,
        navigation_start_time=navigation_start_time,
        goal_potentials=goal_potentials,
        obstacles=obstacles,
        obstacle_positions=obstacle_positions,
        obstacle_potentials=obstacle_potentials,
    )


def _generate_route_crossing_moving_obstacles(
    generator: np.random.Generator,
    count: int,
    initial_agent_positions: NDArray[np.float64],
    initial_centroid: NDArray[np.float64],
    goal_direction: NDArray[np.float64],
    goal_distance: float,
) -> tuple[DynamicObstacle, ...]:
    """Crea rette casuali che incrociano il corridoio di navigazione dopo l'avvio."""
    lateral_direction = np.array((-goal_direction[1], goal_direction[0]))
    obstacles: list[DynamicObstacle] = []
    for identifier in range(count):
        for _ in range(1_000):
            angle = generator.uniform(0.0, 2.0 * np.pi)
            direction = np.array((np.cos(angle), np.sin(angle)))
            speed = generator.uniform(0.35, 0.55)
            crossing_time = generator.uniform(4.0, 8.0)
            crossing_point = (
                initial_centroid
                + generator.uniform(0.30, 0.75) * goal_distance * goal_direction
                + generator.uniform(-1.0, 1.0) * lateral_direction
            )
            velocity = speed * direction
            initial_position = crossing_point - crossing_time * velocity
            if np.min(np.linalg.norm(initial_agent_positions - initial_position, axis=1)) > 3.0:
                obstacles.append(
                    DynamicObstacle(identifier=identifier, position=initial_position, velocity=velocity, radius=0.45)
                )
                break
        else:
            raise RuntimeError("Impossibile generare un ostacolo inizialmente sicuro.")
    return tuple(obstacles)


def _generate_moving_obstacles(
    generator: np.random.Generator, count: int
) -> tuple[DynamicObstacle, ...]:
    """Genera traiettorie rettilinee casuali che attraversano la zona del MAS."""
    obstacles: list[DynamicObstacle] = []
    for identifier in range(count):
        angle = generator.uniform(0.0, 2.0 * np.pi)
        direction = np.array((np.cos(angle), np.sin(angle)))
        speed = generator.uniform(0.4, 0.6)
        # Mantiene gli ostacoli inizialmente fuori dal raggio d'influenza;
        # attraverseranno la formazione nella seconda metà della simulazione.
        crossing_time = generator.uniform(12.0, 14.0)
        crossing_point = generator.uniform(-0.75, 0.75, size=2)
        velocity = speed * direction
        initial_position = crossing_point - crossing_time * velocity
        obstacles.append(
            DynamicObstacle(identifier=identifier, position=initial_position, velocity=velocity, radius=0.45)
        )
    return tuple(obstacles)


def _generate_static_obstacles(
    initial_centroid: NDArray[np.float64],
    goal_direction: NDArray[np.float64],
    goal_distance: float,
    count: int,
) -> tuple[DynamicObstacle, ...]:
    """Dispone ostacoli separati lungo la rotta prevista del baricentro."""
    lateral_direction = np.array((-goal_direction[1], goal_direction[0]))
    # Gli ostacoli iniziano lontano dalla configurazione iniziale, così non
    # impediscono la fase preliminare di raggiungimento della formazione.
    longitudinal_positions = np.linspace(0.55, 0.80, count)
    lateral_offsets = [0.0] + [0.85 * (-1.0 if index % 2 else 1.0) for index in range(1, count)]
    return tuple(
        DynamicObstacle(
            identifier=index,
            position=(
                initial_centroid
                + fraction * goal_distance * goal_direction
                + lateral_offset * lateral_direction
            ),
            radius=0.45,
        )
        for index, (fraction, lateral_offset) in enumerate(zip(longitudinal_positions, lateral_offsets, strict=True))
    )


def _random_initial_positions(
    generator: np.random.Generator,
    count: int,
    workspace_half_size: float = 3.0,
    min_separation: float = 0.5,
    min_twice_area: float = 0.2,
) -> NDArray[np.float64]:
    """Estrae posizioni separate e non quasi-collineari per il primo test."""
    if count != 3:
        raise ValueError("L'esperimento base è definito per esattamente tre agenti.")

    for _ in range(1_000):
        points = generator.uniform(-workspace_half_size, workspace_half_size, size=(count, 2))
        pair_distances = [np.linalg.norm(points[first] - points[second])
                          for first in range(count) for second in range(first + 1, count)]
        first_side = points[1] - points[0]
        second_side = points[2] - points[0]
        twice_area = abs(float(first_side[0] * second_side[1] - first_side[1] * second_side[0]))
        if min(pair_distances) >= min_separation and twice_area >= min_twice_area:
            return points
    raise RuntimeError("Impossibile generare condizioni iniziali valide.")


def plot_basic_formation_result(result: ExperimentResult, save_path: str | Path | None = None) -> None:
    """Mostra in due finestre distinte traiettorie e potenziali dell'esperimento."""
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    # Le traiettorie dell'esperimento dinamico sono soprattutto orizzontali;
    # le altre sono prevalentemente verticali e richiedono una figura alta.
    space_figure_size = (14, 8) if isinstance(result, (MovingObstacleExperimentResult, DynamicNavigationExperimentResult)) else (11, 14)
    figure_space, axis_space = plt.subplots(figsize=space_figure_size)
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    for index, agent_id in enumerate(result.agent_ids):
        trajectory = result.positions[:, index, :]
        color = colors[index % len(colors)]
        axis_space.plot(trajectory[:, 0], trajectory[:, 1], color=color, label=f"Agente {agent_id}")
        axis_space.scatter(*trajectory[0], color=color, marker="x", s=70, zorder=3)
        axis_space.scatter(*trajectory[-1], color=color, marker="o", s=45, zorder=3)

    for edge in result.formation.edges:
        first_index = result.agent_ids.index(edge.first_agent)
        second_index = result.agent_ids.index(edge.second_agent)
        endpoints = result.final_positions[[first_index, second_index]]
        axis_space.plot(endpoints[:, 0], endpoints[:, 1], "k--", alpha=0.55, zorder=1)

    if isinstance(result, FormationGoalExperimentResult):
        axis_space.scatter(*result.goal, color="tab:red", marker="*", s=180, label="Goal", zorder=4)
        if isinstance(result, StaticObstacleExperimentResult):
            for index, obstacle in enumerate(result.obstacles):
                axis_space.add_patch(Circle(obstacle.position, obstacle.radius, color="dimgray", alpha=0.75))
                if index == 0:
                    axis_space.plot([], [], color="dimgray", linewidth=8, label="Ostacoli fissi")
            axis_space.set_title("Formazione, navigazione e ostacoli fissi")
        elif isinstance(result, DynamicNavigationExperimentResult):
            for index, obstacle in enumerate(result.obstacles):
                trajectory = result.obstacle_positions[:, index, :]
                axis_space.plot(trajectory[:, 0], trajectory[:, 1], "--", color="dimgray", alpha=0.8)
                axis_space.add_patch(Circle(trajectory[-1], obstacle.radius, color="dimgray", alpha=0.75))
                if index == 0:
                    axis_space.plot([], [], "--", color="dimgray", label="Traiettoria ostacolo mobile")
                    axis_space.plot([], [], color="dimgray", linewidth=8, label="Posizione finale ostacolo")
            axis_space.set_title("Navigazione con ostacoli mobili casuali")
        else:
            axis_space.set_title("Formazione e navigazione verso il goal")
    elif isinstance(result, MovingObstacleExperimentResult):
        for index, obstacle in enumerate(result.obstacles):
            trajectory = result.obstacle_positions[:, index, :]
            axis_space.plot(trajectory[:, 0], trajectory[:, 1], "--", color="dimgray", alpha=0.8)
            axis_space.add_patch(Circle(trajectory[-1], obstacle.radius, color="dimgray", alpha=0.75))
            if index == 0:
                axis_space.plot([], [], "--", color="dimgray", label="Traiettoria ostacolo mobile")
                axis_space.plot([], [], color="dimgray", linewidth=8, label="Posizione finale ostacolo")
        axis_space.set_title("Formazione perturbata da ostacoli mobili")
    else:
        axis_space.set_title("Convergenza della formazione triangolare")
    axis_space.set_xlabel("x [m]")
    axis_space.set_ylabel("y [m]")
    axis_space.set_aspect("equal", adjustable="box")
    axis_space.set_anchor("W")
    axis_space.grid(True, alpha=0.3)
    axis_space.legend(loc="upper left", bbox_to_anchor=(1.03, 1.0), borderaxespad=0.0)
    figure_space.subplots_adjust(
        left=0.09, right=0.70,
        top=0.90 if isinstance(result, (MovingObstacleExperimentResult, DynamicNavigationExperimentResult)) else 0.95,
        bottom=0.13 if isinstance(result, (MovingObstacleExperimentResult, DynamicNavigationExperimentResult)) else 0.08,
    )

    figure_potential, axis_potential = plt.subplots(figsize=(14, 6))

    def positive_values(values: NDArray[np.float64]) -> NDArray[np.float64]:
        """Nasconde gli zeri: non sono rappresentabili su scala logaritmica."""
        return np.where(values > 0.0, values, np.nan)

    axis_potential.plot(
        result.times, positive_values(result.potentials), color="tab:purple", label=r"$V_f$ formazione"
    )
    axis_potential.set_title("Decadimento dei potenziali (scala logaritmica)")
    if isinstance(result, FormationGoalExperimentResult):
        axis_potential.plot(
            result.times, positive_values(result.goal_potentials), color="tab:orange", label=r"$V_g$ attrattivo del goal"
        )
        if isinstance(result, (StaticObstacleExperimentResult, DynamicNavigationExperimentResult)):
            axis_potential.plot(
                result.times, positive_values(result.obstacle_potentials), color="tab:green", label=r"$V_o$ repulsivo ostacoli"
            )
        axis_potential.axvline(
            result.navigation_start_time, color="tab:red", linestyle="--", label="inizio navigazione"
        )
    elif isinstance(result, MovingObstacleExperimentResult):
        axis_potential.plot(
            result.times, positive_values(result.obstacle_potentials), color="tab:green", label=r"$V_o$ repulsivo ostacoli"
        )
    axis_potential.set_xlabel("tempo [s]")
    axis_potential.set_ylabel("potenziale")
    axis_potential.set_yscale("log")
    axis_potential.grid(True, alpha=0.3)
    axis_potential.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), borderaxespad=0.0)
    figure_potential.subplots_adjust(left=0.09, right=0.75, top=0.90, bottom=0.14)

    if save_path is not None:
        output_path = Path(save_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        suffix = output_path.suffix or ".png"
        trajectory_path = output_path.with_name(f"{output_path.stem}_trajectories{suffix}")
        potentials_path = output_path.with_name(f"{output_path.stem}_potentials{suffix}")
        figure_space.savefig(trajectory_path, dpi=150)
        figure_potential.savefig(potentials_path, dpi=150)
        print(f"Grafico traiettorie salvato in: {trajectory_path}")
        print(f"Grafico potenziali salvato in: {potentials_path}")
    plt.show()


def show_interactive_simulation(result: ExperimentResult) -> None:
    """Mostra un frame interattivo della simulazione controllato da uno slider.

    Lo slider seleziona un indice temporale discreto; a ogni aggiornamento sono
    ridisegnati posizione istantanea, storia delle traiettorie, lati della
    formazione e posizione corrente degli eventuali ostacoli mobili.
    """
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    from matplotlib.widgets import Slider

    dynamic_obstacles = isinstance(result, (MovingObstacleExperimentResult, DynamicNavigationExperimentResult))
    static_obstacles = isinstance(result, StaticObstacleExperimentResult)
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    all_points = [result.positions.reshape(-1, 2)]
    if dynamic_obstacles:
        all_points.append(result.obstacle_positions.reshape(-1, 2))
    elif static_obstacles:
        all_points.append(np.array([obstacle.position for obstacle in result.obstacles]))
    if isinstance(result, FormationGoalExperimentResult):
        all_points.append(result.goal.reshape(1, 2))
    bounds_points = np.vstack(all_points)
    lower = bounds_points.min(axis=0) - 1.0
    upper = bounds_points.max(axis=0) + 1.0

    figure, axis = plt.subplots(figsize=(12, 8))
    figure.subplots_adjust(left=0.10, right=0.78, bottom=0.18, top=0.92)
    axis.set_xlim(lower[0], upper[0])
    axis.set_ylim(lower[1], upper[1])
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.grid(True, alpha=0.3)

    agent_paths = []
    agent_markers = []
    for index, agent_id in enumerate(result.agent_ids):
        color = colors[index % len(colors)]
        (path,) = axis.plot([], [], color=color, alpha=0.7, label=f"Agente {agent_id}")
        (marker,) = axis.plot([], [], "o", color=color, markersize=8)
        agent_paths.append(path)
        agent_markers.append(marker)

    formation_edges = [axis.plot([], [], "k--", alpha=0.65)[0] for _ in result.formation.edges]
    if isinstance(result, FormationGoalExperimentResult):
        axis.scatter(*result.goal, color="tab:red", marker="*", s=180, label="Goal", zorder=4)

    obstacle_circles: list[Circle] = []
    obstacle_paths = []
    if dynamic_obstacles:
        for index, obstacle in enumerate(result.obstacles):
            (path,) = axis.plot([], [], "--", color="dimgray", alpha=0.65)
            circle = Circle((0.0, 0.0), obstacle.radius, color="dimgray", alpha=0.7)
            axis.add_patch(circle)
            obstacle_paths.append(path)
            obstacle_circles.append(circle)
        axis.plot([], [], "--", color="dimgray", label="Traiettoria ostacolo mobile")
        axis.plot([], [], color="dimgray", linewidth=8, label="Ostacolo mobile")
    elif static_obstacles:
        for obstacle in result.obstacles:
            circle = Circle(obstacle.position, obstacle.radius, color="dimgray", alpha=0.7)
            axis.add_patch(circle)
            obstacle_circles.append(circle)
        axis.plot([], [], color="dimgray", linewidth=8, label="Ostacolo fisso")

    time_label = axis.set_title("")
    axis.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), borderaxespad=0.0)
    slider_axis = figure.add_axes((0.12, 0.07, 0.56, 0.03))
    slider = Slider(
        ax=slider_axis,
        label="Indice temporale",
        valmin=0,
        valmax=len(result.times) - 1,
        valinit=0,
        valstep=1,
    )

    def update(frame_value: float) -> None:
        frame = int(frame_value)
        for index, (path, marker) in enumerate(zip(agent_paths, agent_markers, strict=True)):
            trajectory = result.positions[: frame + 1, index, :]
            path.set_data(trajectory[:, 0], trajectory[:, 1])
            marker.set_data([trajectory[-1, 0]], [trajectory[-1, 1]])
        current_positions = result.positions[frame]
        for edge, line in zip(result.formation.edges, formation_edges, strict=True):
            first_index = result.agent_ids.index(edge.first_agent)
            second_index = result.agent_ids.index(edge.second_agent)
            endpoints = current_positions[[first_index, second_index]]
            line.set_data(endpoints[:, 0], endpoints[:, 1])
        if dynamic_obstacles:
            for index, (path, circle) in enumerate(zip(obstacle_paths, obstacle_circles, strict=True)):
                trajectory = result.obstacle_positions[: frame + 1, index, :]
                path.set_data(trajectory[:, 0], trajectory[:, 1])
                circle.center = tuple(trajectory[-1])
        time_label.set_text(f"Simulazione al tempo t = {result.times[frame]:.2f} s")
        figure.canvas.draw_idle()

    slider.on_changed(update)
    update(0)
    plt.show()
