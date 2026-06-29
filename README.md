# AETHER — Autonomous Exploration & Trajectory planning under Heuristic-Estimated Risk

A modular ROS 2 software stack for autonomous UAV navigation, obstacle avoidance, and uncertainty-aware planning in simulated 3D environments.

---

## 1. Project Identity

**Project name:** AETHER (Autonomous Exploration & Trajectory planning under Heuristic-Estimated Risk).

**Short scientific description.** AETHER is a simulation-first autonomous navigation stack for a single quadrotor operating in cluttered 3D environments. It couples a sampling-based global planner with a receding-horizon local controller, maintains a probabilistic 3D occupancy representation from noisy simulated sensors, and modulates its motion using an explicit, calibrated estimate of collision risk. The defining design choice is that risk is a first-class quantity that flows through perception, planning, and control rather than a post-hoc safety check.

**Core motivation.** Most hobby-grade drone navigation projects treat obstacles as known, sensing as perfect, and planning as deterministic. Real UAVs face the opposite: maps are partial, sensors are noisy, the world moves, and a single collision is catastrophic and unrecoverable. AETHER exists to build the smallest coherent system that takes uncertainty seriously end-to-end, while staying buildable by one person who is willing to put in sustained effort.

**Project goals.**
- Build a clean, layered, ROS 2-native stack where each layer has a single responsibility and a stable interface.
- Demonstrate autonomous point-to-point navigation, obstacle avoidance, and dynamic replanning in simulation with reproducible metrics.
- Represent and propagate uncertainty from sensing through to a risk-aware cost that the planner actually optimizes against.
- Provide an evaluation harness that produces quantitative results (success rate, collision rate, energy, replanning latency) suitable for a portfolio or a workshop-paper appendix.
- Leave clean seams for research extensions (exploration, multi-agent, learning-based perception) without rewriting the core.

**Why the problem matters.** Autonomous aerial navigation underpins inspection, search-and-rescue, mapping, and logistics. The hard part is not flying to a waypoint in open air; it is doing so safely when the map is incomplete and the perception is uncertain. A system that can quantify and respect its own uncertainty is the difference between a demo and something deployable. Building that intuition in simulation, with honest metrics, is exactly the kind of work that transfers to graduate research and to real platforms running PX4/ROS 2.

**Scope honesty.** AETHER is a simulation project. It does not claim sim-to-real transfer, does not require a GPU cluster, and does not promise "fully autonomous" anything. It is a research prototype with clearly bounded assumptions, listed explicitly in the simulation section.

---

## 2. System Architecture

AETHER is organized as seven layers. Each layer is one or more ROS 2 nodes communicating over typed topics, services, and actions. The cardinal rule: data flows up (sensing → state), commands flow down (goals → setpoints), and the safety layer can preempt at any level.

```
┌──────────────────────────────────────────────────────────────┐
│  Visualization Layer        RViz2 · Open3D · Streamlit/Plotly  │
├──────────────────────────────────────────────────────────────┤
│  Logging / Evaluation       rosbag2 · structured run logs      │
├──────────────────────────────────────────────────────────────┤
│  Safety Layer               risk monitor · geofence · RTH FSM  │  ← can preempt
├──────────────────────────────────────────────────────────────┤
│  Planning Layer    global (RRT*/A*) · local (MPC) · waypoints  │
├──────────────────────────────────────────────────────────────┤
│  Perception Layer  occupancy mapping · localization · risk map │
├──────────────────────────────────────────────────────────────┤
│  Control Layer              trajectory tracker · rate control  │
├──────────────────────────────────────────────────────────────┤
│  Simulation Layer  Gazebo/PX4 SITL · sensor + noise models     │
└──────────────────────────────────────────────────────────────┘
```

**Perception layer.** Consumes raw simulated sensor streams (LiDAR point clouds, depth images, IMU) and produces three things: an ego-state estimate (pose + velocity + covariance), a probabilistic 3D occupancy map, and a derived risk field. It is the only layer that touches raw sensors, which keeps sensor-model changes from leaking into the planner.

**Planning layer.** Two-tier. The global planner answers "what corridor of free space gets me from here to the goal?" over the current map. The local planner answers "what dynamically feasible trajectory should I execute for the next horizon, given the global corridor, nearby obstacles, and current risk?" Waypoint management sits here too: it sequences mission goals and decides when a global replan is warranted.

**Control layer.** Takes the local planner's trajectory (position/velocity/acceleration setpoints) and tracks it. In SITL this hands off to PX4's inner loops via offboard setpoints; the layer itself owns the outer position/velocity tracking and the conversion to PX4-compatible commands. Keeping control separate means the planner never has to know about actuator dynamics.

**Safety layer.** Runs asynchronously and at higher priority than planning. It validates every trajectory before execution against a safety envelope, monitors battery and risk thresholds, enforces the geofence, and owns the return-to-home / emergency-land state machine. It can preempt the planning layer by publishing an override goal or a hard hover/land command.

**Simulation layer.** Gazebo (physics + worlds) plus PX4 SITL (flight dynamics + firmware behavior). It injects sensor noise, weather/wind disturbances, moving obstacles, and battery drain. Everything above it is agnostic to whether it is talking to sim or, eventually, hardware.

**Visualization layer.** Two audiences: live operator view (RViz2 for maps/trajectories/TF; Open3D for richer point-cloud inspection) and an analytical dashboard (Streamlit + Plotly) for replaying runs and inspecting risk/uncertainty over time.

**Logging / evaluation layer.** Records rosbag2 for raw replay and a structured per-run log (JSON/Parquet) of metrics. The evaluation harness reads these to compute aggregate statistics across scenario batches. This layer is what turns "it flew" into "it succeeded in 47/50 runs with a 0.4% collision rate."

**Interface contracts (the glue).** A handful of stable message types keep layers decoupled: `EgoState` (pose, twist, covariance), `OccupancyUpdate` (voxel deltas), `RiskField` (sampled risk over a local volume), `Trajectory` (timed setpoints + feasibility flag), and `SafetyStatus` (mode, battery, geofence, risk level). If these stay stable, any single layer can be reimplemented in isolation.

---

## 3. Drone Navigation System

The navigation problem is split deliberately: a slow, global, geometric planner that reasons about the whole map, and a fast, local, dynamics-aware planner that reasons about the next few seconds. This hierarchy is standard in serious UAV stacks because no single algorithm is both globally optimal and real-time feasible at quadrotor speeds.

### 3.1 3D occupancy representation

A **voxel occupancy map** (OctoMap-style octree) stores the probability that each cell is occupied, updated with a log-odds sensor model so repeated observations sharpen confidence. Octrees are chosen over dense grids because free space dominates and the octree only refines where there is structure — memory scales with surface area, not volume. The map exposes three queries the planners need: `isOccupied(point)`, `nearestObstacleDistance(point)` (via a maintained Euclidean Distance Transform), and `occupancyProbability(point)`.

**Why log-odds:** it makes Bayesian updates additive and numerically stable, and it naturally encodes "I have seen this free space many times" versus "I glimpsed this once."

### 3.2 Global planner

Primary: **RRT\*** in the 3D position space, biased toward the goal, with the octree as the collision checker. RRT\* is chosen because it handles high-dimensional, non-convex free space well, is probabilistically complete, and asymptotically optimal — and crucially, it's anytime: it returns a valid path quickly and improves it as time allows.

Secondary / fallback: **A\*** on a coarse voxel graph for short-range or highly structured queries (e.g., indoor corridors) where a grid search is faster and more predictable than sampling. The planner selects between them based on environment volume and obstacle density.

The output is not a trajectory — it's a geometric **waypoint corridor**: a sequence of collision-free positions with a clearance radius at each, defining a tube of free space the local planner must stay inside.

### 3.3 Trajectory smoothing

The raw RRT\*/A\* path is jagged and dynamically infeasible. Before handing it down, it's smoothed into a **minimum-snap polynomial trajectory** (piecewise polynomials, continuity enforced up to snap). Minimum-snap is the standard for quadrotors because snap (4th derivative of position) maps to angular acceleration, so minimizing it yields smooth, energy-efficient, trackable motion. The smoothing is constrained to remain within the corridor clearance so it can't smooth itself into a wall.

### 3.4 Local planner

**Model Predictive Control (MPC)** over a short receding horizon (e.g., 1–2 s at 20–50 Hz). At each step it solves a constrained optimization: track the reference trajectory while respecting the quadrotor dynamics model, actuator limits, and obstacle/risk constraints in the local volume. MPC is chosen because it produces dynamically feasible motion *and* incorporates constraints natively — obstacle avoidance becomes a constraint, not a heuristic nudge. When the full MPC is too slow to converge, the system degrades gracefully to a velocity-tracking controller that still respects the safety envelope.

### 3.5 Obstacle avoidance

Two complementary mechanisms:
- **Hard constraints in the MPC:** stay outside inflated obstacle boundaries (inflation = drone radius + a margin scaled by velocity and risk). These are encoded as **barrier-style soft constraints** in the cost (large penalty as the trajectory approaches an obstacle), which keeps the QP solvable while strongly discouraging contact.
- **Reactive potential-field layer** as a last-resort fallback for fast-appearing dynamic obstacles: a repulsive gradient from the nearest-obstacle distance field, blended in only when an obstacle enters a reaction radius the planner hasn't yet accounted for. Potential fields are known to have local-minima problems, which is exactly why they're the fallback and not the primary planner.

### 3.6 Dynamic replanning

Triggered by, in priority order: (a) the safety layer (risk/geofence/battery), (b) the global path becoming invalid because a newly observed obstacle blocks the corridor, (c) the goal moving, or (d) a periodic staleness timer. To avoid thrashing, replans are rate-limited and hysteretic — a small new obstacle that the local MPC can already handle does *not* trigger an expensive global replan. The global planner runs in its own thread and the current trajectory keeps executing until a new valid one is ready (plan-while-flying).

### 3.7 Uncertainty / risk estimation

This is AETHER's distinguishing feature. Three uncertainty sources are tracked and fused into a scalar **risk field** over the local volume:
1. **Map uncertainty:** cells with occupancy probability near 0.5 (unknown) are treated as risky, not free. "I don't know what's there" is penalized.
2. **State uncertainty:** the localization covariance inflates the effective drone radius — if the drone is unsure where it is, it gives obstacles more room.
3. **Dynamic-obstacle uncertainty:** predicted future positions of moving obstacles carry a growing covariance; their swept risk volume widens with prediction horizon.

These combine into a per-point risk score. The planner doesn't just avoid the *mean* obstacle position — it minimizes a **CVaR-inspired (Conditional Value-at-Risk) cost** that penalizes the worst-case tail of the risk distribution along the trajectory, not just the average. This is what makes the drone behave conservatively exactly where it's most uncertain. **Belief-space planning** ideas inform this: the planner reasons over distributions of state, not point estimates, at least in the simplified form of covariance-inflated constraints (full POMDP planning is explicitly out of scope as too expensive for a solo build).

**Why CVaR rather than a simple safety margin:** a fixed margin treats a 1%-likely collision the same as a 30%-likely one if both are "inside the margin." CVaR lets the cost grow with the probability mass in the dangerous tail, producing risk-proportionate behavior — cautious in clutter, efficient in open space.

---

## 4. Simulation Environment

The simulation is the product's foundation, so it's designed to be honest about what it does and doesn't model.

### 4.1 Stack

**Gazebo (Harmonic/Garden) + PX4 SITL.** Gazebo provides rigid-body physics, world geometry, and sensor plugins; PX4 SITL provides realistic flight-control firmware behavior so the control layer talks to the same interface it eventually would on hardware. ROS 2 connects to both via standard bridges. AirSim is noted as an alternative for photorealistic camera data but is not the primary target — its heavier footprint works against solo iteration speed.

### 4.2 Environments

- **Urban canyon:** blocks, streets, varying-height buildings; tests corridor planning and GPS-degraded behavior between tall structures.
- **Indoor / warehouse:** shelves, doorways, low ceilings; GPS-denied by definition, stresses LiDAR/depth localization and tight-clearance planning.
- **Forest / clutter:** thin, irregular obstacles (poles, branches) that stress the occupancy map's resolution and the risk estimator.
- **Open field with dynamic obstacles:** baseline for moving-obstacle prediction and reactive avoidance.

### 4.3 Disturbances and constraints

- **Moving obstacles:** scripted and randomized agents (other vehicles, pedestrians, a second drone) with configurable speed profiles.
- **Weather/noise:** wind as a force disturbance (steady + gusts via a turbulence model), affecting tracking error and energy.
- **Sensor noise:** per-sensor models (below) toggled by config so the same scenario can run "clean" or "degraded."
- **Battery constraints:** an energy model draining with thrust/maneuver intensity; feeds the safety layer's RTH math.
- **GPS-denied zones:** regions where the GPS sensor model returns no/biased fixes, forcing reliance on onboard SLAM.

### 4.4 Drone physics assumptions (stated explicitly)

A standard quadrotor rigid-body model: 6-DOF dynamics, thrust and body-rate inputs, mass and inertia from a real small platform (~1.5 kg class), first-order motor dynamics. Aerodynamic drag is modeled as a simple linear/quadratic term; blade-flapping, ground effect, and detailed prop aerodynamics are **not** modeled — these are the honest limits of the fidelity. PX4 SITL handles the inner-loop dynamics; AETHER owns everything outside the flight controller.

### 4.5 Simulation loop structure

The sim runs in (optionally) lock-stepped real-time-factor mode for reproducibility:
```
for each control tick:
    advance physics (Gazebo)         → true state
    sample sensors + inject noise     → sensor msgs
    perception updates map + state    → EgoState, OccupancyUpdate, RiskField
    safety layer evaluates            → SafetyStatus (may preempt)
    local planner solves MPC          → Trajectory
    control tracks trajectory         → setpoints to PX4
    logger records tick               → rosbag2 + metrics row
```
Determinism is pursued via fixed random seeds, fixed real-time factor, and recorded seeds per run — essential for the evaluation harness.

### 4.6 Evaluation scenarios

A scenario is a config bundle: world + start/goal + obstacle script + noise profile + seed. Scenarios are versioned so results are comparable across code changes. Examples: "warehouse-tight-clearance-cleansensors-seed{1..50}", "urban-dynamic-windy-degradedGPS-seed{1..50}". Batch runs over seed sweeps produce the statistics the evaluation framework reports.

---

## 5. Perception Stack

The perception stack is deliberately lightweight — the goal is realistic *behavior under uncertainty*, not state-of-the-art SLAM. Every sensor has an explicit noise model so the planner is always reasoning about degraded data, never ground truth.

### 5.1 Sensors and models

- **3D LiDAR (primary mapping sensor):** simulated point cloud with range-dependent Gaussian noise, dropout probability, and a max range. It is the main input to the occupancy map. Noise grows with range, so distant geometry is mapped with lower confidence — which directly feeds map uncertainty.
- **RGB camera + depth:** depth either from a simulated stereo/depth sensor or a monocular depth-estimation stub. Depth carries higher noise and a confidence mask; it supplements LiDAR for thin obstacles LiDAR can miss. Depth estimation is treated as a pluggable module so a learned model can drop in later.
- **IMU:** high-rate accel/gyro with bias random-walk and white noise, the standard model. Drives the state estimator between slower absolute fixes.
- **GPS (when available):** position fixes with configurable covariance, disabled in GPS-denied zones.

### 5.2 Localization / SLAM

Two modes, selectable by environment:
- **GPS-aided EKF** (outdoor): an Error-State Kalman Filter fusing IMU + GPS + velocity, producing pose with covariance. This is the realistic baseline for outdoor flight.
- **LiDAR-inertial odometry** (GPS-denied): scan-matching odometry (point-to-plane ICP against the local map) fused with IMU in the EKF. Loop closure and full SLAM are scoped as an extension, not a requirement — for navigation, drift-bounded local odometry is sufficient and far more buildable solo.

The estimator's **covariance output is not discarded** — it is the state-uncertainty input to the risk field (Section 3.7). This is the deliberate thread connecting perception to risk-aware planning.

### 5.3 Uncertainty handling

Every perception output is a distribution, not a point: occupancy is a probability, state is mean + covariance, depth carries a confidence mask. The perception layer's contract is that downstream consumers can always ask "how sure are you?" — and the planner is built to actually use that answer rather than collapse to the mean.

---

## 6. Safety System

Safety is a separate, higher-priority layer precisely because it must be able to override a planner that is confidently wrong. It runs at a fixed high rate and is intentionally simple and auditable — no learned components, no opaque logic.

### 6.1 Safety envelope and trajectory validation

Before any trajectory executes, the safety layer checks it against a **safety envelope**: position geofence, velocity/acceleration limits, minimum obstacle clearance (inflated by current state covariance), and minimum battery reserve to still reach home. A trajectory that violates any constraint is rejected and the drone holds its last safe state (hover) while the planner is asked for an alternative. This is the **barrier-style** principle made operational: the set of allowed states is explicitly bounded and never knowingly left.

### 6.2 Probabilistic collision prediction

Beyond geometric checking, the layer forward-simulates the planned trajectory against predicted obstacle motion (with their growing covariances) to estimate **collision probability over the horizon**. If that probability exceeds a threshold, it escalates — first requesting a replan, then, if time-critical, commanding a reactive hard stop/evade. Using probability rather than a binary check is what lets the system distinguish "tight but fine" from "genuinely dangerous."

### 6.3 State machine: modes and fail-safes

```
        ┌─────────┐  goal set   ┌──────────┐  risk/low-batt   ┌────────────┐
        │  IDLE   │ ──────────▶ │ NAVIGATE │ ───────────────▶ │ RETURN-HOME│
        └─────────┘             └──────────┘                  └────────────┘
                                     │                              │
                               critical fault                 cannot reach home
                                     ▼                              ▼
                                ┌──────────┐                  ┌────────────┐
                                │ HOVER /  │ ───────────────▶ │ EMERGENCY  │
                                │ HOLD     │   unrecoverable  │ LAND       │
                                └──────────┘                  └────────────┘
```

- **Low-battery recovery:** the layer continuously computes energy-to-home; when reserve hits the threshold it preempts the mission with a return-to-home goal. If home is unreachable on remaining charge, it commands a controlled emergency land at the safest nearby location (lowest risk, lowest occupancy).
- **Localization-loss fail-safe:** if state covariance explodes (e.g., GPS-denied + odometry diverging), the drone slows and holds rather than flying blind.
- **Comms/planner-stall fail-safe:** if no fresh valid trajectory arrives within a timeout, hover then land.

Every safety event is logged with its trigger, so post-run analysis can show exactly why the drone did what it did. Auditability is a feature.

---

## 7. Repository Structure

```
aether/
├── README.md
├── LICENSE
├── DESIGN.md                      # this document
├── pyproject.toml / package.xml   # Python + ROS 2 packaging
├── docker/                        # reproducible sim + dev environment
│   ├── Dockerfile
│   └── docker-compose.yml
│
├── src/aether/                    # core Python library (sim-agnostic logic)
│   ├── perception/
│   │   ├── occupancy_map.py        # octree / log-odds map
│   │   ├── distance_field.py       # EDT for clearance queries
│   │   ├── localization.py         # EKF / LIO front-ends
│   │   ├── risk_field.py           # uncertainty fusion → risk
│   │   └── sensor_models.py        # noise injection models
│   ├── planning/
│   │   ├── global/
│   │   │   ├── rrt_star.py
│   │   │   └── astar_grid.py
│   │   ├── local/
│   │   │   ├── mpc.py               # receding-horizon QP/NLP
│   │   │   └── potential_field.py   # reactive fallback
│   │   ├── smoothing/
│   │   │   └── min_snap.py
│   │   ├── risk/
│   │   │   └── cvar_cost.py         # risk-aware cost terms
│   │   └── waypoint_manager.py
│   ├── drone_control/
│   │   ├── trajectory_tracker.py
│   │   ├── px4_interface.py         # offboard setpoint bridge
│   │   └── dynamics_model.py        # model used inside MPC
│   ├── safety/
│   │   ├── safety_monitor.py
│   │   ├── envelope.py
│   │   ├── collision_predictor.py
│   │   └── state_machine.py
│   └── common/
│       ├── messages.py              # interface contracts / dataclasses
│       └── transforms.py
│
├── ros2_ws/                       # ROS 2 nodes wrapping the core library
│   └── src/
│       ├── aether_perception/
│       ├── aether_planning/
│       ├── aether_control/
│       ├── aether_safety/
│       ├── aether_bringup/         # launch files, params
│       └── aether_msgs/            # custom .msg / .srv / .action defs
│
├── simulations/
│   ├── worlds/                     # Gazebo world files
│   │   ├── urban_canyon.sdf
│   │   ├── warehouse.sdf
│   │   └── forest.sdf
│   ├── models/                     # drone + obstacle models
│   ├── scenarios/                  # versioned scenario configs (yaml)
│   └── launch/                     # sim bringup launch files
│
├── configs/
│   ├── drone/                      # mass, inertia, limits
│   ├── sensors/                    # noise profiles
│   ├── planner/                    # horizons, weights, CVaR alpha
│   └── safety/                     # thresholds, geofence
│
├── planners/                      # standalone planner experiments / notebooks
├── datasets/                      # recorded maps, obstacle scripts
├── logs/                          # rosbag2 + structured run logs (gitignored)
├── experiments/
│   ├── run_batch.py                # seed-sweep runner
│   ├── configs/                    # experiment definitions
│   └── results/                    # aggregated metrics (committed)
├── visualizations/
│   ├── dashboard/                  # Streamlit app
│   ├── rviz/                       # .rviz configs
│   └── open3d_tools/
├── docs/
│   ├── architecture.md
│   ├── interfaces.md               # message contracts
│   ├── algorithms.md
│   └── results.md
└── tests/
    ├── unit/                       # per-module (planners, map, risk)
    ├── integration/                # multi-node, sim-in-the-loop
    └── scenarios/                  # regression scenarios w/ expected metrics
```

The split between `src/aether/` (pure, testable Python with no ROS dependency) and `ros2_ws/` (thin ROS 2 wrappers) is intentional and important: it means the planners, map, and risk logic can be unit-tested without spinning up ROS or Gazebo, which is what keeps a solo project's iteration loop fast.

---

## 8. Implementation Roadmap

Each phase is independently demonstrable — there's always something that runs and produces a result. Phases are sized for a motivated solo developer working steadily, not for a sprint.

### Phase 1 — Minimal navigation prototype
- **Goals:** PX4 SITL + Gazebo + ROS 2 bringup; fly a quadrotor to a static waypoint in an empty world via the control layer; basic RViz visualization.
- **Outputs:** a launch file that takes off, flies to a goal, lands; TF tree and trajectory visible in RViz.
- **Milestones:** PX4 offboard control working; EKF pose published; min-snap trajectory tracked within tolerance.

### Phase 2 — Obstacle avoidance
- **Goals:** occupancy map from LiDAR; RRT*/A* global planner; MPC local planner with obstacle constraints; static-obstacle worlds.
- **Outputs:** drone navigates a cluttered static world start-to-goal without collision; map and planned path visualized live.
- **Milestones:** octree map updating in real time; global corridor → smoothed trajectory → tracked; MPC respecting clearance constraints.

### Phase 3 — Uncertainty-aware planning
- **Goals:** sensor noise models on; risk field from map + state covariance; CVaR-inspired cost in the planner; dynamic replanning with hysteresis; full safety layer + RTH + low-battery.
- **Outputs:** measurably more conservative behavior in clutter/degraded sensing; safety events logged; replan-while-flying working.
- **Milestones:** risk field published and visualized; CVaR weight demonstrably changes behavior; safety state machine passes fault-injection tests.

### Phase 4 — Aerial exploration
- **Goals:** autonomous exploration of an unknown environment (frontier-based, risk-weighted); coverage/mapping objective rather than a fixed goal.
- **Outputs:** drone autonomously builds a near-complete map of an unknown world; coverage-over-time curves.
- **Milestones:** frontier detection on the octree; exploration policy balancing information gain vs. risk vs. energy; termination criterion.

### Phase 5 — Multi-drone coordination
- **Goals:** 2–3 drones sharing a map, with decentralized deconfliction (priority/reservation or velocity-obstacle-based).
- **Outputs:** multiple drones explore/navigate a shared space without mutual collision; merged map.
- **Milestones:** inter-agent comms model; shared/merged occupancy map; collision-free coordinated runs in the evaluation harness.

### Phase 6 — Advanced research extensions
- **Goals:** pick from Section 9 and integrate one or two cleanly (e.g., learned depth, RL local policy, semantic mapping) behind existing interfaces.
- **Outputs:** a comparative study (baseline vs. extension) using the evaluation framework — the seed of a workshop paper.
- **Milestones:** extension swapped in without touching the core contracts; results table showing the trade-off.

---

## 9. Research Extensions

All of these plug into existing interfaces — the point of the layered design is that none require a rewrite. Listed roughly by buildability.

- **Active exploration:** upgrade Phase 4's frontier policy to information-theoretic next-best-view (maximize expected map-entropy reduction per unit energy). Grounded, well-studied, directly buildable on the octree.
- **Semantic mapping:** attach class labels (from a segmentation model on the RGB stream) to occupancy cells, enabling semantically-aware risk (a "person" cell is riskier than a "wall" cell).
- **Reinforcement-learning local policy:** train an RL agent for reactive avoidance as an *alternative* local planner, benchmarked head-to-head against MPC in the same scenarios. Sim-only, so reward shaping and reset are clean.
- **Event-camera perception:** add a simulated event-camera sensor model for high-speed reactive avoidance; a focused, novel perception extension.
- **NeRF / Gaussian-splatting mapping:** offline, build a photorealistic / continuous map from logged camera data as an alternative dense representation, and compare its planning utility against the octree. Treated as a mapping-research side-track, not on the live control path.
- **VLM mission planning:** a vision-language model translates a natural-language mission ("inspect the north wall, avoid the loading dock") into waypoint goals + geofence constraints. Sits cleanly above the waypoint manager; the safety layer still bounds whatever it proposes.
- **Swarm scaling:** extend Phase 5 from a handful to a larger decentralized swarm with emergent coverage behavior.
- **Federated learning:** drones collaboratively improve a shared perception model without centralizing raw data — relevant once there's a learned perception component to federate.
- **Adversarial robustness:** stress perception/planning against adversarial sensor perturbations; quantify degradation. A natural companion study once learned components exist.

Each extension is framed as a *comparison against the established baseline*, which is what makes them publishable rather than just features.

---

## 10. Evaluation Framework

The evaluation framework is what separates AETHER from a demo. Every claim about the system should be backed by numbers from seed-swept scenario batches.

### 10.1 Metrics

| Metric | Definition | Why it matters |
|---|---|---|
| **Success rate** | fraction of runs reaching goal within tolerance, no collision, no fail-safe land | top-line capability |
| **Collision rate** | collisions per run (and per km flown) | the metric that actually matters for safety |
| **Energy usage** | integrated thrust energy per run / per meter | efficiency; couples to battery realism |
| **Replanning latency** | wall-clock per global replan; MPC solve time per tick | real-time feasibility |
| **Trajectory efficiency** | path length / straight-line distance; smoothness (integrated jerk/snap) | quality of motion |
| **Clearance distribution** | min and mean obstacle distance over a run | how close to the edge it operates |
| **Uncertainty calibration** | do predicted collision probabilities match observed frequencies? (reliability diagram, Brier score) | validates the *risk* claim specifically |
| **Coverage** (Phase 4+) | fraction of explorable space mapped vs. time/energy | exploration quality |

### 10.2 Benchmarks and protocol

- **Ablations** are the core method: run the *same* scenarios with risk-awareness on vs. off, MPC vs. potential-field, clean vs. noisy sensors. The differences are the result.
- **Seed sweeps** (e.g., 50 seeds/scenario) give confidence intervals, not single anecdotes.
- **Difficulty ladders:** increasing obstacle density / decreasing clearance / increasing noise, to find where the system breaks — reporting the failure boundary is more honest and more useful than reporting only successes.

### 10.3 Reproducibility

Fixed seeds recorded per run; pinned dependencies (Docker image hash); scenario configs versioned in-repo; `experiments/run_batch.py` regenerates every results table from scratch; raw rosbags retained for the headline runs. The standard is: a reader should be able to clone, `docker compose up`, run one command, and reproduce a results table.

---

## 11. Dashboard / Visualization

Two distinct surfaces, because live operation and post-hoc analysis have different needs.

**Live operator view (RViz2 + Open3D).** RViz2 shows the TF tree, occupancy map, planned global corridor, executed trajectory, and a risk-colored point cloud in real time — it's the native ROS 2 tool and integrates for free. Open3D is used for richer, scriptable point-cloud / map inspection when RViz's rendering isn't enough.

**Analytical dashboard (Streamlit + Plotly).** A run-replay app that loads a logged run and shows:
- 3D **drone trajectory** with executed vs. planned overlay (Plotly 3D).
- **Live obstacle map** scrubbed by a time slider.
- **Risk heatmaps** — the risk field over time, the headline visualization of the project's thesis.
- **Battery state** and energy-to-home over the run.
- **Sensor uncertainty** — state covariance and map-confidence traces.
- **Planner decisions** — replan events, mode transitions, safety triggers on a timeline.
- **Simulation playback** synchronized across all panels via the time slider.

A small **comparison view** loads two runs side by side (e.g., risk-on vs. risk-off) — this is the figure that goes in the README and any writeup.

Matplotlib generates the static figures for `docs/results.md` and papers.

---

## 12. Technology Stack

- **Language / core:** Python 3.11+ (core logic), with performance-critical inner loops (octree queries, EDT, MPC) optionally in C++ or accelerated via NumPy/Numba; clean Python-first, optimize only where profiling demands.
- **Middleware:** ROS 2 (Humble/Jazzy LTS).
- **Simulation:** Gazebo (Harmonic/Garden) + PX4 SITL; AirSim optional for photorealistic camera data.
- **Flight stack:** PX4 (firmware behavior via SITL), MAVSDK / micro-ROS-XRCE or the ROS 2–PX4 bridge for offboard control.
- **Planning / optimization:** OSQP or acados/CasADi for the MPC QP/NLP; OMPL (or a clean from-scratch implementation) for RRT*; SciPy for smoothing/QP prototyping.
- **Perception / mapping:** OctoMap (or an equivalent octree), Open3D, OpenCV; an EKF/LIO implementation in NumPy or a wrapped existing filter.
- **Numerics:** NumPy, SciPy, Numba; PyTorch only where learned extensions are added.
- **Visualization:** RViz2, Open3D, Streamlit, Plotly, Matplotlib.
- **Tooling / reproducibility:** Docker + docker-compose, rosbag2, pytest, pre-commit (ruff/black), GitHub Actions CI (unit tests + a headless smoke scenario).

Everything listed is open-source and runs on a single capable workstation (a mid-range GPU helps the camera/learning paths but is not required for the core nav stack).

---

## 13. README Structure

The repo `README.md` should follow this structure (this design doc is `DESIGN.md`; the README is the shorter front door):

1. **Overview** — one-paragraph what-and-why, plus a single hero GIF of the drone navigating a cluttered world with the risk heatmap overlaid. The GIF does most of the selling.
2. **Key features** — bulleted, honest: layered ROS 2 stack, RRT*+MPC navigation, probabilistic occupancy mapping, CVaR-inspired risk-aware planning, full safety FSM, reproducible evaluation harness.
3. **Architecture** — the layer diagram from Section 2 with a two-sentence description each; link to `docs/architecture.md`.
4. **Installation** — Docker path first (`docker compose up`), native path second; explicit version pins.
5. **Quickstart** — the single command that flies a default scenario, and the single command that runs an evaluation batch.
6. **Experiments** — how to define and run a scenario sweep; where results land.
7. **Results** — the headline table (success/collision/energy/latency) and the risk-on-vs-off comparison figure. Numbers, not adjectives.
8. **Roadmap** — the phase list with checkboxes showing what's done.
9. **Limitations** — stated plainly (sim-only, simplified aerodynamics, no real loop-closure SLAM in baseline, etc.).
10. **Future work** — pointer to Section 9 extensions.
11. **Citing / acknowledgements** — references to the algorithms (RRT*, min-snap, OctoMap, CVaR) and tools used.

---

## 14. Engineering Realism — Design Principles

These are the constraints the whole document is written to honor:

- **No AGI, no "fully autonomous everything," no hype.** AETHER navigates and avoids obstacles under uncertainty in simulation. That's the claim, and it's enough.
- **Every capability is bounded by a stated assumption.** Simplified aerodynamics, local odometry instead of full SLAM, simplified belief-space planning instead of full POMDPs — each limit is named where it occurs, not buried.
- **Algorithms are chosen for reasons, and the reasons are written down.** RRT* for anytime global planning, MPC for constrained dynamic feasibility, CVaR for risk-proportionate caution, octrees for memory-efficient mapping. No algorithm is present as a buzzword.
- **It's buildable in phases, each of which runs.** A solo developer is never more than one phase from a working demo, which is what sustains a long project.
- **The core logic is decoupled from ROS and sim,** so it's unit-testable and the iteration loop stays fast.
- **Results are quantitative and reproducible,** because "it looked like it worked" is not a result.

The honest summary: AETHER is a serious, academically-inspired robotics research *prototype* — coherent enough to anchor graduate-level work and a strong portfolio, modest enough that one motivated person can actually build it.
