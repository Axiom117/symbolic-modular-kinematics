# symbolic-modular-kinematics

> Symbolic kinematics architecture for modular closed-loop mechanisms.
> Topology encoding → automatic generation of forward/inverse kinematics equations.

## Overview

This project develops a **symbol schema architecture** that encodes modular closed-loop
mechanism topologies using a custom description language (inspired by URDF/DH but
specialized for closed kinematic chains), then automatically generates the corresponding
forward and inverse kinematics equations.

The generated equations are solved via numerical optimization (fmincon / SLSQP / IPOPT),
replacing the current Simulink-based multibody simulation solver.

## Development Strategy

The roadmap still follows three milestones, but the repository starts with one unified
development environment and one shared source tree. Phases A/B/C are validation targets,
not separate codebases.

| Phase | Goal |
|---|---|
| **A** | Prove the core algorithm: topology encoding → kinematic equations |
| **B** | Reproduce the same semantics in a cross-platform toolchain |
| **C** | Export or compile a deployable runtime solver for servo control |

## Initial Project Structure

```
.
├── docs/
│   ├── README.md                # Documentation map and writing rules
│   ├── development-roadmap.md   # Current roadmap bootstrap document
│   ├── inverse-kinematics-solver-design.md
│   ├── pathplanner-architecture.md
│   ├── architecture/            # System boundaries and milestone architecture
│   ├── design/                  # DSL / IR / generator / solver design notes
│   ├── decisions/               # Architecture decision records
│   └── references/              # External references and legacy mappings
├── specs/
│   ├── README.md                # Machine-readable contracts and versioning policy
│   ├── dsl/                     # Encoding syntax and versioned examples
│   ├── schema/                  # JSON/YAML schema definitions
│   ├── ir/                      # Topology / chain / constraint IR contracts
│   └── solver-contracts/        # FK/IK I/O and diagnostics contracts
├── cases/
│   ├── README.md                # Benchmark mechanism cases and expected outputs
│   ├── shared-targets/          # Reusable target poses, bounds, and initial guesses
│   ├── planar-single-loop/      # Smallest closed-loop validation case
│   ├── three-branch-parallel/   # Parallel benchmark case
│   └── mrf-like-spatial/        # Spatial benchmark close to the MRF method
├── src/
│   ├── README.md                # Unified implementation surface
│   ├── encoding/                # Parse and normalize topology descriptions
│   ├── topology/                # Graph construction and semantic linking
│   ├── templates/               # Module geometry and joint transform templates
│   ├── generator/               # Chain extraction and constraint generation
│   ├── solver/                  # Numerical solve pipeline and diagnostics
│   ├── validation/              # Topology and contract validation
│   └── visualization/           # Topology and equation inspection helpers
├── scripts/                     # Repeatable dev entry points and automation
├── tests/
│   ├── README.md                # Regression strategy and test categories
│   ├── specs/                   # Schema and contract checks
│   ├── pipeline/                # End-to-end encoding → solve flow checks
│   ├── regression/              # Benchmark case regression
│   └── performance/             # Expression size and solve-time baselines
└── generated/
	└── README.md                # Generated equations, reports, and compiled assets
```

## Build Order

1. Freeze `specs/` for DSL, schema, IR, and solver contracts.
2. Add a minimal set of benchmark mechanisms in `cases/`.
3. Implement the core pipeline in `src/`.
4. Use `tests/` to lock regression behavior before expanding solver backends.

## Key References

- **MRF 2.4 Inverse Kinematics Solver** — Proven pipeline: symbolic spatial transforms → closed-loop constraints → fmincon optimization
- **Timor Python** (Külz et al., 2023) — Modular robot model auto-generation
- **Modern Robotics Ch.7** (Lynch & Park, 2017) — Standard closed-chain constraint formulation
- **Decroly et al. (2023)** — Voxel-based kinematics generation at microscale

## License

MIT
