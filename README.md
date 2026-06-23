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

## Phased Development

| Phase | Platform | Goal |
|---|---|---|
| **A** | MATLAB | Prove the core algorithm: topology encoding → kinematic equations |
| **B** | Python (SymPy + SciPy) | Cross-platform equivalent, free from MATLAB license |
| **C** | C++ / Rust | Compilable real-time solver for servo control (100 Hz+) |

## Project Structure

```
.
├── phase-a-matlab/          # Stage A: Core algorithm validation
│   ├── encoder/             # Topology encoding language & parser
│   ├── generator/           # Encoding → kinematics equation generator
│   └── solver/              # fmincon-based IK/FK solver
├── phase-b-python/          # Stage B: Cross-platform implementation
│   ├── encoder/             # SymPy-based topology encoder
│   ├── generator/           # SymPy equation generator
│   └── solver/              # SciPy/IPOPT numerical solver
├── phase-c-rust/            # Stage C: Real-time deployment solver
├── docs/                    # Design docs & references
└── tests/                   # Cross-phase validation tests
```

## Key References

- **MRF 2.4 Inverse Kinematics Solver** — Proven pipeline: symbolic spatial transforms → closed-loop constraints → fmincon optimization
- **Timor Python** (Külz et al., 2023) — Modular robot model auto-generation
- **Modern Robotics Ch.7** (Lynch & Park, 2017) — Standard closed-chain constraint formulation
- **Decroly et al. (2023)** — Voxel-based kinematics generation at microscale

## License

MIT
