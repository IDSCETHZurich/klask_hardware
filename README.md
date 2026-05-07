# KLASK Robotic Hardware Stack

<img src="docs/res//imgs/general/klask_robotic_system.png" alt="KLASK Robotic System" width="500"/>

**Autonomous robotic KLASK game platform for reinforcement learning research and education.** This repository contains everything you need to build and control a robotic KLASK table: ROS2 nodes for vision processing and motor control, CAD files for mechanical components, and comprehensive documentation.

<!-- CI pipeline (Colcon build and test) -->
<a href="https://github.com/IDSCETHZurich/klask_hardware/actions/workflows/ci.yml">
  <img alt="CI pipeline" src="https://github.com/IDSCETHZurich/klask_hardware/actions/workflows/ci.yml/badge.svg">
</a>

<!-- Docs pipeline (MkDocs/Doxygen build) -->
<a href="https://github.com/IDSCETHZurich/klask_hardware/actions/workflows/docs.yml">
  <img alt="Docs pipeline" src="https://github.com/IDSCETHZurich/klask_hardware/actions/workflows/docs.yml/badge.svg">
</a>

<!-- Docs site status -->
<a href="https://IDSCETHZurich.github.io/klask_hardware/">
  <img alt="Docs" src="https://img.shields.io/website?url=https%3A%2F%2FIDSCETHZurich.github.io%2Fklask_hardware%2F&label=docs%20site">
</a>

<!-- Stars (social) -->
<a href="https://github.com/IDSCETHZurich/klask_hardware/stargazers">
  <img alt="GitHub stars" src="https://img.shields.io/github/stars/IDSCETHZurich/klask_hardware?style=social">
</a>

<!-- Contributors -->
<a href="https://github.com/IDSCETHZurich/klask_hardware/graphs/contributors">
  <img alt="Contributors" src="https://img.shields.io/github/contributors/IDSCETHZurich/klask_hardware">
</a>

<!-- Releases -->
<a href="https://github.com/IDSCETHZurich/klask_hardware/releases">
  <img alt="Release" src="https://img.shields.io/github/v/release/IDSCETHZurich/klask_hardware?sort=semver">
</a>

## Documentation & Tutorials

All documentation lives at [https://IDSCETHZurich.github.io/klask_hardware/](https://IDSCETHZurich.github.io/klask_hardware/). You'll find guides for:

- **Hardware Assembly:** Mechanical assembly, wiring, and component installation
- **Runtime Startup:** Launching the system and running the autonomous game
- **Development Setup:** Environment configuration, Docker containers, and building documentation
- **Code Guidelines:** Coding standards and best practices
- **API Reference:** Full Doxygen documentation for all ROS2 packages

## What is the KLASK Robotic System

KLASK is a popular magnetic table game where players control magnetic pegs to hit a ball into the opponent's goal while avoiding obstacles. Researchers and students at the **Institute for Dynamic Systems and Control** (IDSC), ETH Zurich, have developed an autonomous robotic platform that can play KLASK using advanced reinforcement learning algorithms. The system provides the ability to test, benchmark and improve RL strategies in a real-world environment.

This repository contains the hardware and software components required to build and operate the KLASK robotic system. The inference node running the RL agent is hosted in the separated [klask_software](https://github.com/IDSCETHZurich/klask_software) repository.

<img src="docs/res/diagrams/repo_overview.png" alt="KLASK Repo Overview" width="500"/>

## System Architecture

The ROS2 workspace (`ros/src/`) is organized into the following packages and nodes:

### klask_imaging_pkg (Python)

- **`camera_node`** — acquires camera frames, detects and rectifies the board, and publishes the rectified board image.
- **`image_downscaler_node`** — republishes the board image at a reduced resolution and color depth.
- **`image_viewer`** — displays the rectified board image with debug overlays for ball, pegs, goals, and command velocities.

### klask_state_estimator_pkg (Python)

- **`state_estimator_node`** — estimates ball and peg positions and velocities from the rectified board image and publishes the consolidated `State` message.

### klask_motor_commander_pkg (C++)

- **`klask_motor_commander`** — top-level node that hosts the motor stack in a multi-threaded executor.
- **`ODriveController`** — coordinates both players and exposes the calibration, homing, and motor-state services.
- **`Player`** — converts `cmd_vel` into per-motor commands using CoreXY kinematics and enforces wall clearance (one instance per side).
- **`OpenLoopController`** — performs open-loop peg homing via the `HomePeg` action (one instance per side).

### klask_sprite_generator_pkg (Python)

- **`klask_sprite_generator_node`** — walks both pegs across a configurable grid and captures synchronized images and state snapshots to build a labeled dataset.
- **`process_sprites`** — offline script that segments captured frames into individual sprite PNGs with metadata for the renderer.

### klask_system_id (Python)

- **`acceleration_test_node`** — drives a player along a triangle pattern at sweeping commanded velocities and records one rosbag per velocity step.
- **`velocity_profile_node`** — drives line and circle patterns at randomized or stepped speeds and records the resulting trajectories.

### klask_interfaces

Custom ROS2 messages, services, and actions shared between all nodes (e.g. `State`, `StampedPolygon`, `CalibrateEncoders`, `HomePeg`, `HomeAndCalibrate`).

## Hardware Components

This repository includes:

- **CAD Files** (`hardware/cad/`) SolidWorks assemblies and parts for mechanical structure
  - Gantry frame and mounting brackets
  - Motor assemblies and belt attachments
  - Camera mount and board frame
  - Custom game board adapter components

- **Third-Party Parts** References to commercial components (bearings, motors, profiles)

See the [Hardware Assembly tutorial](https://IDSCETHZurich.github.io/klask_hardware/tutorials/01-hardware-assembly/) for complete build instructions.

## Roadmap

We track work in GitHub issues and milestones.

## Contributing

We welcome contributions! Whether you're fixing bugs, adding features, or improving documentation:

1. **Fork** the repository and create a feature branch
2. **Develop** following our [code guidelines](https://IDSCETHZurich.github.io/klask_hardware/contribution/code_guideline/)
3. **Test** your changes thoroughly
4. **Submit a PR** with a clear description and context

See our [Contributing Guide](https://IDSCETHZurich.github.io/klask_hardware/contribution/contributing/) for detailed information.

## Maintainers

This project is mostly maintained by:

- [Aswin](https://github.com/akrv) - Lead Researcher
- [Tobias](https://github.com/MeierTobias) - Student

## Citing

If you use this work in an academic context, please cite the following publication:

- Aswin Karthik Ramachandran Venkatapathy, Jona Schulz, Maurus Derungs, Carlo Angelini, Tobias Meier, Raffaello D’Andrea, **"KlaskTron: An Open-Source Platform for Physical Adversarial Multi-Agent RL"**, 2026. ([PDF](https://openreview.net/pdf?id=UaLgID9r1i))

    ```bibtex
    @inproceedings{
      anonymous2026klasktron,
      title={KlaskTron: An Open-Source Platform for Physical Adversarial Multi-Agent {RL}},
      author={Anonymous},
      booktitle={Robotics: Science and Systems 2026},
      year={2026},
      url={https://openreview.net/forum?id=UaLgID9r1i}
    }
    ```

## License

- ROS2 code and software is licensed under [**AGPL-3.0**](LICENSE-AGPL-3.0)
- Hardware designs (CAD, mechanical) are under [**CERN-OHL-S v2**](LICENSE-CERN-OHL-S-2.0)
- Documentation and tutorials are under [**CC BY 4.0**](LICENSE-CC-BY-4.0)

### Third-party Components

- Dependencies and libraries retain their original licenses

## Acknowledgements

- ETH Zurich's **Institute for Dynamic Systems and Control** for project support
- All students and researchers involved in the KLASK project
- The ROS2 and open-source robotics community
- ODrive project for excellent motor control hardware and software
- All contributors and maintainers who make this project possible
