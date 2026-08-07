# Verification Evidence

This directory preserves the complete development and verification history of the SO-101 Visual-Tactile Grasp project.

## Directory Structure

```
data/verification/
├── README.md                  ← You are here
├── final/                     ← Final acceptance evidence
├── archive/                   ← Archived development-stage evidence
└── ...                        ← Legacy reports kept in place for reference
```

## FINAL Acceptance Evidence

See `final/` for the definitive verification reports corresponding to the **final hardware acceptance** (MVP-4E):

- Final simplification and workflow freeze
- Tactile grasp-and-lift verification (51/53 automated checks)
- X-axis +20 mm forward grasp offset
- Close-until-tactile-contact gripper behavior
- Direct COM8 FlexiTac serial integration

These are the reports that directly validate the current production behavior.

## ARCHIVED Development Evidence

See `archive/` for the complete development history:

- **Stage -1**: Windows environment audit, ROS2 Lyrical setup
- **Stage 0**: MVP-0 skeleton launch, git baseline, ROS2 smoke tests
- **Stage 1**: Kinematics verification (FK validation)
- **Stage 2**: Visual perception, workspace calibration, timed trajectory, command gate, real joint bridge (stages 2A through 2D)
- **Stage 3**: Hardware executor, TCP bridge, live read-only, single TCP architecture, stop/confirm fixes (MVP-3A through MVP-3D)
- **Stage 4**: Pregrasp planner, IK multiseed, speed tuning, segmented descent, occlusion handoff, integrated visual grasp (MVP-4A through MVP-4D)
- **MVP-4E development**: Bridge launcher debugging, WinError classification, command file syntax, process spawn, log severity, process readiness

These reports document the full engineering journey from initial Windows/ROS2 setup through the complete visual-tactile grasp pipeline. They are retained as engineering history but do not represent the final running workflow.

## Notes for Users

- **New users** should focus on the `final/` directory and the final acceptance documentation in `docs/`.
- **The multi-terminal manual workflow** (Terminal 0-3) is the official running procedure. The complex one-launch orchestrator documented in some archive reports is deprecated.
- All verification reports were generated from real hardware runs or automated offline checks. No results are fabricated.
- Some reports reference specific hardware (COM4, COM8, USB serial numbers, calibration paths) from the developer's machine. These are configuration examples, not universal requirements.
