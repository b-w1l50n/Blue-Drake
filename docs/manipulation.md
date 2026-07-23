# ROV grippers and manipulation

Blue Drake's first manipulation primitive is a generic parallel-jaw gripper
attached to a marine vehicle. It exists to exercise Drake multibody, joint,
actuation, contact, control, and visualization workflows. It is not a model of
a named commercial gripper.

## Multibody model

Passing a `ParallelJawGripperConfig` in the `grippers` mapping of
`build_marine_fleet_diagram()` adds a separate Drake model instance containing:

- a palm welded to the front of the selected vehicle body;
- two rigid jaws on independently actuated prismatic joints;
- symmetric opening limits and declared default positions;
- viscous joint damping and actuator effort metadata; and
- box illustration and Coulomb-friction collision geometry.

Keeping the gripper in a separate model instance preserves the existing
13-element vehicle state port. The diagram adds:

- `<id>_gripper_state`: left position, right position, left velocity, and right
  velocity; and
- `<id>_gripper_actuation_N`: left and right prismatic-joint forces.

Drake's `MultibodyPlant` enforces joint limits and calculates the reaction of
the articulated mass on the vehicle. The vehicle and gripper therefore do not
move as independent visual objects.

## Joint controller

`ParallelJawGripperController` consumes gripper state and a desired total
opening. The pure `parallel_jaw_actuation()` calculation clips the requested
opening to joint limits, creates symmetric targets, applies joint-space PD
feedback, and clips each actuator force:

```text
q_desired = [opening / 2, -opening / 2]
force = clip(Kp (q_desired - q) - Kd qdot, +/- maximum_force)
```

The tested [`rov_gripper.py`](../examples/rov_gripper.py) example connects a
Drake piecewise-polynomial opening trajectory through this controller to the
plant. It closes the gripper from 0.16 m to its 0.04 m joint limit without any
mission or grasp-planning logic.

## Fidelity boundary

Mass, dimensions, gains, friction, and limits are assumed generic values.
Current marine forces are applied to the vehicle base, not separately to the
palm and moving jaws. The model omits compliant pads, backlash, motor and
electrical dynamics, cable or hydraulic transmission, wrist force sensing,
object perception, grasp planning, vendor protocols, and hardware calibration.

Before using a physical gripper's name, replace the assumed parameters with
traceable data and validate motion, effort, contact, and payload behavior.
