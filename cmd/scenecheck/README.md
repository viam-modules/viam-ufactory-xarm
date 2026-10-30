# scenecheck

Checks that a machine's world scene keeps an xArm's gripper out of the table. It asks the motion service to plan, never to execute, so the arm does not move.

```
go run ./cmd/scenecheck -addr localhost:18080          # local replica, insecure
go run ./cmd/scenecheck -cred path/to/machine.cloudcred # live machine
```

The credentials file is the machine's "Cloud credentials" export: a `Remote address` line followed by the address, and the cloud JSON holding `api_key`.

## Checks

1. **Frames.** World poses of the plate origin, the bolt circle, the arm origin, and the gripper. Reports the table top and whether the plate's bottom rests on it.
2. **Start state.** Fails when any arm or gripper geometry already touches an obstacle. The planner ignores every pair in collision at the start of a move for the rest of that move (`computeInitialCollisionsToIgnore` in RDK `motionplan/constraint_checker.go`), so a gripper starting on the table disables the table for that move.
3. **Sweep.** Plans the gripper, pointing down, to a column of heights relative to the table top at `-x`, `-y`. A goal below the table top that plans is a failure. Goals above it may be refused when the gripper body reaches the table.
4. **Paths.** Re-checks each planned trajectory, interpolated `-substeps` times per segment, for arm or gripper geometry hitting an obstacle, and reports the closest approach to the table.

Exit status: 0 pass, 1 a check failed, 2 the tool could not run.

## Replica

`replica/run.sh` runs `viam-server` on `localhost:18080` with a stand-in workcell built from this checkout's module:

- `xarm6-mounting-plate` under `world` at (0, −5, −8), so the bolt circle is the world origin.
- A fake xArm6 parented to the plate.
- `gripper` and `gripper-case`: the G1 gripper's default collision boxes (claws 40 × 170 × 105 centered at z −2.5; case 50 × 100 × 100 centered at z −50) at the arm flange + 105 mm, rotated 180°.
- A 3000 × 3000 × 200 table centered at z −108, so its top meets the plate's bottom; back and left walls; ceiling.

Arguments set the fake arm's six joints in degrees. The default, `0 -30 -30 0 60 0`, holds the gripper pointing down at (315, 0, 234). At all-zero joints the gripper sits 7 mm above the table with its claws in it, which the start-state check catches.

Expected result from the default pose: PASS, with the sweep planning down to +50 mm and refusing +45 mm and below. The claw box reaches 50 mm past the gripper frame, so +50 mm leaves 0 mm clearance; the planner accepts contact.

## Scene notes

- An `erh:vmodutils:obstacle` returns an error from `Kinematics`, so RDK builds its frame from `Geometries()`. A component without a kinematic model carries its geometry on `<name>_origin`; one with a model, such as the plate or the real gripper, carries it on `<name>`. `geomsOf` reads both.
- The xArm6 kinematics put a radius-1 sphere (`base`) at the arm origin and the `base_top` capsule's lower end on the mounting face, so both touch the plate. Neither moves relative to the plate, and `-fixed-links` reports them as expected.
- The real gripper's geometry comes from `viam:ufactory:gripper`: the G1 or G2 box set (both reach 50 mm past the gripper frame), or URDF meshes with `use_urdfs`.

## Plan for the live machine

The machine's arm comes from a fragment with its frame parented to `world`. Adding the plate takes two edits to the machine config:

1. A `viam:ufactory:xarm6_mounting_plate` component under `world` at (0, −5, −8). The arm's base stays at the world origin, so obstacles and saved world poses keep their meaning.
2. A fragment mod that reparents the arm: `{"$set": {"components.arm.frame.parent": "<plate name>"}}`.

Before running scenecheck on the machine:

- The plate model ships only once its PR is merged and released; until then the machine runs this branch's module.
- Measure the table top relative to the arm base. The table fragment placed it 23 mm below the arm base before the plate existed; with the plate resting on the table it should be 8 mm below, a table center of −108 for a 200 mm table.
- Confirm the plate's orientation: the model puts its 185 mm side on x and the bolt circle 5 mm toward +y of the plate center.
- Start the arm well clear of the table.

Then run scenecheck from the machine's usual working pose and from a pose near the table edge. Repeat the sweep at points under the walls' reach (`-x`, `-y`) and with `use_urdfs` on the gripper.
