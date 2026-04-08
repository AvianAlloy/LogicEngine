# FTC Robot Controller Pathfinder Base

This folder is a TeamCode-oriented base for FTC Robot Controller projects, containing a lightweight Java A* pathfinder tuned for FTC field usage.

## Included

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pathing/InchGridPathfinder.java`
  - 12ft x 12ft default grid support at 1-inch resolution (144x144 cells)
  - compact obstacle storage (`byte[]`)
  - A* pathfinding with optional diagonal moves and corner-cut prevention
  - lightweight Road Runner-style pose API: `findPath(Pose startPose, Pose targetPose)`
  - internal waypoint reduction so returned paths are easier/lighter to follow
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/PathfinderSmokeOpMode.java`
  - disabled smoke-test OpMode demonstrating integration with Robot Controller telemetry

## How to use with FTC SDK

1. Use this folder as the base for your `TeamCode` sources within an FTC SDK checkout.
2. Keep package names under `org.firstinspires.ftc.teamcode`.
3. Enable `PathfinderSmokeOpMode` once integrated and deployed to robot controller.
