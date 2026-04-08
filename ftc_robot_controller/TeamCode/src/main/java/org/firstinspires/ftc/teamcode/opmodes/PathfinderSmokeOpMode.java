package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pathing.InchGridPathfinder;

import java.util.List;

/**
 * Minimal FTC Robot Controller integration example.
 */
@Disabled
@TeleOp(name = "Pathfinder Smoke Test", group = "debug")
public final class PathfinderSmokeOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        InchGridPathfinder pf = new InchGridPathfinder(144, 144, 1, true);

        // Add a 1-inch wall with a gap.
        pf.setBlockedRect(70, 0, 70, 130, true);
        pf.setBlockedRect(70, 60, 70, 80, false);

        InchGridPathfinder.Pose start = new InchGridPathfinder.Pose(12f, 12f, 0f);
        InchGridPathfinder.Pose goal = new InchGridPathfinder.Pose(130f, 130f, 0f);

        telemetry.addLine("Pathfinder initialized");
        telemetry.update();

        waitForStart();

        List<InchGridPathfinder.Pose> path = pf.findPath(start, goal);
        telemetry.addData("grid", "%dx%d", pf.getWidthCells(), pf.getHeightCells());
        telemetry.addData("found", !path.isEmpty());
        telemetry.addData("len", path.size());
        if (!path.isEmpty()) {
            telemetry.addData("startPose", path.get(0).toString());
            telemetry.addData("endPose", path.get(path.size() - 1).toString());
        }
        telemetry.update();

        while (opModeIsActive()) {
            sleep(50);
        }
    }
}
