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

        InchGridPathfinder.Cell start = new InchGridPathfinder.Cell(12, 12);
        InchGridPathfinder.Cell goal = new InchGridPathfinder.Cell(130, 130);

        telemetry.addLine("Pathfinder initialized");
        telemetry.update();

        waitForStart();

        List<InchGridPathfinder.Cell> path = pf.findPath(start, goal);
        telemetry.addData("grid", "%dx%d", pf.getWidthCells(), pf.getHeightCells());
        telemetry.addData("found", !path.isEmpty());
        telemetry.addData("len", path.size());
        if (!path.isEmpty()) {
            telemetry.addData("start", path.get(0).toString());
            telemetry.addData("end", path.get(path.size() - 1).toString());
        }
        telemetry.update();

        while (opModeIsActive()) {
            sleep(50);
        }
    }
}
