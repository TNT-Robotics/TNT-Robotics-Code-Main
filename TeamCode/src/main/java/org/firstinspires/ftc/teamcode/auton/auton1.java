package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.misc.config;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class auton1 extends LinearOpMode {

    config cfg = new config();
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory drivePreload = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(33.5)
                .forward(27)
                .build();
        Trajectory drivePreload2 = drive.trajectoryBuilder(new Pose2d())
                .back(27)
                .build();
        while (!opModeIsActive() && !isStopRequested()) {
            //loop detection here
            cfg.getVision().updateTags(cfg);

            if (cfg.getVision().idGetter() != 0) {
                // ID Found
                int cConeId = cfg.getVision().idGetter();
                cfg.setConeId(cfg.getVision().idGetter());
                telemetry.addData("Cone id", "%d", cConeId);
                telemetry.update();
            }
        }
        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(drivePreload);
        drive.followTrajectory(drivePreload2);
    }
}