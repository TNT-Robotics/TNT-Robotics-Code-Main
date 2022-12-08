package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.misc.PID;
import org.firstinspires.ftc.teamcode.misc.config;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous(group = "Autonomous")
public class auton1 extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    config cfg = new config();
    private DistanceSensor poleDistanceSensor;

    PID slidesPID = new PID(.02, 0, .02, .008);

    double totalTimeOfCycles = 0;
    int numberOfCycles = 0;
    double averageTimePerCycle = 0;

    int coneStackValue = 5;
    double lastPing = 0;
    boolean closeClaw = true;
    double movedDuringAlignmentBy = 0.0;
    Pose2d startPose = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        // ASSIGN LINEAR SLIDE / ARM MOTOR
        DcMotor slide1Motor = hardwareMap.get(DcMotor.class, "s1");
        DcMotor slide2Motor = hardwareMap.get(DcMotor.class, "s2");

        // ASSIGN SERVOS
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
        Servo rotateServo = hardwareMap.get(Servo.class, "rotateServo");
        Servo pivotServo = hardwareMap.get(Servo.class, "pivotServo");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        poleDistanceSensor = hardwareMap.get(DistanceSensor.class, "chassis");

        slidesPID.getOutputFromError(0, 0);

        drive.setPoseEstimate(startPose);

        // PRELOAD POLE ADJUSTMENTS
        /*
         * Trajectory restartAdjustBackwardPRELOAD =
         * drive.trajectoryBuilder(adjustFowardPRELOAD.end())
         * .backward(abs(movedDuringAlignmentBy))
         * .build();
         * 
         * Trajectory adjustFowardPRELOAD =
         * drive.trajectoryBuilder(driveWithPreloadOption1P2.end())
         * .forward(.5)
         * .build();
         * 
         * // CONE ADJUSTMENTS
         * Trajectory restartAdjustBackwardCONE =
         * drive.trajectoryBuilder(adjustFowardCONE.end())
         * .backward(abs(movedDuringAlignmentBy))
         * .build();
         * 
         * Trajectory restartAdjustForwardCONE =
         * drive.trajectoryBuilder(adjustBackwardCONE.end())
         * .forward(abs(movedDuringAlignmentBy))
         * .build();
         * 
         * Trajectory adjustFowardCONE = drive.trajectoryBuilder(drivePreload2.end())
         * .forward(.5)
         * .build();
         * 
         * Trajectory adjustBackwardCONE = drive.trajectoryBuilder(drivePreload2.end())
         * .backward(.5)
         * .build();
         * 
         * // POLE ADJUSTMENTS
         * Trajectory restartAdjustBackwardPOLE =
         * drive.trajectoryBuilder(adjustFowardPOLE.end())
         * .backward(abs(movedDuringAlignmentBy))
         * .build();
         * 
         * Trajectory adjustFowardPOLE = drive.trajectoryBuilder(drivePreload2.end())
         * .forward(.5)
         * .build();
         */

        // PRELOAD
        Trajectory driveWithPreloadOption1P1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(33.5)
                .build();
        Trajectory driveWithPreloadOption1P2 = drive.trajectoryBuilder(driveWithPreloadOption1P1.end())
                .forward(27)
                .build();
        Trajectory drivePreload2 = drive.trajectoryBuilder(driveWithPreloadOption1P2.end())
                .back(27)
                .build();

        // CYCLE
        Trajectory driveToPole1 = drive.trajectoryBuilder(drivePreload2.end())
                .forward(27)
                .addDisplacementMarker(10, () -> {
                    placeOnPole(3);
                })
                .build();
        Trajectory driveToCone1 = drive.trajectoryBuilder(drivePreload2.end())
                .back(27)
                .build();

        while (!opModeIsActive() && !isStopRequested()) {
            // loop detection here
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
        clawServo.setPosition(0);
        closeClaw = false;

        if (isStopRequested())
            return;

        driveWithCone();
        telemetryUpdate("Starting to follow trajectory with preload");
        drive.followTrajectory(driveWithPreloadOption1P1);
        drive.followTrajectory(driveWithPreloadOption1P2);
        drive.followTrajectory(driveToPole1);
        prepareForGrab();
        /*
         * if (movedDuringAlignmentBy > 0) {
         * drive.followTrajectory(restartAdjustBackward);
         * } else if (movedDuringAlignmentBy < 0) {
         * drive.followTrajectory(restartAdjustForward);
         * }
         */
        drive.followTrajectory(drivePreload2);
        cycleRun();

    }

    // METHODS
    void placeOnPole(int level) {
        if (level == 0) {
            placeOnPoleHandler(-150);
            return;
        }
        if (level == 1) {
            placeOnPoleHandler(-2179);
            return;
        }
        if (level == 2) {
            placeOnPoleHandler(-3440);
            return;
        }
        if (level == 3) {
            placeOnPoleHandler(-4610);
            return;
        }
    }

    boolean closeEnough(int currentPos, int desiredPos, int deadZone) {

        int floorValue = desiredPos - deadZone;
        int ceilValue = desiredPos + deadZone;

        if (currentPos > floorValue && currentPos < ceilValue) {
            return true;
        }
        return false;
    }

    void updateMotors(int targetState) {
        double currentArmPID = slidesPID.getOutputFromError(targetState, slide1Motor.getCurrentPosition());
        slide1Motor.setPower(currentArmPID);
        slide2Motor.setPower(currentArmPID);
    }

    void placeOnPoleHandler(int targetPos) {

        while (closeEnough(slide1Motor.getCurrentPosition(), targetPos, 10) == false) {
            updateMotors(targetPos);
            updateClawServo();
        }

        clawServo.setPosition(1);
        closeClaw = false;

        pivotServo.setPosition(.15);
        rotateServo.setPosition(0);

        double currentMsTime = runtime.milliseconds();
        while (runtime.milliseconds() < currentMsTime + 750) {
            updateMotors(targetPos);
            updateClawServo();
        }
        pivotServo.setPosition(1);
        currentMsTime = runtime.milliseconds();
        while (runtime.milliseconds() < currentMsTime + 750) {
            updateMotors(targetPos);
            updateClawServo();
        }
    }

    void driveWithCone() {
        clawServo.setPosition(1);
        updateClawServo();
        closeClaw = true;
        pivotServo.setPosition(.5);
    }

    void updateClawServo() {
        if (runtime.milliseconds() >= lastPing + 3000) {
            lastPing = runtime.milliseconds();
            if (closeClaw) {
                if (clawServo.getPosition() == 1) {
                    clawServo.setPosition(.95);
                } else {
                    clawServo.setPosition(1);
                }
            }
        }
    }

    void prepareForGrab() {
        pivotServo.setPosition(0.05);
        rotateServo.setPosition(1);
        double currentMsTime = runtime.milliseconds();
        boolean servoTurned = false;
        while (closeEnough(slide1Motor.getCurrentPosition(), 0, 20) == false) {
            updateMotors(0);
            updateClawServo();
            if (runtime.milliseconds() < currentMsTime + 750) {
                servoTurned = true;
                closeClaw = false;
                clawServo.setPosition(0);
            }
        }
        if (servoTurned == false) {
            closeClaw = false;
            clawServo.setPosition(0);
        }
    }

    void grabCone() {
        closeClaw = true;
        clawServo.setPosition(1);
    }

    void checkPolePosition(double desiredDistance) {
        updateClawServo();
        telemetry.addLine("Pole alignment");
        telemetry.addData("Distance from pole in mm (actual, target)", "%4.2f, %4.2f",
                poleDistanceSensor.getDistance(DistanceUnit.MM), desiredDistance);

        closeClaw = false;
        clawServo.setPosition(0);
    }

    void cycleRun() {

    }

    void telemetryUpdate(string message) {
        telemetry.addLine(message);
        telemetry.update();
    }

    void telemetryUpdate(string message, double value) {
        telemetry.addData(message, "%4.2f", value);
        telemetry.update();
    }

    void telemetryUpdate(string message, int value) {
        telemetry.addData(message, "%d", value);
        telemetry.update();
    }

    void telemetryUpdate(string message, boolean value) {
        telemetry.addData(message, "%b", value);
        telemetry.update();
    }
}