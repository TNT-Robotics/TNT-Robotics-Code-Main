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
    double msTimeMarker = 0;
    boolean ranFirstTime = false;
    boolean startLift = false;

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
        Trajectory getFromWall = drive.trajectoryBuilder(new Pose2d())
                .forward(.2)
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                })
                .build();

        Trajectory driveWithPreloadOption1P1 = drive.trajectoryBuilder(getFromWall.end())
                .strafeRight(33.5)
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                })
                .build();
        Trajectory driveWithPreloadOption1P2 = drive.trajectoryBuilder(driveWithPreloadOption1P1.end())
                .forward(27)

                .addDisplacementMarker(10, () -> {
                    startLift = true;
                })
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                    if (startLift) {
                        updateMotors(-4610, slide1Motor, slide2Motor);
                        if (closeEnough(slide1Motor.getCurrentPosition(), -4610, 10)) {
                            if (!ranFirstTime) {
                                pivotServo.setPosition(.15);
                                rotateServo.setPosition(0);
                            }
                            if (msTimeMarker == 0) {
                                msTimeMarker = runtime.milliseconds();
                            }
                            if (runtime.milliseconds() > msTimeMarker + 750) {
                                pivotServo.setPosition(1);
                            }
                        }
                    }
                })
                .build();
        Trajectory drivePreload2 = drive.trajectoryBuilder(driveWithPreloadOption1P2.end())
                .back(27)
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                    updateMotors(0, slide1Motor, slide2Motor);
                    if (closeEnough(slide1Motor.getCurrentPosition(), 0, 10)) {
                        if (!ranFirstTime) {
                            pivotServo.setPosition(0.05);
                            rotateServo.setPosition(1);
                        }
                        if (msTimeMarker == 0) {
                            msTimeMarker = runtime.milliseconds();
                        }
                        if (runtime.milliseconds() > msTimeMarker + 750) {
                            closeClaw = false;
                            clawServo.setPosition(0);
                        }
                    }
                })
                .build();

        // CYCLE
        Trajectory driveToPole1 = drive.trajectoryBuilder(drivePreload2.end())
                .forward(27)
                .addDisplacementMarker(10, () -> {
                    placeOnPole(3, slide1Motor, slide2Motor, clawServo, pivotServo, rotateServo);
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

        driveWithCone(clawServo, pivotServo);
        telemetryUpdate("Starting to follow trajectory with preload");
        drive.followTrajectory(getFromWall);
        drive.followTrajectory(driveWithPreloadOption1P1);
        drive.followTrajectory(driveWithPreloadOption1P2);

        msTimeMarker = 0;
        startLift = false;
        ranFirstTime = false;

        dropCone(clawServo);
        drive.followTrajectory(drivePreload2);
        /*
         * if (movedDuringAlignmentBy > 0) {
         * drive.followTrajectory(restartAdjustBackward);
         * } else if (movedDuringAlignmentBy < 0) {
         * drive.followTrajectory(restartAdjustForward);
         * }
         */
        grabCone(clawServo);
        drive.followTrajectory(driveToPole1);
        cycleRun();

    }

    // METHODS
    void placeOnPole(int level, DcMotor slide1Motor, DcMotor slide2Motor, Servo clawServo, Servo pivotServo, Servo rotateServo) {
        if (level == 0) {
            placeOnPoleHandler(-150, slide1Motor, slide2Motor, clawServo, pivotServo, rotateServo);
            return;
        }
        if (level == 1) {
            placeOnPoleHandler(-2179, slide1Motor, slide2Motor, clawServo, pivotServo, rotateServo);
            return;
        }
        if (level == 2) {
            placeOnPoleHandler(-3440, slide1Motor, slide2Motor, clawServo, pivotServo, rotateServo);
            return;
        }
        if (level == 3) {
            placeOnPoleHandler(-4610, slide1Motor, slide2Motor, clawServo, pivotServo, rotateServo);
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

    void updateMotors(int targetState, DcMotor slide1Motor, DcMotor slide2Motor) {
        double currentArmPID = slidesPID.getOutputFromError(targetState, slide1Motor.getCurrentPosition());
        slide1Motor.setPower(currentArmPID);
        slide2Motor.setPower(currentArmPID);
    }

    void placeOnPoleHandler(int targetPos, DcMotor slide1Motor, DcMotor slide2Motor, Servo clawServo, Servo pivotServo, Servo rotateServo) {

        while (closeEnough(slide1Motor.getCurrentPosition(), targetPos, 10) == false) {
            updateMotors(targetPos, slide1Motor, slide2Motor);
            updateClawServo(clawServo);
        }

        clawServo.setPosition(1);
        closeClaw = false;

        pivotServo.setPosition(.15);
        rotateServo.setPosition(0);

        double currentMsTime = runtime.milliseconds();
        while (runtime.milliseconds() < currentMsTime + 750) {
            updateMotors(targetPos, slide1Motor, slide2Motor);
            updateClawServo(clawServo);
        }
        pivotServo.setPosition(1);
        currentMsTime = runtime.milliseconds();
        while (runtime.milliseconds() < currentMsTime + 750) {
            updateMotors(targetPos, slide1Motor, slide2Motor);
            updateClawServo(clawServo);
        }
    }

    void driveWithCone(Servo clawServo, Servo pivotServo) {
        clawServo.setPosition(1);
        updateClawServo(clawServo);
        closeClaw = true;
        pivotServo.setPosition(.5);
    }

    void updateClawServo(Servo clawServo) {
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

    void prepareForGrab(DcMotor slide1Motor, DcMotor slide2Motor, Servo clawServo, Servo pivotServo, Servo rotateServo) {
        pivotServo.setPosition(0.05);
        rotateServo.setPosition(1);
        double currentMsTime = runtime.milliseconds();
        boolean servoTurned = false;
        while (closeEnough(slide1Motor.getCurrentPosition(), 0, 20) == false) {
            updateMotors(0, slide1Motor, slide2Motor);
            updateClawServo(clawServo);
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

    void grabCone(Servo clawServo) {
        closeClaw = true;
        clawServo.setPosition(1);
    }
    void dropCone(Servo clawServo) {
        closeClaw = false;
        clawServo.setPosition(0);
    }

    void checkPolePosition(double desiredDistance, Servo clawServo) {
        updateClawServo(clawServo);
        telemetry.addLine("Pole alignment");
        telemetry.addData("Distance from pole in mm (actual, target)", "%4.2f, %4.2f",
                poleDistanceSensor.getDistance(DistanceUnit.MM), desiredDistance);

        closeClaw = false;
        clawServo.setPosition(0);
    }

    void cycleRun() {

    }

    void telemetryUpdate(String message) {
        telemetry.addLine(message);
        telemetry.update();
    }

    void telemetryUpdate(String message, double value) {
        telemetry.addData(message, "%4.2f", value);
        telemetry.update();
    }

    void telemetryUpdate(String message, int value) {
        telemetry.addData(message, "%d", value);
        telemetry.update();
    }

    void telemetryUpdate(String message, boolean value) {
        telemetry.addData(message, "%b", value);
        telemetry.update();
    }
}