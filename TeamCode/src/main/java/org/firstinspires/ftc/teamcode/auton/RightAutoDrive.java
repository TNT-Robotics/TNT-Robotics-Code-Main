package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.misc.PID;
import org.firstinspires.ftc.teamcode.misc.config;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.AprilTagDemo;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(group = "Autonomous")
public class RightAutoDrive extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    config cfg = new config();

    PID slidesPID = new PID(.02, 0, .02, .008);

    double totalTimeOfCycles = 0;
    int numberOfCycles = 0;
    double averageTimePerCycle = 0;

    int coneStackValue = 5;
    double lastPing = 0;
    boolean closeClaw = true;
    Pose2d startPose = new Pose2d(65.00, -39.00, 180);
    double msTimeMarker = 0;
    boolean ranFirstTime = false;
    boolean startLift = false;

    @Override
    public void runOpMode() throws InterruptedException {

        AprilTagDemo vision = new AprilTagDemo();
        OpenCvCamera camera;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        vision.initCamera(camera);
        // ASSIGN LINEAR SLIDE / ARM MOTOR
        DcMotor slide1Motor = hardwareMap.get(DcMotor.class, "s1");
        DcMotor slide2Motor = hardwareMap.get(DcMotor.class, "s2");

        // ASSIGN SERVOS
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
        Servo rotateServo = hardwareMap.get(Servo.class, "rotateServo");
        Servo pivotServo = hardwareMap.get(Servo.class, "pivotServo");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int id = 0;

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
        Trajectory getFromWall = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(2)
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                })
                .build();

        Trajectory partial1 = drive.trajectoryBuilder(getFromWall.end())
                .strafeRight(11)
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                })
                .build();
        Trajectory partial2 = drive.trajectoryBuilder(partial1.end())
                .forward(6.8)
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                })
                .build();
        Trajectory partial3 = drive.trajectoryBuilder(partial2.end())
                .back(6.8)
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                })
                .build();

        Trajectory drive1 = drive.trajectoryBuilder(partial3.end())
                .strafeRight(13.2)
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                })
                .build();

        Trajectory drive2 = drive.trajectoryBuilder(drive1.end())
                .forward(39.5)
                .addDisplacementMarker(2, () -> {
                    startLift = true;
                })
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                    /*if (startLift) {
                        updateMotors(-1000, slide1Motor, slide2Motor);
                        if (closeEnough(slide1Motor.getCurrentPosition(), -1000, 50)) {
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
                    }*/
                })
                .build();

        Trajectory drive3 = drive.trajectoryBuilder(drive2.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .back(6)
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                })
                .build();

        Trajectory drive4 = drive.trajectoryBuilder(drive3.end())
                .forward(4)
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                    /*slide1Motor.setPower(0);
                    slide2Motor.setPower(0);
                    if (closeEnough(slide1Motor.getCurrentPosition(), 0, 40)) {
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
                    }*/
                })
                .build();

        Trajectory drive5 = drive.trajectoryBuilder(drive4.end())
                .strafeLeft(16)

                .addDisplacementMarker(5, () -> {
                    driveWithCone(clawServo, pivotServo);
                })
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                })
                .build();

        Trajectory idk1 = drive.trajectoryBuilder(drive5.end())
                .back(8)

                .addDisplacementMarker(5, () -> {
                    pivotServo.setPosition(0);
                    clawServo.setPosition(0);
                    closeClaw = false;
                })
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                })
                .build();
        // CYCLE START
        Trajectory drive6 = drive.trajectoryBuilder(idk1.end())
                .back(0.1)
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                })
                .build();

        Trajectory drive7 = drive.trajectoryBuilder(drive6.end())
                .back(15.7)
                .addDisplacementMarker(2, () -> {
                    startLift = true;
                })
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                    /*if (startLift) {
                        updateMotors(-3000, slide1Motor, slide2Motor);
                        if (closeEnough(slide1Motor.getCurrentPosition(), -3000, 50)) {
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
                    }*/
                })
                .build();

        Trajectory drive8 = drive.trajectoryBuilder(drive7.end())
                .strafeRight(13)
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                    if (runtime.milliseconds() > msTimeMarker + 750) {
                        pivotServo.setPosition(1);
                        msTimeMarker = 0;
                    }
                })

                .build();

        Trajectory drive9 = drive.trajectoryBuilder(drive8.end())
                .back(4)
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                })
                .build();

        Trajectory drive10 = drive.trajectoryBuilder(drive9.end())
                .forward(4)
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                    /*slide1Motor.setPower(0);
                    slide2Motor.setPower(0);
                    if (closeEnough(slide1Motor.getCurrentPosition(), 0, 50)) {
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
                    }*/
                })
                .build();

        Trajectory drive11 = drive.trajectoryBuilder(drive10.end())
                .strafeLeft(12)
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                    if (runtime.milliseconds() > msTimeMarker + 750) {
                        closeClaw = false;
                        clawServo.setPosition(0);
                    }
                })
                .build();

        // CYCLE END

        Trajectory parkNumber1 = drive.trajectoryBuilder(drive11.end())
                .back(25)
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                })
                .build();
        Trajectory parkNumber3 = drive.trajectoryBuilder(drive11.end())
                .forward(25)
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                })
                .build();

        while (!opModeIsActive() && !isStopRequested()) {
            // loop detection here
            vision.updateTags();

            if (vision.idGetter() != 0) {;
                id = vision.idGetter();
                telemetry.addData("Cone id", "%d", id);
                telemetry.update();
            }
        }
        waitForStart();
        clawServo.setPosition(0);
        closeClaw = false;

        driveWithCone(clawServo, pivotServo);
        telemetryUpdate("Starting to follow trajectory with preload");
        drive.followTrajectory(getFromWall);
        drive.followTrajectory(partial1);
        pivotServo.setPosition(0.05);
        drive.followTrajectory(partial2);
        clawServo.setPosition(0);
        closeClaw = false;
        drive.followTrajectory(partial3);
        drive.followTrajectory(drive1);
        drive.followTrajectory(drive2);
 /*
        while (!closeEnough(slide1Motor.getCurrentPosition(), -1000, 50)) {
            updateClawServo(clawServo);
            updateMotors(-1000, slide1Motor, slide2Motor);
            if (runtime.milliseconds() > msTimeMarker + 750) {
                pivotServo.setPosition(1);
            }
        }*/
        ranFirstTime = false;
        msTimeMarker = 0;
        startLift = false;
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(drive3);
        //dropCone(clawServo);
        drive.followTrajectory(drive4);
        drive.followTrajectory(drive5);
        clawServo.setPosition(0);
        closeClaw = false;
        msTimeMarker = 0;
        startLift = false;
        ranFirstTime = false;
        drive.followTrajectory(idk1);
        drive.followTrajectory(drive6);
        //grabCone(clawServo);
        //driveWithCone(clawServo, pivotServo);
        drive.followTrajectory(drive7);
        drive.followTrajectory(drive8);
        msTimeMarker = 0;
        startLift = false;
        ranFirstTime = false;
        drive.followTrajectory(drive9);
        //dropCone(clawServo);
        drive.followTrajectory(drive10);
        drive.followTrajectory(drive11);
        msTimeMarker = 0;
        startLift = false;
        ranFirstTime = false;
        if (id == 440) { // Number 1
            drive.followTrajectory(parkNumber1);
        } else if (id == 373) { // Number 2

        } else if (id == 182) { // Number 3
            drive.followTrajectory(parkNumber3);
        }

        /*
         * if (movedDuringAlignmentBy > 0) {
         * drive.followTrajectory(restartAdjustBackward);
         * } else if (movedDuringAlignmentBy < 0) {
         * drive.followTrajectory(restartAdjustForward);
         * }
         */

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