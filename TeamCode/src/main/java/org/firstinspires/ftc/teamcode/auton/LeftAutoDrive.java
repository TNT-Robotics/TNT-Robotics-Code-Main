package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.misc.PID;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagDemo;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(group = "Autonomous")
public class LeftAutoDrive extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    PID slidesPID = new PID(.02, 0, .02, .008);

    double totalTimeOfCycles = 0;
    int numberOfCycles = 0;
    double averageTimePerCycle = 0;

    int coneStackValue = 5;
    double lastPing = 0;
    boolean closeClaw = true;
    Pose2d startPose = new Pose2d(-39, -60.5, Math.toRadians(90));
    double msTimeMarker = 0;
    boolean ranFirstTime = false;
    boolean startLift = false;
    double loopTime = 0.0;

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

        TrajectorySequence botTrajectory = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-58, -60, Math.toRadians(0)))
                //.splineToSplineHeading(new Pose2d(-58, -24, Math.toRadians(180)), Math.toRadians(0))
                .lineTo(new Vector2d(-58,-24))
                .forward(3)
                // Drop Cone
                .back(2)
                // Turn claw the other way
                .lineTo(new Vector2d(-57, -12))
                .back(2)
                // Grab cone
                // turn claw with cone to drop
                .lineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(270)))
                .back(3)
                // drop cone
                .forward(3)
                .lineToLinearHeading(new Pose2d(-57, -12, Math.toRadians(180)))
                .forward(3)
                // grab cone
                .back(3)
                .lineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(90)))
                .back(3)
                // drop cone
                .forward(3)

                // Vision 1

                /* .lineTo(new Vector2d(-59,-12))
                 .lineTo(new Vector2d(-59,-20))*/
                // Vision 2
/*
                                .lineTo(new Vector2d(-36,-12))
                                .lineTo(new Vector2d(-36,-20))*/

                // Vision 3
                .lineTo(new Vector2d(-12,-12))
                .lineTo(new Vector2d(-12,-20))

                .build();

        Trajectory drive5 = drive.trajectoryBuilder(startPose)
                .strafeRight(16)

                .addDisplacementMarker(5, () -> {
                    driveWithCone(clawServo, pivotServo);
                })
                .addDisplacementMarker(() -> {
                    updateClawServo(clawServo);
                    updateMotors(-1500, slide1Motor, slide2Motor);
                    telemetry.addData("Loop time", "ms", runtime.milliseconds() - loopTime);
                    loopTime = runtime.milliseconds();
                    telemetry.update();
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

        drive.followTrajectorySequence(botTrajectory);
        if (id == 440) { // Number 1
            //drive.followTrajectory(parkNumber1);
        } else if (id == 373) { // Number 2

        } else if (id == 182) { // Number 3
            //drive.followTrajectory(parkNumber3);
        }

        /*
         * if (movedDuringAlignmentBy > 0) {
         * drive.followTrajectory(restartAdjustBackward);
         * } else if (movedDuringAlignmentBy < 0) {
         * drive.followTrajectory(restartAdjustForward);
         * }
         */

    }

    void updateMotors(int targetState, DcMotor slide1Motor, DcMotor slide2Motor) {
        double currentArmPID = slidesPID.getOutputFromError(targetState, slide1Motor.getCurrentPosition());
        slide1Motor.setPower(currentArmPID);
        slide2Motor.setPower(currentArmPID);
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



    void telemetryUpdate(String message) {
        telemetry.addLine(message);
        telemetry.update();
    }
}