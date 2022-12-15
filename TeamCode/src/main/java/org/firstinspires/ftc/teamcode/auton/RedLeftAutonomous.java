package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

import java.util.concurrent.atomic.AtomicInteger;



/* TODO:
- Adjust the values for driving in autonomous (including park, the code is written just double check the values are accurate if not adjust)
- Write code for servos to turn claw/rotate/pivot so pickup/drive/drop off works properly
- Tune linear slide PID to not vibrate when to certain position (the PID value are in the "slidesPID" variable)
- Write code for linear slides to go to required heights
- Double check it is reliable at least 5-10x in a row
- Replicate for 4 classes (RedLeftAutonomous; RedRightAutonomous; BlueLeftAutonomous; BlueRightAutonomous), you will probably only need to mirror (multiply by -1) most of the global position values.

FIXME:
- Write here whatever you need to be fixed that's not working, I will try to do my best and see whats the issue but do not rely on this.
 */

/*

------------- General tips & tricks -------------
1. Upload to GitHub everyday for me to see what's going on as well as if something goes wrong we can version control save it.
2.

------------- Get into FTC Dashboard instructions -------------
1. Connect to the bot wifi with the computer
2. Open browser and connect to this url - http://192.168.43.1:8080/dash
3. Top right click the "Default" and switch to "Field"
4. To start a program use the dropdown on the left side under "OpMode". Click on "INIT" to initialize and then press "START" to begin. To turn off press the "STOP"!

 */
@Autonomous(group = "Red")
public class RedLeftAutonomous extends LinearOpMode {



    ElapsedTime runtime = new ElapsedTime();

    PID slidesPID = new PID(.02, 0, .02, .008);

    double lastPing = 0;
    boolean closeClaw = true;

    Pose2d startPose = new Pose2d(-39, -70, Math.toRadians(90));
    double loopTime = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        AprilTagDemo vision = new AprilTagDemo();
        OpenCvCamera camera;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        vision.initCamera(camera);
        // ASSIGN LINEAR SLIDE / ARM MOTOR
        DcMotor slide1Motor = hardwareMap.get(DcMotor.class, "s1");
        DcMotor slide2Motor = hardwareMap.get(DcMotor.class, "s2");

        slide1Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide1Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ASSIGN SERVOS
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
        Servo rotateServo = hardwareMap.get(Servo.class, "rotateServo");
        Servo pivotServo = hardwareMap.get(Servo.class, "pivotServo");


        pivotServo.setPosition(.5);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int id = 0;
        AtomicInteger targetPos = new AtomicInteger();

        slidesPID.getOutputFromError(0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence botTrajectory = drive.trajectorySequenceBuilder(startPose)

                .forward(4)

                .lineToLinearHeading(new Pose2d(-58, -60, Math.toRadians(0)))

                .lineTo(new Vector2d(-58,-24))

                /* example of raising slide
                .addDisplacementMarker(() -> {
                    targetPos.set(-1500);
                })*/

                /* example of moving pivot servo
                .addDisplacementMarker(() -> {
                    rotateServo.setPosition(.5);
                })*/

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


                .build();

        TrajectorySequence parkNumber1 = drive.trajectorySequenceBuilder(botTrajectory.end())
                .lineTo(new Vector2d(-59,-12))
                .lineTo(new Vector2d(-59,-20))
                .build();

        TrajectorySequence parkNumber2 = drive.trajectorySequenceBuilder(botTrajectory.end())
                .lineTo(new Vector2d(-36,-12))
                .lineTo(new Vector2d(-36,-20))
                .build();
        TrajectorySequence parkNumber3 = drive.trajectorySequenceBuilder(botTrajectory.end())
                .lineTo(new Vector2d(-12,-12))
                .lineTo(new Vector2d(-12,-20))
                .build();

        while (!opModeIsActive() && !isStopRequested()) {
            // loop detection here
            vision.updateTags();

            if (vision.idGetter() != 0) {
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

        drive.followTrajectorySequenceAsync(botTrajectory);
        if (id == 440) { // Number 1
            drive.followTrajectorySequence(parkNumber1);
        } else if (id == 373) { // Number 2
            drive.followTrajectorySequence(parkNumber2);

        } else if (id == 182) { // Number 3
            drive.followTrajectorySequence(parkNumber3);
        }

        while (opModeIsActive()) {
            drive.update();
            updateMotors(targetPos.get(), slide1Motor, slide2Motor);
            updateClawServo(clawServo);
        }
    }

    void updateMotors(int targetState, DcMotor slide1Motor, DcMotor slide2Motor) {
        double currentArmPID = slidesPID.getOutputFromError(targetState, slide1Motor.getCurrentPosition());
        slide1Motor.setPower(currentArmPID);
        slide2Motor.setPower(currentArmPID);
        telemetry.addData("PID Power", currentArmPID);
        telemetry.addData("Position (current vs target)", String.valueOf(slide1Motor.getCurrentPosition()), targetState);
        telemetry.update();

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