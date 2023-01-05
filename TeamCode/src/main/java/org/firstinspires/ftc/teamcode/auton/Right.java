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
import org.firstinspires.ftc.teamcode.vision.AprilTags;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.concurrent.atomic.AtomicInteger;

/**
 * This class represents an autonomous program for a robot that starts on the right side of the field
 * follows a path. It uses the Road Runner library for path following and
 * a PID (proportional-integral-derivative) controller for controlling the linear slide motors. The program
 * also utilizes a webcam for vision processing with the OpenCV library and the EasyOpenCV library.
 */

// WORK ON ONLY AFTER FINISHING "Left"
/* TODO:
- Adjust starting position - startPose variable
- Copy values from "Left"
- Adjust according to the needs (you will probably just need to flip some from negative to positive or positive to negative, and change rotation by 180)
- Don't forget to test parking
- After you are done don't forget blue side autonomous, again its probably just going to be flipping from negative to positive
FIXME:
- Write here whatever you need to be fixed that's not working, I will try to do my best and see whats the issue but do not rely on this.
 */

/*

------------- General tips & tricks -------------
1. Upload to GitHub everyday for me to see what's going on as well as if something goes wrong we can version control save it.
2. Use MeepMeep for coordinate and rotation system. Here is an attached picture https://ctrlv.link/yG1o of the grid, but if you need something more precise use MeepMeep (follow steps here https://github.com/NoahBres/MeepMeep)


------------- Get into FTC Dashboard instructions -------------
1. Connect to the bot wifi with the computer
2. Open browser and connect to this url - http://192.168.43.1:8080/dash
3. Top right click the "Default" and switch to "Field"
4. To start a program use the dropdown on the left side under "OpMode". Click on "INIT" to initialize and then press "START" to begin. To turn off press the "STOP"!

 */
@Autonomous(group = "Autonomous")
public class Right extends LinearOpMode {

    // Create a timer object to track elapsed time
    ElapsedTime runtime = new ElapsedTime();

    // Create a PID controller for the slide motors
    PID slidesPID = new PID(.02, 0, .02, .008);

    // Initialize the last ping time to 0 and the closeClaw boolean to true
    double lastPing = 0;
    boolean closeClaw = true;

    // Create a pose object representing the starting pose of the robot
    Pose2d startPose = new Pose2d(-39, -70, Math.toRadians(90));

    // Initialize the loop time to 0
    double loopTime = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize telemetry for both the driver station and the dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Create a vision object
        AprilTags vision = new AprilTags();

        // Set up the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Initialize the camera for the vision object
        vision.initCamera(camera);

        // Get the linear slide / arm motors from the hardware map
        DcMotor slide1Motor = hardwareMap.get(DcMotor.class, "s1");
        DcMotor slide2Motor = hardwareMap.get(DcMotor.class, "s2");

        // Set the mode of the motors to STOP_AND_RESET_ENCODER and then RUN_WITHOUT_ENCODER
        slide1Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Get the servos from the hardware map
        Servo clawServo = hardwareMap.get(Servo.class, "clawServo");
        Servo rotateServo = hardwareMap.get(Servo.class, "rotateServo");
        Servo pivotServo = hardwareMap.get(Servo.class, "pivotServo");

        // Set the pivot servo to position .5
        pivotServo.setPosition(.5);
        clawServo.setPosition(1);
        closeClaw = true;

        // Create a drive object
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Initialize the cone ID to 0
        int id = 0;

        // Create an AtomicInteger to hold the target position for the motors
        AtomicInteger targetPos = new AtomicInteger();

        // Initialize the PID controller
        slidesPID.getOutputFromError(0, 0);

        // Set the starting pose of the drive object
        drive.setPoseEstimate(startPose);

        /* example of raising slide
                .addDisplacementMarker(() -> {
                    targetPos.set(-1500);
                })*/
        /* example of moving rotate servo
                .addDisplacementMarker(() -> {
                    rotateServo.setPosition(.5);
                })*/


        TrajectorySequence parkNumber1 = drive.trajectorySequenceBuilder(startPose)
                .forward(4)

                .lineToLinearHeading(new Pose2d(-58, -60, Math.toRadians(0)))

                .addDisplacementMarker(() -> {
                    targetPos.set(-1350);
                    pivotServo.setPosition(0);
                })
                .lineTo(new Vector2d(-58,-20))
                .forward(1)

                // Drop Cone
                .addDisplacementMarker(() -> {
                    clawServo.setPosition(0);
                    closeClaw = false;
                })
                .back(1)
                .addDisplacementMarker(() -> {
                    targetPos.set(-350);
                    pivotServo.setPosition(1);
                })

                // Turn claw the other way
                .lineTo(new Vector2d(-57, -5.5))
                .back(5)
                // Grab cone
                .addDisplacementMarker(() -> {
                    clawServo.setPosition(1);
                    closeClaw = true;
                })
                // turn claw with cone to drop
                .forward(1)

                .addDisplacementMarker(() -> {
                    targetPos.set(-1000);
                })
                .forward(6)
                .addDisplacementMarker(() -> {
                    targetPos.set(-4000);
                })
                .lineToLinearHeading(new Pose2d(-20, -5.5, Math.toRadians(270)))
                .back(5)
                // drop cone
                .addDisplacementMarker(() -> {
                    targetPos.set(-300);
                    clawServo.setPosition(0);
                    closeClaw = false;

                })
                .forward(4)
                .lineToLinearHeading(new Pose2d(-62, -5.5, Math.toRadians(180)))
                .forward(4)
                // grab cone
                .addDisplacementMarker(() -> {
                    clawServo.setPosition(1);
                    closeClaw = true;
                })

                .back(1)
                .addDisplacementMarker(() -> {
                    targetPos.set(-1000);
                })
                .back(3)
                .addDisplacementMarker(() -> {
                    targetPos.set(-2900);
                })

                .lineToLinearHeading(new Pose2d(-20, -3.5, Math.toRadians(90)))
                .back(3)
                // drop cone
                .addDisplacementMarker(() -> {
                    targetPos.set(0);
                    clawServo.setPosition(0);
                    closeClaw = false;

                })
                .forward(3)

                // Parking
                .lineTo(new Vector2d(-62,-2.5))
                .lineTo(new Vector2d(-62,-6.5))
                .build();

        TrajectorySequence parkNumber2 = drive.trajectorySequenceBuilder(startPose)
                .forward(4)

                .lineToLinearHeading(new Pose2d(-58, -60, Math.toRadians(0)))

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

                // Parking
                .lineTo(new Vector2d(-36,-12))
                .lineTo(new Vector2d(-36,-20))
                .build();
        TrajectorySequence parkNumber3 = drive.trajectorySequenceBuilder(startPose)
                .forward(4)

                .lineToLinearHeading(new Pose2d(-58, -60, Math.toRadians(0)))

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

                // Parking
                .lineTo(new Vector2d(-12,-12))
                .lineTo(new Vector2d(-12,-20))
                .build();

        while (!opModeIsActive() && !isStopRequested()) {
            // Update the vision object to detect any visible tags
            vision.updateTags();

            // If a tag is detected, get its ID and display it in telemetry
            if (vision.idGetter() != 0) {
                id = vision.idGetter();
                updateClawServo(clawServo);
                telemetry.addData("Cone id", "%d", id);
                telemetry.update();
            }
        }

        // Wait for the start command
        waitForStart();

        // Reset runtime
        runtime.reset();

        // Set the claw servo to position 0 and set closeClaw to false
        clawServo.setPosition(0);
        closeClaw = false;

        // Drive with the cone using the claw and pivot servos
        driveWithCone(clawServo, pivotServo);

        // Display a message in telemetry
        telemetryUpdate("Starting to follow trajectory");

        // Choose the appropriate trajectory based on the detected cone ID
        if (id == 440) { // Number 1
        } else if (id == 373) { // Number 2
            drive.followTrajectorySequenceAsync(parkNumber2);
            drive.followTrajectorySequenceAsync(parkNumber1);

        } else if (id == 182) { // Number 3
            drive.followTrajectorySequenceAsync(parkNumber3);
        } else { // Found none
            drive.followTrajectorySequenceAsync(parkNumber1);
        }

        // Run the loop until the op mode is no longer active
        while (opModeIsActive()) {
            // Update the drive object
            drive.update();

            // Update the power of the slide motors based on the target position and the current position
            updateMotors(targetPos.get(), slide1Motor, slide2Motor);

            // Update the position of the claw servo based on the value of closeClaw
            updateClawServo(clawServo);

            // Display the loop time in telemetry
            telemetry.addData("Loop time (ms)", runtime.milliseconds() - loopTime);
            loopTime = runtime.milliseconds();
            telemetry.update();
        }
    }

    /*

    ---------------- Function description ----------------

    This function is used to update the power of two motors (slide1Motor and slide2Motor) based on a target state (targetState) and a PID controller (slidesPID).
    The function calculates the output power for the motors using the getOutputFromError method of the slidesPID object, and then sets the power of the motors to this value.
     */
    void updateMotors(int targetState, DcMotor slide1Motor, DcMotor slide2Motor) {
        // Calculate the output power for the motors using a PID controller
        double currentArmPID = slidesPID.getOutputFromError(targetState, slide1Motor.getCurrentPosition());

        // Set the power of both motors to the calculated output power
        slide1Motor.setPower(currentArmPID);
        slide2Motor.setPower(currentArmPID);
    }


    void driveWithCone(Servo clawServo, Servo pivotServo) {
        // Set the claw servo to position 1
        clawServo.setPosition(1);

        // Update the claw servo position based on the value of closeClaw
        updateClawServo(clawServo);

        // Set closeClaw to true
        closeClaw = true;

        // Set the pivot servo to position .5
        pivotServo.setPosition(.5);
    }


    void updateClawServo(Servo clawServo) {
        // Check if it has been at least 3 seconds (3000 milliseconds) since the last time the claw servo was updated
        if (runtime.milliseconds() >= lastPing + 3000) {
            // Update the value of lastPing to the current time
            lastPing = runtime.milliseconds();

            // If closeClaw is true, toggle the position of the claw servo
            if (closeClaw) {
                // If the claw servo is currently at position 1, set it to .95
                if (clawServo.getPosition() == 1) {
                    clawServo.setPosition(.95);
                }
                // Otherwise, set the claw servo to position 1
                else {
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