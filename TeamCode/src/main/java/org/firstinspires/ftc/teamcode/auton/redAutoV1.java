package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;

import org.firstinspires.ftc.teamcode.vision.AprilTagDemo;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous
public class redAutoV1 extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // private DcMotor pivotMotor = null;
    // private DcMotor armMotor = null;

    // motors vars
    // 1 rotation (360 degrees) is equal to about 145ms
    // 360 = 145ms or 146ms (145.6ms) | 270 = 109ms | 180 = 73ms | 90 = 36ms

    static final double MAX_POS = 1; // Maximum rotational position
    static final double MIN_POS = 0; // Minimum rotational position
    Servo claw1;
    Servo claw2;
    Servo claw3;
    Servo claw4;
    // double position = ((MAX_POS - MIN_POS) / 2) + MIN_POS; // Start at halfway
    // position

    // vision
    int camCounter = 0;
    OpenCvCamera camera;

    AprilTagDemo vision = new AprilTagDemo();
    int coneId = 0;

    double[] position = { 0, 0, 0 }; // X,Y,Z
    double[] rotation = { 0, 0, 0 }; // YAW, PITCH, ROLL

    @Override
    public void runOpMode() {

        // BEGIN VISION
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId);

        // ASSIGN DRIVE MOTORS
        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");

        // ASSIGN LINEAR SLIDE / ARM MOTOR
        // pivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");
        // elbowMotor = hardwareMap.get(DcMotor.class, "elbowMotor");

        // ASSIGN SERVOS
        claw1 = hardwareMap.get(Servo.class, "left_hand"); // CHANGE NAME
        // claw2 = hardwareMap.get(Servo.class, "claw2");
        // claw3 = hardwareMap.get(Servo.class, "claw3");
        // claw4 = hardwareMap.get(Servo.class, "claw4");

        // DRIVE MOTOR DIRECTION
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // ARM MOTOR DIRECTION
        // armMotor.setDirection(DcMotorSimple.Direction.FORWARD); // TEST FORWARD OR
        // BACKWARDS
        // slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // TELEMETRY
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // write test code here

        // if nothing detected for 1.5s move a bit forward until 5s then stop searching
        // for cone id
        while (runtime.milliseconds() < 5000) {
            vision.updateTags(camera);

            if (vision.idGetter() != 0) {
                // ID Found
                coneId = vision.idGetter();
                position = vision.position();
                rotation = vision.rotation();

                updateTele("Found ID " + coneId, 0);
            }
            if (runtime.milliseconds() > 1500) {
                goForward(.2, 100);
                dontMove(50);
                camCounter++;
            } else {
                sleep(50);
            }
        }
        if (camCounter > 0) {
            goBackwards(1, (camCounter * 100) / 5);
        }

        // grab cones
        for (int counter = 0; counter < 3; counter++) {
            moveToPole();
            dontMove(200);
            placeCone(2);
            moveToCone();
            grabCone();
        }

        goToAfterId(coneId, position, rotation);

        /*
         * while (opModeIsActive()) {
         * }
         */
    }

    // statusNum > 0 = Nominal | 1 = Warning | 2 = Minor Error | 3 = Fatal Error
    public void updateTele(String action, int statusNum) {
        telemetry.addData("Status", statusNum);
        telemetry.addData("Action", action);
        telemetry.addData("Running for", runtime.toString());
        telemetry.update();
    }

    public void goForward(double power, int time) { // 1 sec is about 9ft
        updateTele("Going forward with power " + power + " for " + time + "ms", 0);
        rightFrontDrive.setPower(power);
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        sleep(time);
        dontMove();
    }

    public void goBackwards(double power, int time) {
        power *= -1;

        updateTele("Going backwards with power " + power + " for " + time + "ms", 0);

        rightFrontDrive.setPower(power);
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        sleep(time);
        dontMove();
    }

    public void dontMove(int time) {

        updateTele("Waiting for " + time + "ms", 0);
        rightFrontDrive.setPower(0);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        sleep(time);
    }

    public void dontMove() {
        updateTele("Stopped", 0);
        rightFrontDrive.setPower(0);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void diagonalLeft(double power, int time) {
        updateTele("Strafing diagonal left with power " + power + " for " + time + "ms", 0);
        rightFrontDrive.setPower(power);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(0);
        sleep(time);
        dontMove();
    }

    public void diagonalRight(double power, int time) {
        updateTele("Strafing diagonal right with power " + power + " for " + time + "ms", 0);
        rightFrontDrive.setPower(0);
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(power);
        sleep(time);
        dontMove();
    }

    public void turnRight(double power, int time) {
        updateTele("Turning right with power " + power + " for " + time + "ms", 0);
        rightFrontDrive.setPower(0);
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(0);
        sleep(time);
        dontMove();
    }

    public void turnLeft(double power, int time) {
        updateTele("Turning left with power " + power + " for " + time + "ms", 0);
        rightFrontDrive.setPower(power);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(power);
        sleep(time);
        dontMove();
    }

    public void turn90left() {
        updateTele("Doing 90 degrees left turn!", 0);
        rightFrontDrive.setPower(1);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(1);
        sleep(500);
        dontMove();
    }

    public void turn90right() {
        updateTele("Doing 90 degrees right turn!", 0);
        rightFrontDrive.setPower(0);
        leftFrontDrive.setPower(1);
        leftBackDrive.setPower(1);
        rightBackDrive.setPower(0);
        sleep(500);
        dontMove();
    }

    public void strafeLeft(double power, int time) {
        updateTele("Strafing left with power " + power + " for " + time + " ms.", 0);
        rightFrontDrive.setPower(power);
        leftFrontDrive.setPower(power * -1);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power * -1);
        sleep(time);
        dontMove();
    }

    public void strafeRight(double power, int time) {
        updateTele("Strafing left with power " + power + " for " + time + " ms.", 0);
        rightFrontDrive.setPower(power * -1);
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power * -1);
        rightBackDrive.setPower(power);
        sleep(time);
        dontMove();
    }

    // ARM Code
    // 1 rotation (360 degrees) is equal to about 145ms
    // 360 = 145ms or 146ms (145.6ms) | 270 = 109ms | 180 = 73ms | 90 = 36ms
    public void moveArmForward(double power, int time) {
        updateTele("Going turning arm forward with " + power + " for " + time + "ms", 0);
        //armMotor.setPower(power);
        dontMove(time);
    }

    public void moveArmBackwards(double power, int time) {
        updateTele("Going turning arm backwards with " + power + " for " + time + "ms", 0);
        //armMotor.setPower(power * -1);
        dontMove(time);
    }

    // advanced drive

    public void moveToPole() {
        goForward(1, 350);
        dontMove(1000);
        strafeLeft(1, 600);
    }

    public void moveToCone() {
        strafeRight(1, 600);
        dontMove(1000);
        goBackwards(1, 350);
    }

    // cones
    public void grabCone() {

    }

    public void placeCone(int level) { // small = 0, medium = 1, large = 2

    }

    // vision

    public void goToAfterId(int id, double[] pos, double[] rot) {
        if (id == 440) { // Number 1

        } else if (id == 373) { // Number 2

        } else if (id == 182) { // Number 3

        }
    }
}
