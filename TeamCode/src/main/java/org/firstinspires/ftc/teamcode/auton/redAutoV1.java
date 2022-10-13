package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;

import org.firstinspires.ftc.teamcode.vision.AprilTagDemo;
import org.openftc.easyopencv.OpenCvCameraFactory;

@TeleOp(name = "redAutoV1", group = "Red")
public class redAutoV1 extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    //private DcMotor armMotor = null;
    //private DcMotor slideMotor = null;

    private final double speedMultiplier = 1;


    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_POS = 1;     // Maximum rotational position
    static final double MIN_POS = 0;     // Minimum rotational position
    Servo left_hand;
    //double  position = ((MAX_POS - MIN_POS) / 2) + MIN_POS; // Start at halfway position

    // vision
    OpenCvCamera camera;

    AprilTagDemo vision = new AprilTagDemo();
    int coneId = 0;

    double[] position = {0, 0, 0}; // X,Y,Z
    double[] rotation = {0, 0, 0}; // YAW, PITCH, ROLL

    @Override
    public void runOpMode() {

        // BEGIN VISION
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        // ASSIGN DRIVE MOTORS
        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl");
        leftBackDrive = hardwareMap.get(DcMotor.class, "bl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br");

        // ASSIGN LINEAR SLIDE / ARM MOTOR
        //armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        //slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");

        // ASSIGN SERVOS
        left_hand = hardwareMap.get(Servo.class, "left_hand");

        // DRIVE MOTOR DIRECTION
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // ARM MOTOR DIRECTION
        //armMotor.setDirection(DcMotorSimple.Direction.FORWARD); // TEST FORWARD OR BACKWARDS
        //slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // TELEMETRY
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // write test code here

        // if nothing detected for 1.5s move a bit forward until 5s then stop searching for cone id
        while (runtime.milliseconds() < 5000) {
            vision.updateTags(camera);

            if (vision.idGetter() != 0) {
                // ID Found
                coneId = vision.idGetter();
                position[0] = vision.xGetter();
                position[1] = vision.yGetter();
                position[2] = vision.zGetter();

                rotation[0] = vision.yawGetter();
                rotation[1] = vision.pitchGetter();
                rotation[2] = vision.rollGeter();

                updateTele("Found ID " + coneId, 0);
            }
            if (runtime.milliseconds() > 1500) {
                goForward(.2, 100);
                dontMove(50);
            } else {
                sleep(50);
            }
        }

        // grab cones
        for (int counter = 0; counter < 3; counter++) {
            moveToPole();
            dontMove(1000);
            moveToCone();
        }

        goToAfterId(
                coneId, // ID
                position[0], position[1], position[2], // POS
                rotation[0], rotation[1], rotation[2] // ROT
        );

        /*
        while (opModeIsActive()) {
        }*/
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

    public void goToAfterId(int id, double xdist, double ydist, double zdist, double roll, double yaw, double pitch) {
        if (id == 440) { // Number 1

        } else if (id == 373) { // Number 2

        } else if (id == 182) { // Number 3

        }
    }
}
