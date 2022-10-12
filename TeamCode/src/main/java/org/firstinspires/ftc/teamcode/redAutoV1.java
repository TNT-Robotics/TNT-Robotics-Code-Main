package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.String;

@TeleOp(name="redAutoV1", group="Linear Opmode")
public class redAutoV1 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    //private DcMotor armMotor = null;
    //private DcMotor slideMotor = null;

    private double speedMultiplier = 1;

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1;     // Maximum rotational position
    static final double MIN_POS     =  0;     // Minimum rotational position
    Servo left_hand;
    //double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position

    @Override
    public void runOpMode() {

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

        // move the robot forward
        goForward(.5, 1500);
        // move the robot backwards
        goBackwards(.5,1500);
        // Do not move
        dontMove(1000);

        // turn robot
        strafeLeft(.5, 1000);

        strafeRight(.5, 1000);

        dontMove(2000);

        turnLeft(1, 1000);

        dontMove(2000);

        turnRight(1,1000);

        dontMove(1000);

        turn90left();

        dontMove(1000);

        turn90right();

        dontMove(1000);

        turn90left();
        turn90left();

        runtime.reset();
        /*
        // BEGIN CODE
        while (opModeIsActive()) {

            // move forward toward the nearest high junction
            // place cone on the high junction
            // move back and continue until opMode isn't active


        }

         */
    }

    // statusNum > 0 = Nominal | 1 = Warning | 2 = Minor Error | 3 = Fatal Error
    public void updateTele(String action, int statusNum) {
        telemetry.addData("Status", statusNum);
        telemetry.addData("Action", action);
        telemetry.addData("Running for", runtime.toString());
        telemetry.update();
    }

    public void goForward(double power, int time) {
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

    public void strafeLeft(double power, int time) {
        updateTele("Strafing left with power " + power + " for " + time + "ms", 0);
        rightFrontDrive.setPower(power);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(0);
        sleep(time);
        dontMove();
    }

    public void strafeRight(double power, int time) {
        updateTele("Strafing right with power " + power + " for " + time + "ms", 0);
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
}
