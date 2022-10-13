package org.firstinspires.ftc.teamcode.auton;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.vision.AprilTagDemo;

@TeleOp(name="blueAutoV1", group="Blue")
// REMIND ME (Tomas) TO CHECK res/xml/teamwebcamcalibrations.xml ty
public class blueAutoV1 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor armMotor = null;
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
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        //slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");

        // ASSIGN SERVOS
        left_hand = hardwareMap.get(Servo.class, "left_hand");

        // DRIVE MOTOR DIRECTION
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // ARM MOTOR DIRECTION
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD); // TEST FORWARD OR BACKWARDS
        //slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // TELEMETRY
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // BEGIN CODE
        while (opModeIsActive()) {

        }
    }
}
