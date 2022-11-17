package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class brianDriverInit {

    config cfg;

    public brianDriverInit(config cfg) {
        this.cfg = cfg;
    }

    public void initDrive(HardwareMap hwMap) {
        // Declare OpMode members for each of the 4 motors.
        ElapsedTime runtime = new ElapsedTime();

        DcMotor leftFrontDrive = null;
        DcMotor leftBackDrive = null;
        DcMotor rightFrontDrive = null;
        DcMotor rightBackDrive = null;

        DcMotor slide1Motor = null;
        DcMotor slide2Motor = null;
        //private DcMotor slideMotor = null;

        Servo arm1;
        Servo arm2;
        Servo arm3;

        double speedMultiplier = 1;
        double armElbowSpeedMultiplier = .5;

        //double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
        // ASSIGN DRIVE MOTORS
        leftFrontDrive = (hwMap.get(DcMotor.class, "leftFront"));
        leftBackDrive = hwMap.get(DcMotor.class, "leftRear");
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hwMap.get(DcMotor.class, "rightRear");


        // ASSIGN LINEAR SLIDE / ARM MOTOR
        slide1Motor = hwMap.get(DcMotor.class, "s1");//slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        slide2Motor = hwMap.get(DcMotor.class, "s2");//slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");


        // ASSIGN SERVOS
        arm1 = hwMap.get(Servo.class, "arm1");
        arm2 = hwMap.get(Servo.class, "arm2");
        arm3 = hwMap.get(Servo.class, "arm3");

        arm1.setPosition(0);
        arm2.setPosition(0);
        arm3.setPosition(1);


        // DRIVE MOTOR DIRECTION
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // ARM MOTOR DIRECTION
        slide1Motor.setDirection(DcMotorSimple.Direction.FORWARD); // TEST FORWARD OR BACKWARDS
        slide2Motor.setDirection(DcMotorSimple.Direction.FORWARD); // TEST FORWARD OR BACKWARDS

        //slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        /*
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // reset encoder
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        */

        slide1Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide1Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Setters

        //Speed
        cfg.setSpdMult(speedMultiplier);
        cfg.setArmElbowSpdMult(armElbowSpeedMultiplier);
        // Runtime
        cfg.setrTime(runtime);


        // Motors
        cfg.setLfD(leftFrontDrive);
        cfg.setLbD(leftBackDrive);
        cfg.setRfD(rightFrontDrive);
        cfg.setRbD(rightBackDrive);

        cfg.setArm(slide1Motor);
        cfg.setElbow(slide2Motor);

        // Servos
        cfg.setA1(arm1);
        cfg.setA2(arm2);
        cfg.setA3(arm3);


    }
}
