package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;


public class autonInit {

    config cfg = new config();

    public void initAuton(HardwareMap hwMap, int teamColor) {
        // Declare OpMode members for each of the 4 motors.
        ElapsedTime runtime = new ElapsedTime();

        DcMotor leftFrontDrive = null;
        DcMotor leftBackDrive = null;
        DcMotor rightFrontDrive = null;
        DcMotor rightBackDrive = null;

        DcMotor armMotor = null;
        DcMotor elbowMotor = null;
        //private DcMotor slideMotor = null;

        Servo arm1;
        Servo arm2;
        Servo arm3;

        double speedMultiplier = 1;
        double armElbowSpeedMultiplier = .5;

        boolean armBool = false;
        boolean elbowBool = false;

        int armCurrentPos = 0;
        int elbowCurrentPos = 0;

        //double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
        // ASSIGN DRIVE MOTORS
        leftFrontDrive = hwMap.get(DcMotor.class, "fl");
        leftBackDrive = hwMap.get(DcMotor.class, "bl");
        rightFrontDrive = hwMap.get(DcMotor.class, "fr");
        rightBackDrive = hwMap.get(DcMotor.class, "br");


        // ASSIGN LINEAR SLIDE / ARM MOTOR
        armMotor = hwMap.get(DcMotor.class, "armMotor");
        elbowMotor = hwMap.get(DcMotor.class, "elbowMotor");
        //slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");


        // ASSIGN SERVOS
        arm1 = hwMap.get(Servo.class, "arm1");
        arm2 = hwMap.get(Servo.class, "arm2");
        arm3 = hwMap.get(Servo.class, "arm3");


        // DRIVE MOTOR DIRECTION
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // ARM MOTOR DIRECTION
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD); // TEST FORWARD OR BACKWARDS
        elbowMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        // reset encoder
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set motors to run forward for 5000 encoder counts.
        armMotor.setTargetPosition(0);
        elbowMotor.setTargetPosition(0);
        // Set pos mode for grab mechanism motors
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Vision
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                hwMap.appContext.getPackageName());
        cfg.setCamera(OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId));

        // Setters

        //Speed
        cfg.setSpdMult(speedMultiplier);
        cfg.setArmElbowSpdMult(armElbowSpeedMultiplier);
        // Runtime
        cfg.setrTime(runtime);

        // Positions
        cfg.setArmPos(armCurrentPos);
        cfg.setElbowPos(elbowCurrentPos);

        // Motors
        cfg.setLfD(leftFrontDrive);
        cfg.setLbD(leftBackDrive);
        cfg.setRfD(rightFrontDrive);
        cfg.setRbD(rightBackDrive);

        cfg.setArm(armMotor);
        cfg.setElbow(elbowMotor);

        // Servos
        cfg.setA1(arm1);
        cfg.setA2(arm2);
        cfg.setA3(arm3);

        // Team
        cfg.setTeamColor(teamColor);

    }
}
