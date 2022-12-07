package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.misc.PID;
import org.firstinspires.ftc.teamcode.misc.config;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous
public class auton1 extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    config cfg = new config();

    PID slidesPID = new PID(.02,0,.02,.008);
    //PID elbowPID = new PID(.02,.0,.02,.008);


    //elbowPID.getOutputFromError(0,0);
    int coneStackValue = 5;

    double lastPing = 0;
    boolean closeClaw = true;
    // ASSIGN LINEAR SLIDE / ARM MOTOR
    DcMotor slide1Motor = hardwareMap.get(DcMotor .class, "s1");//slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
    DcMotor slide2Motor = hardwareMap.get(DcMotor.class, "s2");//slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");


    // ASSIGN SERVOS
    Servo clawServo = hardwareMap.get(Servo .class, "clawServo");
    Servo rotateServo = hardwareMap.get(Servo.class, "rotateServo");
    Servo pivotServo = hardwareMap.get(Servo.class, "pivotServo");

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        slidesPID.getOutputFromError(0,0);

        Trajectory drivePreload = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(33.5)
                .forward(27)
                .build();
        Trajectory drivePreload2 = drive.trajectoryBuilder(drivePreload.end())
                .back(27)
                .build();
        while (!opModeIsActive() && !isStopRequested()) {
            //loop detection here
            cfg.getVision().updateTags(cfg);

            if (cfg.getVision().idGetter() != 0) {
                // ID Found
                int cConeId = cfg.getVision().idGetter();
                cfg.setConeId(cfg.getVision().idGetter());
                telemetry.addData("Cone id", "%d", cConeId);
                telemetry.update();
            }
        }
        waitForStart();

        if(isStopRequested()) return;

        driveWithCone();
        drive.followTrajectory(drivePreload);
        placeOnPole(3);
        prepareForGrab();
        drive.followTrajectory(drivePreload2);
    }

    void placeOnPole(int level) {
        if (level == 0) {
            placeOnPoleHandler(-150);
            return;
        }
        if (level == 1) {
            placeOnPoleHandler(-2179);
            return;
        }
        if (level == 2) {
            placeOnPoleHandler(-3440);
            return;
        }
        if (level == 3) {
            placeOnPoleHandler(-4610);
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

    void updateMotors(int targetState) {
        double currentArmPID = slidesPID.getOutputFromError(targetState, slide1Motor.getCurrentPosition());
        slide1Motor.setPower(currentArmPID);
        slide2Motor.setPower(currentArmPID);
    }

    void placeOnPoleHandler(int targetPos) {
        while (closeEnough(slide1Motor.getCurrentPosition(), targetPos, 20) == false) {
            updateMotors(targetPos);
            updateClawServo();
        }
        clawServo.setPosition(1);
        closeClaw = false;
        pivotServo.setPosition(.15);
        rotateServo.setPosition(0);
        double currentMsTime = runtime.milliseconds();
        while (runtime.milliseconds() < currentMsTime + 750) {
            updateMotors(targetPos);
            updateClawServo();
        }
        pivotServo.setPosition(1);
        currentMsTime = runtime.milliseconds();
        while (runtime.milliseconds() < currentMsTime + 750) {
            updateMotors(targetPos);
            updateClawServo();
        }
        clawServo.setPosition(0);
        closeClaw = false;
    }

    void driveWithCone() {
        clawServo.setPosition(1);
        updateClawServo();
        closeClaw = true;
        pivotServo.setPosition(.5);
    }

    void updateClawServo() {
        if (runtime.milliseconds() >= lastPing + 3000) {
            lastPing = runtime.milliseconds();
            if (closeClaw) {
                if (clawServo.getPosition() == 1) {
                    clawServo.setPosition(.95);
                } else {
                    clawServo.setPosition(1);
                }
            } else {
                if (clawServo.getPosition() == 0) {
                    clawServo.setPosition(0.05);
                } else {
                    clawServo.setPosition(0);
                }
            }
        }
    }

    void prepareForGrab() {
        pivotServo.setPosition(0.05);
        rotateServo.setPosition(1);
        double currentMsTime = runtime.milliseconds();
        boolean servoTurned = false;
        while (closeEnough(slide1Motor.getCurrentPosition(), 0, 20) == false) {
            updateMotors(0);
            updateClawServo();
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

    void grabCone() {

    }
}