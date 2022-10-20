/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.driver;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.lang.Math;

import org.firstinspires.ftc.teamcode.misc.config;
import org.firstinspires.ftc.teamcode.misc.driverInit;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MainDrive", group="Driving")

public class MainDrive extends LinearOpMode {
/*
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor armMotor = null;
    private DcMotor elbowMotor = null;
    //private DcMotor slideMotor = null;

    private double speedMultiplier = 1;
    private double armElbowSpeedMultiplier = .5;

    boolean armBool = false;
    boolean elbowBool = false;

    int armCurrentPos = 0;
    int elbowCurrentPos = 0;

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1;     // Maximum rotational position
    static final double MIN_POS     =  0;     // Minimum rotational position


    Servo arm1;
    Servo arm2;
    Servo arm3;
    //double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position

 */
    public void runOpMode() {
        driverInit init = new driverInit();
        config cfg = new config();

        // INIT
        init.initDrive(hardwareMap);
        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        cfg.getrTime().reset();

        // BEGIN CODE
        while (opModeIsActive()) {

            // START OF MOTORS

            // START OF DRIVING
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // arm variable for power level
            double armPower = -gamepad2.left_stick_y;
            double elbowPower = -gamepad2.right_stick_y;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // START OF FAST / SLOW MODE

            // Change state
            if (gamepad1.cross) {
                cfg.setSpdMult(1);
            }
            if (gamepad1.square) {
                cfg.setSpdMult(.75);
            }
            if (gamepad1.triangle) {
                cfg.setSpdMult(.5);
            }
            if (gamepad1.circle) {
                cfg.setSpdMult(.25);
            }
            // Adjust power
            leftFrontPower *= cfg.getSpdMult();
            rightFrontPower *= cfg.getSpdMult();
            leftBackPower *= cfg.getSpdMult();
            rightBackPower *= cfg.getSpdMult();
            // END OF FAST / SLOW MODE
            // UPDATE WHEELS POWER
            cfg.getLfD().setPower(leftFrontPower);
            cfg.getRfD().setPower(rightFrontPower);
            cfg.getLbD().setPower(leftBackPower);
            cfg.getRbD().setPower(rightBackPower);

            // END OF DRIVING
            // START OF LINEAR SLIDE / ARM (MOTOR)

            // gamepad2 speed mutliplier
            // Change state
            if (gamepad2.cross) {
                cfg.setArmElbowSpdMult(1);
            }
            if (gamepad2.square) {
                cfg.setArmElbowSpdMult(.75);
            }
            if (gamepad2.triangle) {
                cfg.setArmElbowSpdMult(.5);
            }
            if (gamepad2.circle) {
                cfg.setArmElbowSpdMult(.25);
            }



           // Set power
            int armNewPos = (int)(cfg.getArmPos() + armPower * .1);
            int elbowNewPos = (int)(cfg.getElbowPos() + elbowPower * .1);

            cfg.getArm().setTargetPosition(armNewPos);
            cfg.getElbow().setTargetPosition(elbowNewPos);

            cfg.setArmPos(armNewPos);
            cfg.setElbowPos(elbowNewPos);


            // END OF LINEAR SLIDE / ARM (MOTOR)

            // END OF MOTORS
            // START OF SERVOS
            // START OF CLAW 1 (
            if (gamepad2.left_bumper) {
                if (cfg.getA1().getPosition() >= cfg.getMAX_POS()) {
                    cfg.getA1().setPosition(cfg.getMAX_POS());
                } else {
                    cfg.getA1().setPosition(cfg.getA1().getPosition() + cfg.getINCREMENT());
                }
            }
            if (gamepad2.right_bumper) {
                if (cfg.getA1().getPosition() <= cfg.getMIN_POS()) {
                    cfg.getA1().setPosition(cfg.getMIN_POS());
                } else {
                    cfg.getA1().setPosition(cfg.getA1().getPosition() - cfg.getINCREMENT());
                }
            }
            // END OF CLAW 1

            // START OF CLAW 2
            if (gamepad2.left_trigger != 0) {
                if (cfg.getA1().getPosition() >= cfg.getMAX_POS()) {
                    cfg.getA1().setPosition(cfg.getMAX_POS());
                } else {
                    cfg.getA1().setPosition(cfg.getA1().getPosition() + cfg.getINCREMENT());
                }
            }
            if (gamepad2.left_trigger != 0) {
                if (cfg.getA2().getPosition() <= cfg.getMIN_POS()) {
                    cfg.getA2().setPosition(cfg.getMIN_POS());
                } else {
                    cfg.getA2().setPosition(cfg.getA2().getPosition() - cfg.getINCREMENT());
                }
            }
            // END OF CLAW 2

            // START OF CLAW 3
            if (gamepad2.dpad_up) {
                if (cfg.getA3().getPosition() >= cfg.getMAX_POS()) {
                    cfg.getA3().setPosition(cfg.getMAX_POS());
                } else {
                    cfg.getA3().setPosition(cfg.getA3().getPosition() + cfg.getINCREMENT());
                }
            }
            if (gamepad2.dpad_down) {
                if (cfg.getA3().getPosition() <= cfg.getMIN_POS()) {
                    cfg.getA3().setPosition(cfg.getMIN_POS());
                } else {
                    cfg.getA3().setPosition(cfg.getA3().getPosition() - cfg.getINCREMENT());
                }
            }
            // END OF CLAW 3

            // END OF SERVOS

            // telemetry
            telemetry.addData("Status", "Run Time: " + cfg.getrTime().toString());
            telemetry.addLine("Motors");
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Arm", "%4.2f", armPower);
            telemetry.addLine("Servos");
            telemetry.addData("Claw1", "%4.2f", cfg.getA1().getPosition());
            telemetry.addData("Claw2", "%4.2f", cfg.getA2().getPosition());
            telemetry.addData("Claw3", "%4.2f", cfg.getA3().getPosition());
            telemetry.update();
        }
    }

    public void grabCone() {

    }
    public void placeCode(int level) {

    }
}


