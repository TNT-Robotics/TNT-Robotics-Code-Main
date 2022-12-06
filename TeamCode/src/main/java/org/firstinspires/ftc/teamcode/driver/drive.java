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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.misc.PID;
import org.firstinspires.ftc.teamcode.misc.config;
import org.firstinspires.ftc.teamcode.misc.driveInit;

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

@TeleOp(name="Drive", group="Driving")

public class drive extends LinearOpMode {
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
        config cfg = new config();
        driveInit init = new driveInit(cfg);


        PID slidesPID = new PID(.02,.0,.02,.008);
        //PID elbowPID = new PID(.02,.0,.02,.008);

        slidesPID.getOutputFromError(0,0);
        //elbowPID.getOutputFromError(0,0);

        double loopTime = 0;
        double turnInit = 0;
        double turnInit2 = 0;
        double lastPing = 0;

        boolean closeClaw = false;

        // INIT
        init.initDrive(hardwareMap);
        telemetry.addData(">", "Ready");
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        gamepad1.rumble(5, 5, 100);
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
            double slidesPower = -gamepad2.left_stick_y * 10;

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
                cfg.setSpeedMultiplier(1);
            }
            if (gamepad1.square) {
                cfg.setSpeedMultiplier(.75);
            }
            if (gamepad1.triangle) {
                cfg.setSpeedMultiplier(.5);
            }
            if (gamepad1.circle) {
                cfg.setSpeedMultiplier(.25);
            }
            // Adjust power
            leftFrontPower *= cfg.getSpeedMultiplier();
            rightFrontPower *= cfg.getSpeedMultiplier();
            leftBackPower *= cfg.getSpeedMultiplier();
            rightBackPower *= cfg.getSpeedMultiplier();
            // END OF FAST / SLOW MODE
            // UPDATE WHEELS POWER
            cfg.getLfD().setPower(leftFrontPower);
            cfg.getRfD().setPower(rightFrontPower);
            cfg.getLbD().setPower(leftBackPower);
            cfg.getRbD().setPower(rightBackPower);

            // END OF DRIVING
            // START OF LINEAR SLIDE / ARM (MOTOR)

            // Set adjust position
            int armNewPos = (int) (cfg.getSlide1Position() + slidesPower);

            if (armNewPos < -4610) {
                armNewPos = -4610;
            }
            if (armNewPos > 0) {
                armNewPos = 0;
            }

            double currentArmPID = slidesPID.getOutputFromError(armNewPos, cfg.getSlide1Motor().getCurrentPosition());

            if (gamepad2.dpad_up) {
                armNewPos = -4610;
            }
            if (gamepad2.dpad_left) {
                armNewPos = -3440;
            }
            if (gamepad2.dpad_right) {
                armNewPos = -2179;
            }
            if (gamepad2.dpad_down) {
                armNewPos = 0;
            }

            cfg.getSlide1Motor().setPower(currentArmPID);
            cfg.getSlide2Motor().setPower(currentArmPID);

            cfg.setSlide1Position(armNewPos);

            // END OF LINEAR SLIDE / ARM (MOTOR)

            // END OF MOTORS
            // START OF SERVOS
            // START OF CLAW 1 (Claw grab)
            if (gamepad2.left_bumper) {
                cfg.getClawServo().setPosition(1);
                closeClaw = true;
            }
            if (gamepad2.right_bumper) {
                cfg.getClawServo().setPosition(0);
                closeClaw = false;

            }
            // END OF CLAW 1

            // START OF CLAW 2 (180 turn around)
            if (gamepad2.left_trigger != 0) {
                if (cfg.getRotateServo().getPosition() >= 1) {
                    cfg.getRotateServo().setPosition(1);
                } else {
                    cfg.getRotateServo().setPosition(cfg.getRotateServo().getPosition() + cfg.getINCREMENT());
                }
            }
            if (gamepad2.right_trigger != 0) {
                if (cfg.getRotateServo().getPosition() <= 0) {
                    cfg.getRotateServo().setPosition(0);
                } else {
                    cfg.getRotateServo().setPosition(cfg.getRotateServo().getPosition() - cfg.getINCREMENT());
                }
            }
            // END OF CLAW 2

            // IDK THE RIGHT FOUR SHAPES

            // Pickup
            if (gamepad2.cross) {
                cfg.getPivotServo().setPosition(0.05);
                cfg.getRotateServo().setPosition(1);

                if (turnInit2 == 0) {
                    turnInit2 = cfg.getrTime().milliseconds();
                }
            }


            // Placedown (put on pole)
            if (gamepad2.circle) {
                cfg.getClawServo().setPosition(1);
                closeClaw = true;

                if (cfg.getPivotServo().getPosition() < .5) {
                    cfg.getPivotServo().setPosition(.15);
                }

                cfg.getRotateServo().setPosition(0);
                if(turnInit == 0) {
                    turnInit = cfg.getrTime().milliseconds();
                }
            }

            if (gamepad2.square) {
                cfg.getClawServo().setPosition(1);
                closeClaw = true;
                cfg.getPivotServo().setPosition(.5);
            }


            if (cfg.getrTime().milliseconds() >= turnInit2 + 1000 && turnInit2 != 0) {
                turnInit2 = 0;

                closeClaw = false;
                cfg.getClawServo().setPosition(0);
            }

            if (cfg.getrTime().milliseconds() >= turnInit + 750 && turnInit != 0) {
                turnInit = 0;
                cfg.getPivotServo().setPosition(1);
            }
            if (cfg.getrTime().milliseconds() >= lastPing + 3000) {
                lastPing = cfg.getrTime().milliseconds();
                if (closeClaw) {
                    if (cfg.getClawServo().getPosition() == 1) {
                        cfg.getClawServo().setPosition(.95);
                    } else {
                        cfg.getClawServo().setPosition(1);
                    }
                } else {
                    if (cfg.getClawServo().getPosition() == 0) {
                        cfg.getClawServo().setPosition(0.05);
                    } else {
                        cfg.getClawServo().setPosition(0);
                    }
                }
            }

            // END OF SERVOS

            // telemetry
            telemetry.addData("Status", "Run Time: " + cfg.getrTime().toString());
            telemetry.addLine("Motors");
            telemetry.addData("Front left/Right", axial + lateral + yaw);
            telemetry.addData("Back  left/Right",  axial - lateral + yaw);
            telemetry.addLine("Servos");
            telemetry.addData("Claw1, Claw2, Claw3", "%4.2f, %4.2f, %4.2f", cfg.getClawServo().getPosition(), cfg.getRotateServo().getPosition(), cfg.getPivotServo().getPosition());
            telemetry.addLine("Motor Rotations (Current vs Set)");
            telemetry.addData("Arm", "%d, %d", cfg.getSlide1Motor().getCurrentPosition(), cfg.getSlide1MotorTargetPosition());
            telemetry.addData("Elbow", "%d, %d", cfg.getSlide2Motor().getCurrentPosition(), cfg.getSlide2MotorTargetPosition());
            telemetry.addData("Arm Power", "%4.2f", cfg.getSlide1Motor().getPower());
            telemetry.addData("Timers", "%4.2f, %4.2f, %4.2f", turnInit, turnInit2, lastPing/1000);
            telemetry.addData("Loop timer", "%4.2f", cfg.getrTime().milliseconds() - loopTime);
            telemetry.update();

            loopTime = cfg.getrTime().milliseconds();
        }
    }
}

