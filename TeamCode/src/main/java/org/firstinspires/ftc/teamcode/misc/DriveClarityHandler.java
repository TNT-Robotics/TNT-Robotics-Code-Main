package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

/**

 This class handles the control of the holonomic drive motors, robot speed, and linear slide motors
 using input from gamepads. It includes methods to update the motor speeds for the holonomic drive
 based on input from the gamepad's axial, lateral, and yaw values, as well as a method to update the
 robot's speed based on input from the gamepad's buttons. It also includes a method to update the
 linear slide motors based on input from the gamepad's left stick y-axis and dpad, and a method to
 update the servos for the claw, rotate, and pivot based on input from the gamepad's buttons.

 */







public class DriveClarityHandler {

    double max;

    public void updateHolonomicDriveMotors(double axial, double lateral, double yaw, DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4, Config cfg) {
        // Calculate motor speeds here
        double leftFrontMotor = axial + lateral + yaw;
        double rightFrontMotor = axial - lateral - yaw;
        double leftBackMotor = axial - lateral + yaw;
        double rightBackMotor = axial + lateral - yaw;

        max = Math.max(Math.abs(leftFrontMotor), Math.abs(rightFrontMotor));
        max = Math.max(max, Math.abs(leftBackMotor));
        max = Math.max(max, Math.abs(rightBackMotor));

        if (max > 1.0) {
            leftFrontMotor /= max;
            rightFrontMotor /= max;
            leftBackMotor /= max;
            rightBackMotor /= max;
        }


        cfg.getLfD().setPower(leftFrontMotor * cfg.getSpeedMultiplier());
        cfg.getRfD().setPower(rightFrontMotor * cfg.getSpeedMultiplier());
        cfg.getLbD().setPower(leftBackMotor * cfg.getSpeedMultiplier());
        cfg.getRbD().setPower(rightBackMotor * cfg.getSpeedMultiplier());

        // Set motor speeds
        motor1.setPower(leftFrontMotor);
        motor2.setPower(rightFrontMotor);
        motor3.setPower(leftBackMotor);
        motor4.setPower(rightBackMotor);
    }

    public void updateRobotSpeed(Gamepad gamepad1, Config cfg) {
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

    }

    public void updateSlideMotors(Gamepad gamepad2, PID slidesPID, Config cfg) {
        // Determine power of motors based on y-axis value of left stick on gamepad2
        double slidesPower = -gamepad2.left_stick_y * 10;
        // Calculate new desired position for first slide motor
        int armNewPos = (int) (cfg.getSlide1Position() + slidesPower);

        // If new position is less than -4600, set to -4600
        if (armNewPos < -4600) {
            armNewPos = -4600;
        }
        // If new position is greater than 0, set to 0
        if (armNewPos > 0) {
            armNewPos = 0;
        }

        // Calculate power for motors using PID controller and error between new position and current position of first slide motor
        double currentArmPID = slidesPID.getOutputFromError(armNewPos, cfg.getSlide1Motor().getCurrentPosition());

        // Check if dpad buttons on gamepad2 are pressed and set armNewPos accordingly
        if (gamepad2.dpad_up) {
            armNewPos = -3775;
        }
        if (gamepad2.dpad_left) {
            armNewPos = -2668;
        }
        if (gamepad2.dpad_right) {
            armNewPos = -1467;
        }
        if (gamepad2.dpad_down) {
            armNewPos = 0;
        }

        // If triangle button on gamepad2 is pressed, increase armNewPos by 200
        if (gamepad2.triangle) {
            armNewPos = armNewPos + 200;
        }

        // Set power of both motors to currentArmPID
        cfg.getSlide1Motor().setPower(currentArmPID);
        cfg.getSlide2Motor().setPower(currentArmPID);

        // Store new position of first slide motor in configuration object
        cfg.setSlide1Position(armNewPos);
    }


    public boolean updateGamepadServos(Gamepad gamepad2, boolean closeClaw, Config cfg) {
        // Check if left bumper button is pressed on gamepad2
        if (gamepad2.left_bumper) {
            // Set claw servo position to 1
            cfg.getClawServo().setPosition(1);
            // Set closeClaw variable to true
            closeClaw = true;
        }
        // Check if right bumper button is pressed on gamepad2
        if (gamepad2.right_bumper) {
            // Set claw servo position to 0
            cfg.getClawServo().setPosition(0);
            // Set closeClaw variable to false
            closeClaw = false;

        }
        // END OF CLAW 1

        // START OF CLAW 2 (180 turn around)
        // Check if left trigger button is pressed on gamepad2
        if (gamepad2.left_trigger != 0) {
            // Check if rotate servo position is already at or above 1
            if (cfg.getRotateServo().getPosition() >= 1) {
                // Set rotate servo position to 1
                cfg.getRotateServo().setPosition(1);
            } else {
                // Increment rotate servo position by INCREMENT value from cfg object
                cfg.getRotateServo().setPosition(cfg.getRotateServo().getPosition() + cfg.getINCREMENT());
            }
        }
        // Check if right trigger button is pressed on gamepad2
        if (gamepad2.right_trigger != 0) {
            // Check if rotate servo position is already at or below 0
            if (cfg.getRotateServo().getPosition() <= 0) {
                // Set rotate servo position to 0
                cfg.getRotateServo().setPosition(0);
            } else {
                // Decrement rotate servo position by INCREMENT value from cfg object
                cfg.getRotateServo().setPosition(cfg.getRotateServo().getPosition() - cfg.getINCREMENT());
            }
        }
        // END OF CLAW 2

        // Return closeClaw value
        return closeClaw;
    }

    public double[] updateConeServos(Gamepad gamepad2, double turnInit, double turnInit2, Config cfg, boolean bCloseClaw) {
        // Initialize closeClaw variable
        double closeClaw = 0;
        // If bCloseClaw is true, set closeClaw to 1
        if(bCloseClaw) {
            closeClaw = 1;
        }

        // Check if cross button is pressed on gamepad2 (Prepare for pickup)
        if (gamepad2.cross) {
            // Set pivot servo position to 0
            cfg.getPivotServo().setPosition(0);
            // Set rotate servo position to 0
            cfg.getRotateServo().setPosition(0);

            // Check if turnInit2 is equal to 0
            if (turnInit2 == 0) {
                // Set turnInit2 to current time in milliseconds
                turnInit2 = cfg.getrTime().milliseconds();
                // Set turnInit to 0
                turnInit = 0;
            }
        }
        // Check if circle button is pressed on gamepad2 (Put on pole)
        if (gamepad2.circle) {
            // Set claw servo position to 1
            cfg.getClawServo().setPosition(1);
            // Set closeClaw to 1
            closeClaw = 1;
            // Set pivot servo position to 0.05
            cfg.getPivotServo().setPosition(0.05);
            // Set rotate servo position to 0.8
            cfg.getRotateServo().setPosition(.80);

            // Check if turnInit is equal to 0
            if(turnInit == 0) {
                // Set turnInit to current time in milliseconds
                turnInit = cfg.getrTime().milliseconds();
                // Set turnInit2 to 0
                turnInit2 = 0;
            }
        }

        // Check if square button is pressed on gamepad2 (Drive with cone)
        if (gamepad2.square) {
            // Set claw servo position to 1
            cfg.getClawServo().setPosition(1);
            // Set closeClaw to 1
            closeClaw = 1;
            // Set pivot servo position to 0.5
            cfg.getPivotServo().setPosition(0.5);
        }

        // Return array containing turnInit, turnInit2, and closeClaw values
        return new double[] { turnInit, turnInit2, closeClaw};
    }


    public double[] updateServosAfterDelay(double turnInit, double turnInit2, double lastPing, boolean closeClaw, Config cfg) {
        // Check if current time in milliseconds is greater than or equal to turnInit + 1000 and turnInit is not equal to 0
        if (cfg.getrTime().milliseconds() >= turnInit + 1000 && turnInit != 0) {
            // Set turnInit to 0
            turnInit = 0;
            // Set pivot servo position to 1
            cfg.getPivotServo().setPosition(1);
        }

        // Check if current time in milliseconds is greater than or equal to turnInit2 + 1000 and turnInit2 is not equal to 0
        if (cfg.getrTime().milliseconds() >= turnInit2 + 1000 && turnInit2 != 0) {
            // Set turnInit2 to 0
            turnInit2 = 0;
            // Set claw servo position to 0
            cfg.getClawServo().setPosition(0);
            // Set closeClaw to false
            closeClaw = false;
        }

        // Check if current time in milliseconds is greater than or equal to lastPing + 3000
        if (cfg.getrTime().milliseconds() >= lastPing + 3000) {
            // Set lastPing to current time in milliseconds
            lastPing = cfg.getrTime().milliseconds();
            // Check if closeClaw is true
            if (closeClaw) {
                // If claw servo position is equal to 1, set position to 0.9
                if (cfg.getClawServo().getPosition() == 1) {
                    cfg.getClawServo().setPosition(.9);
                    // Otherwise, set claw servo position to 1
                } else {
                    cfg.getClawServo().setPosition(1);
                }
            }
        }

        // Return array containing turnInit, turnInit2, and lastPing values
        return new double[] {turnInit, turnInit2, lastPing};
    }

}
