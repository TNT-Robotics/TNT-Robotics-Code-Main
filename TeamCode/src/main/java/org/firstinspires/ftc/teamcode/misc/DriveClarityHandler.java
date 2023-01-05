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

        double slidesPower = -gamepad2.left_stick_y * 10;
        int armNewPos = (int) (cfg.getSlide1Position() + slidesPower);

        if (armNewPos < -4600) {
            armNewPos = -4600;
        }
        if (armNewPos > 0) {
            armNewPos = 0;
        }

        double currentArmPID = slidesPID.getOutputFromError(armNewPos, cfg.getSlide1Motor().getCurrentPosition());

        if (gamepad2.dpad_up) {
            armNewPos = -4114;
        }
        if (gamepad2.dpad_left) {
            armNewPos = -3023;
        }
        if (gamepad2.dpad_right) {
            armNewPos = -1755;
        }
        if (gamepad2.dpad_down) {
            armNewPos = 0;
        }

        // Secure placement
        if (gamepad2.triangle) {
            armNewPos = armNewPos + 200;
        }

        cfg.getSlide1Motor().setPower(currentArmPID);
        cfg.getSlide2Motor().setPower(currentArmPID);

        cfg.setSlide1Position(armNewPos);
    }

    public boolean updateGamepadServos(Gamepad gamepad2, boolean closeClaw, Config cfg) {
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

        return closeClaw;
    }

    public double[] updateConeServos(Gamepad gamepad2, double turnInit, double turnInit2, Config cfg) {
        double closeClaw = 0;
        // Pickup
        if (gamepad2.cross) {
            cfg.getPivotServo().setPosition(0);
            cfg.getRotateServo().setPosition(0);

            if (turnInit2 == 0) {
                turnInit2 = cfg.getrTime().milliseconds();
                turnInit = 0;
            }
        }
        // Placedown (put on pole)
        if (gamepad2.circle) {
            cfg.getClawServo().setPosition(1);
            closeClaw = 1;
            cfg.getPivotServo().setPosition(0.05);
            cfg.getRotateServo().setPosition(1);

            if(turnInit == 0) {
                turnInit = cfg.getrTime().milliseconds();
                turnInit2 = 0;
            }
        }

        if (gamepad2.square) {
            cfg.getClawServo().setPosition(1);
            closeClaw = 1;
            cfg.getPivotServo().setPosition(.5);
        }

        return new double[] { turnInit, turnInit2, closeClaw};
    }

    public double[] updateServosAfterDelay(double turnInit, double turnInit2, double lastPing, boolean closeClaw, Config cfg) {
        if (cfg.getrTime().milliseconds() >= turnInit + 1000 && turnInit != 0) {
            turnInit = 0;
            cfg.getPivotServo().setPosition(1);
        }

        if (cfg.getrTime().milliseconds() >= turnInit2 + 1000 && turnInit2 != 0) {
            turnInit2 = 0;

            closeClaw = false;
            cfg.getClawServo().setPosition(0);
        }

        if (cfg.getrTime().milliseconds() >= lastPing + 3000) {
            lastPing = cfg.getrTime().milliseconds();
            if (closeClaw) {
                if (cfg.getClawServo().getPosition() == 1) {
                    cfg.getClawServo().setPosition(.95);
                } else {
                    cfg.getClawServo().setPosition(1);
                }
            }
        }


        return new double[] {turnInit, turnInit2, lastPing};
    }
}
