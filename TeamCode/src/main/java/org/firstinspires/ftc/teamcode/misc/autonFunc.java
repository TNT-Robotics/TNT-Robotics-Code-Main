package org.firstinspires.ftc.teamcode.misc;

import org.firstinspires.ftc.teamcode.auton.redAutoV1;

public class autonFunc {
    config cfg = new config();
    redAutoV1 newFunctions = new redAutoV1();

    public void goForward(double power, int time) { // 1 sec is about 9ft
        newFunctions.updateTele("Going forward with power " + power + " for " + time + "ms", 0);
        cfg.getRfD().setPower(power);
        cfg.getLfD().setPower(power);
        cfg.getLbD().setPower(power);
        cfg.getRbD().setPower(power);
        newFunctions.slp(time);
        dontMove();
    }

    public void goBackwards(double power, int time) {
        power *= -1;

        newFunctions.updateTele("Going backwards with power " + power + " for " + time + "ms", 0);

        cfg.getRfD().setPower(power);
        cfg.getLfD().setPower(power);
        cfg.getLbD().setPower(power);
        cfg.getRbD().setPower(power);
        newFunctions.slp(time);
        dontMove();
    }

    public void dontMove(int time) {

        newFunctions.updateTele("Waiting for " + time + "ms", 0);
        cfg.getRfD().setPower(0);
        cfg.getLfD().setPower(0);
        cfg.getLbD().setPower(0);
        cfg.getRbD().setPower(0);
        newFunctions.slp(time);
    }

    public void dontMove() {
        newFunctions.updateTele("Stopped", 0);
        cfg.getRfD().setPower(0);
        cfg.getLfD().setPower(0);
        cfg.getLbD().setPower(0);
        cfg.getRbD().setPower(0);
    }

    public void diagonalLeft(double power, int time) {
        newFunctions.updateTele("Strafing diagonal left with power " + power + " for " + time + "ms", 0);
        cfg.getRfD().setPower(power);
        cfg.getLfD().setPower(0);
        cfg.getLbD().setPower(power);
        cfg.getRbD().setPower(0);
        newFunctions.slp(time);
        dontMove();
    }

    public void diagonalRight(double power, int time) {
        newFunctions.updateTele("Strafing diagonal right with power " + power + " for " + time + "ms", 0);
        cfg.getRfD().setPower(0);
        cfg.getLfD().setPower(power);
        cfg.getLbD().setPower(0);
        cfg.getRbD().setPower(power);
        newFunctions.slp(time);
        dontMove();
    }

    public void turnRight(double power, int time) {
        newFunctions.updateTele("Turning right with power " + power + " for " + time + "ms", 0);
        cfg.getRfD().setPower(0);
        cfg.getLfD().setPower(power);
        cfg.getLbD().setPower(power);
        cfg.getRbD().setPower(0);
        newFunctions.slp(time);
        dontMove();
    }

    public void turnLeft(double power, int time) {
        newFunctions.updateTele("Turning left with power " + power + " for " + time + "ms", 0);
        cfg.getRfD().setPower(power);
        cfg.getLfD().setPower(0);
        cfg.getLbD().setPower(0);
        cfg.getRbD().setPower(power);
        newFunctions.slp(time);
        dontMove();
    }

    public void turn90left() {
        newFunctions.updateTele("Doing 90 degrees left turn!", 0);
        cfg.getRfD().setPower(1);
        cfg.getLfD().setPower(0);
        cfg.getLbD().setPower(0);
        cfg.getRbD().setPower(1);
        newFunctions.slp(500);
        dontMove();
    }

    public void turn90right() {
        newFunctions.updateTele("Doing 90 degrees right turn!", 0);
        cfg.getRfD().setPower(0);
        cfg.getLfD().setPower(1);
        cfg.getLbD().setPower(1);
        cfg.getRbD().setPower(0);
        newFunctions.slp(500);
        dontMove();
    }

    public void strafeLeft(double power, int time) {
        newFunctions.updateTele("Strafing left with power " + power + " for " + time + " ms.", 0);
        cfg.getRfD().setPower(power);
        cfg.getLfD().setPower(power * -1);
        cfg.getLbD().setPower(power);
        cfg.getRbD().setPower(power * - 1);
        newFunctions.slp(time);
        dontMove();
    }

    public void strafeRight(double power, int time) {
        newFunctions.updateTele("Strafing left with power " + power + " for " + time + " ms.", 0);
        cfg.getRfD().setPower(power * -1);
        cfg.getLfD().setPower(power);
        cfg.getLbD().setPower(power * -1);
        cfg.getRbD().setPower(power);
        newFunctions.slp(time);
        dontMove();
    }

    // ARM Code
    // 1 rotation (360 degrees) is equal to about 138ms (435rpm)
    // 360 = 138ms | 270 = 103ms | 180 = 69ms | 90 = 34ms
    public void moveArmForward(double power, int time) {
        newFunctions.updateTele("Going turning arm forward with " + power + " for " + time + "ms", 0);
        //armMotor.setPower(power);
        dontMove(time);
    }

    public void moveArmBackwards(double power, int time) {
        newFunctions.updateTele("Going turning arm backwards with " + power + " for " + time + "ms", 0);
        //armMotor.setPower(power * -1);
        dontMove(time);
    }

    // advanced drive

    public void moveToPole(int team) {
        if (team == 0) { // BLUE CODE (EDIT)
            strafeRight(1, 600);
            dontMove(1000);
            goBackwards(1, 350);
        } else if (team == 1) {
            goForward(1, 350);
            dontMove(1000);
            strafeLeft(1, 600);
        }
    }

    public void moveToCone(int team) {
        if (team == 0) { // BLUE CODE (EDIT)
            strafeRight(1, 600);
            dontMove(1000);
            goBackwards(1, 350);
        } else if (team == 1) { // RED CODE
            strafeRight(1, 600);
            dontMove(1000);
            goBackwards(1, 350);
        }
    }

    // cones
    public void grabCone() {

    }

    public void placeCone(int level) { // small = 0, medium = 1, large = 2

    }

    // vision

    public void goToAfterId(int id, double[] pos, double[] rot, int team) {
        if (team == 0) { // BLUE CODE
            if (id == 440) { // Number 1

            } else if (id == 373) { // Number 2

            } else if (id == 182) { // Number 3

            }
        } else if (team == 1) { // RED CODE
            if (id == 440) { // Number 1

            } else if (id == 373) { // Number 2

            } else if (id == 182) { // Number 3

            }
        }
    }
}
