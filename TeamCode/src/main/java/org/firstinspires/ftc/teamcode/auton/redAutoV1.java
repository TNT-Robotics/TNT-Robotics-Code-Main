package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.misc.autonInit;
import org.firstinspires.ftc.teamcode.misc.config;


/*
Arm1 - Grab servo
Arm2 - 180 degrees turn
Arm3 - Pivot

*/

@Autonomous
public class redAutoV1 extends LinearOpMode {

    config cfg = new config();
    @Override
    public void runOpMode() {

        autonInit aiInit = new autonInit(cfg);

        aiInit.initAuton(hardwareMap, 1);
        cfg.getVision().initCamera(cfg.getCamera());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        cfg.getrTime().reset();

        // if nothing detected for 1.5s move a bit forward until 5s then stop searching
        // for cone id
        while (cfg.getrTime().milliseconds() < 5000 && opModeIsActive()) {
            cfg.getVision().updateTags(cfg);

            if (cfg.getVision().idGetter() != 0) {
                // ID Found
                cfg.setConeId(cfg.getVision().idGetter());
                cfg.setPosition(cfg.getVision().position());
                cfg.setRotation(cfg.getVision().rotation());

                updateTele("Found ID " + cfg.getConeId(), 0);
                break;
            }
            if (cfg.getrTime().milliseconds() > 1500) {
                goForward(.2, 100);
                dontMove(50);
                cfg.setCamCounter(cfg.getCamCounter()+1);
            } else {
                sleep(50);
            }
        }
        if (cfg.getCamCounter() > 0) {
            goBackward(1, (cfg.getCamCounter() * 100) / 5);
        }

        // grab cones
        moveToPole(cfg.getTeamColor());
        for (int counter = 0; counter < 3; counter++) {
            conePhase(3, 10000); // Time before end in milliseconds
        }
        moveToSpawn(cfg.getTeamColor());

        goToAfterId(cfg.getConeId(), cfg.getPosition(), cfg.getRotation(), cfg.getTeamColor());

        /*
         * while (opModeIsActive()) {
         * }
         */
    }
    public void goForward(double power, int time) { // 1 sec is about 9ft
        updateTele("Going forward with power " + power + " for " + time + "ms", 0);
        cfg.getRfD().setPower(power);
        cfg.getLfD().setPower(power);
        cfg.getLbD().setPower(power);
        cfg.getRbD().setPower(power);
        slp(time);
        dontMove();
    }

    public void goBackward(double power, int time) {
        power *= -1;

        updateTele("Going backwards with power " + power + " for " + time + "ms", 0);

        cfg.getRfD().setPower(power);
        cfg.getLfD().setPower(power);
        cfg.getLbD().setPower(power);
        cfg.getRbD().setPower(power);
        slp(time);
        dontMove();
    }

    public void dontMove(int time) {

        updateTele("Waiting for " + time + "ms", 0);
        cfg.getRfD().setPower(0);
        cfg.getLfD().setPower(0);
        cfg.getLbD().setPower(0);
        cfg.getRbD().setPower(0);
        slp(time);
    }

    public void dontMove() {
        updateTele("Stopped", 0);
        cfg.getRfD().setPower(0);
        cfg.getLfD().setPower(0);
        cfg.getLbD().setPower(0);
        cfg.getRbD().setPower(0);
    }

    public void diagonalLeft(double power, int time) {
        updateTele("Strafing diagonal left with power " + power + " for " + time + "ms", 0);
        cfg.getRfD().setPower(power);
        cfg.getLfD().setPower(0);
        cfg.getLbD().setPower(power);
        cfg.getRbD().setPower(0);
        slp(time);
        dontMove();
    }

    public void diagonalRight(double power, int time) {
        updateTele("Strafing diagonal right with power " + power + " for " + time + "ms", 0);
        cfg.getRfD().setPower(0);
        cfg.getLfD().setPower(power);
        cfg.getLbD().setPower(0);
        cfg.getRbD().setPower(power);
        slp(time);
        dontMove();
    }

    public void turnRight(double power, int time) {
        updateTele("Turning right with power " + power + " for " + time + "ms", 0);
        cfg.getRfD().setPower(0);
        cfg.getLfD().setPower(power);
        cfg.getLbD().setPower(power);
        cfg.getRbD().setPower(0);
        slp(time);
        dontMove();
    }

    public void turnLeft(double power, int time) {
        updateTele("Turning left with power " + power + " for " + time + "ms", 0);
        cfg.getRfD().setPower(power);
        cfg.getLfD().setPower(0);
        cfg.getLbD().setPower(0);
        cfg.getRbD().setPower(power);
        slp(time);
        dontMove();
    }

    public void turn90left() {
        updateTele("Doing 90 degrees left turn!", 0);
        cfg.getRfD().setPower(1);
        cfg.getLfD().setPower(0);
        cfg.getLbD().setPower(0);
        cfg.getRbD().setPower(1);
        slp(500);
        dontMove();
    }

    public void turn90right() {
        updateTele("Doing 90 degrees right turn!", 0);
        cfg.getRfD().setPower(0);
        cfg.getLfD().setPower(1);
        cfg.getLbD().setPower(1);
        cfg.getRbD().setPower(0);
        slp(500);
        dontMove();
    }

    public void strafeLeft(double power, int time) {
        updateTele("Strafing left with power " + power + " for " + time + " ms.", 0);
        cfg.getRfD().setPower(power);
        cfg.getLfD().setPower(power * -1);
        cfg.getLbD().setPower(power);
        cfg.getRbD().setPower(power * - 1);
        slp(time);
        dontMove();
    }

    public void strafeRight(double power, int time) {
        updateTele("Strafing left with power " + power + " for " + time + " ms.", 0);
        cfg.getRfD().setPower(power * -1);
        cfg.getLfD().setPower(power);
        cfg.getLbD().setPower(power * -1);
        cfg.getRbD().setPower(power);
        slp(time);
        dontMove();
    }

    // ARM Code
    // 1 rotation (360 degrees) is equal to about 138ms (435rpm)
    // 360 = 138ms | 270 = 103ms | 180 = 69ms | 90 = 34ms
    public void moveArmForward(double power, int time) {
        updateTele("Going turning arm forward with " + power + " for " + time + "ms", 0);
        cfg.getArm().setPower(power);
        dontMove(time);
    }

    public void moveArmBackwards(double power, int time) {
        updateTele("Going turning arm backwards with " + power + " for " + time + "ms", 0);
        cfg.getArm().setPower(power * -1);
        dontMove(time);
    }

    public void setArmMotorPos(int position) { // re-do
        updateTele("Arm turning to " + position, 0);
        cfg.getArm().setTargetPosition(position);
    }
    public void setElbowMotorPos(int position) { // re-do
        updateTele("Elbow turning to " + position, 0);
        cfg.getElbow().setTargetPosition(position);
    }
    // advanced drive

    public void moveToPole(int team) {
        if (team == 0) { // BLUE CODE
            strafeRight(1, 600);
            dontMove(1000);
            goBackward(1, 350);
        } else if (team == 1) { // RED CODE
            goForward(1, 350);
            dontMove(1000);
            strafeLeft(1, 600);
        }
    }

    public void moveToSpawn(int team) {
        if (team == 0) { // BLUE CODE
            strafeRight(1, 600);
            dontMove(1000);
            goBackward(1, 350);
        } else if (team == 1) { // RED CODE
            strafeRight(1, 600);
            dontMove(1000);
            goBackward(1, 350);
        }
    }

    // cones

    // WRTIE THIS

    public void grabCone() {
        // open servo
        cfg.getA1().setPosition(.53);
        // turn motors to pickup zone
        // close servo
        cfg.getA1().setPosition(.12);


    }

    // WRITE THIS
    public void placeCone(int level) { // ground = 0, small = 1, medium = 2, high = 3
        if (level == 0) { // GROUND
            // move motors above pole
            // rotate claw 180 degrees
            // open claw
            // repeat for all levels

        } else if (level == 1) { // SMALL

        } else if (level == 2) { // MEDIUM

        } else if (level == 3) { // HIGH

        }
    }

    public void conePhase(int lvl, int timeBeforeEnd) {
        if (30000 - cfg.getrTime().milliseconds() < timeBeforeEnd) {
            placeCone(lvl);
            grabCone();
        }
    }

    // vision

    // WRITE THIS
    public void goToAfterId(int id, double[] pos, double[] rot, int team) {
        if (team == 0) { // BLUE CODE
            if (id == 440) { // Number 1
                // move to sector 1/2/3 depending on the id

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


    // statusNum > 0 = Nominal | 1 = Warning | 2 = Minor Error | 3 = Fatal Error
    public void updateTele(String action, int statusNum) {
                telemetry.addData("Status", statusNum);
                telemetry.addData("Action", action);
                telemetry.addData("Running for", cfg.getrTime());
                telemetry.update();
    }

    public void slp(int time) {
        sleep(time);
    }
}
