package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.misc.PID;
import org.firstinspires.ftc.teamcode.misc.autonInit;
import org.firstinspires.ftc.teamcode.misc.config;


/*
Arm1 - Grab servo
Arm2 - 180 degrees turn
Arm3 - Pivot

-------TO DO 10/24/2022-------

IF YOU NEED HELP
https://github.com/TNT-Robotics/TNT-Robotics-Code-Main/blob/main/README.md

Method MoveToPole (ADD FOR PHILIPS DESIGN AS CURRENT ONE IS FOR BRIAN'S)

Method placeCone (LOOK AT THE COMMENTS AND WRITE CODE ACCORDINGLY)

Method goToAfterId (EACH SECTION NEEDS TO BE DONE)

---- If you want extra challenge --
Method grabCone (LOOK AT COMMENTS CODE ACCORDINGLY)




Think how much time we need before that and write that down here. (this is the last moment we can go for
a cone place before returning back to spawn and going to id place)
How much time before end in milliseconds - < WRITE THE TIME HERE


*/

@Autonomous
public class redAutoV1 extends LinearOpMode {

    config cfg = new config();
    int lfdTarget = 0;
    int rfdTarget = 0;
    int lbdTarget = 0;
    int rbdTarget = 0;
    @Override
    public void runOpMode() {

            autonInit aiInit = new autonInit(cfg);


            PID lfdPID = new PID(.02, .0, .04, 0);
            PID lbdPID = new PID(.02, .0, .04, .0);
            PID rfdPID = new PID(.02, .0, .04, .0);
            PID rbdPID = new PID(.02, .0, .04, .0);

            lfdPID.getOutputFromError(0, 0);
            lbdPID.getOutputFromError(0, 0);
            rfdPID.getOutputFromError(0, 0);
            rbdPID.getOutputFromError(0, 0);

            aiInit.initAuton(hardwareMap, 1);
            cfg.getVision().initCamera(cfg.getCamera());
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();
            cfg.getrTime().reset();

        while (opModeIsActive()) {
            cfg.setHappenedCycle(false);

            // if nothing detected for 1.5s move a bit forward until 5s then stop searching
            // for cone id

            while (cfg.getrTime().milliseconds() < 5000 && opModeIsActive()) {
                cfg.getVision().updateTags(cfg);

                if (cfg.getVision().idGetter() != 0) {
                    // ID Found
                    cfg.setConeId(cfg.getVision().idGetter());
                    cfg.setPosition(cfg.getVision().position());
                    cfg.setRotation(cfg.getVision().rotation());

                    if (cfg.getVision().idGetter() == 440) {
                        cfg.setPhase1Check(true);
                    } else if (cfg.getVision().idGetter() == 373) {

                    } else if (cfg.getVision().idGetter() == 182) {

                    }

                    updateTele("Found ID " + cfg.getConeId(), 0);
                    break;
                }
                if (cfg.getrTime().milliseconds() > 1500) {
                    moveForwardBackward(50, rfdPID, lfdPID, lbdPID, rbdPID, 10, 0.2);
                    dontMove(50);
                    cfg.setCamCounter(cfg.getCamCounter()+1);
                } else {
                    sleep(50);
                }
            }
            if (cfg.getCamCounter() > 0) {
                moveForwardBackward(cfg.getCamCounter() * -50, rfdPID, lfdPID, lbdPID, rbdPID, 10, 0.4);
            }
            /* FULL AUTON
            if (cfg.isPhase1Check() && !cfg.isHappenedCycle()) {
                boolean finished = moveForwardBackward(200, rfdPID, lfdPID, lbdPID, rbdPID, 10, 0.5);
                telemetry.addLine("Current phase: 1");
                if (finished) {
                 cfg.setPhase1Check(false);
                 cfg.setPhase2Check(true);
                }
                cfg.setHappenedCycle(true);
            }

            if (cfg.isPhase2Check() && !cfg.isHappenedCycle()) {
                boolean finished = strafeLeft(800, rfdPID, lfdPID, lbdPID, rbdPID, 10, 0.5);
                telemetry.addLine("Current phase: 2");
                if (finished) {
                    cfg.setPhase2Check(false);
                    cfg.setPhase3Check(true);
                }
                cfg.setHappenedCycle(true);
            }

            if (cfg.isPhase3Check() && !cfg.isHappenedCycle()) {
                boolean finished = moveForwardBackward(300, rfdPID, lfdPID, lbdPID, rbdPID, 10, 0.5);
                telemetry.addLine("Current phase: 3");
                if (finished) {
                    cfg.setPhase3Check(false);
                    cfg.setPhase4Check(true);
                }
                cfg.setHappenedCycle(true);
            }

            if (cfg.isPhase4Check() && !cfg.isHappenedCycle()) {
                boolean finished = turnRight(100, rfdPID, lfdPID, lbdPID, rbdPID, 10, 0.5);
                telemetry.addLine("Current phase: 4");
                if (finished) {
                    cfg.setPhase4Check(false);
                    cfg.setPhase5Check(true);
                }
                cfg.setHappenedCycle(true);
            }

            if (cfg.isPhase5Check() && !cfg.isHappenedCycle()) {
                // ARM ACTION
                boolean finished = strafeLeft(300, rfdPID, lfdPID, lbdPID, rbdPID, 10, 0.5);
                telemetry.addLine("Current phase: 5");
                if (finished) {
                    cfg.setPhase5Check(false);
                    cfg.setPhase6Check(true);
                }
                cfg.setHappenedCycle(true);
            } */

            // ID AUTON ONLY

            // MOVE FROM WALL
            moveForwardBackward(200, rfdPID, lfdPID, lbdPID, rbdPID, 10, 0.5);

            // ID 1
            if (cfg.isPhase1Check() && !cfg.isHappenedCycle()) {
                boolean finished = strafeLeft(800, rfdPID, lfdPID, lbdPID, rbdPID, 10, 0.5);
                telemetry.addLine("Current phase: 1");
                if (finished) {
                    cfg.setPhase1Check(false);
                    cfg.setPhase2Check(true);
                }
                cfg.setHappenedCycle(true);
            }

            if (cfg.isPhase2Check() && !cfg.isHappenedCycle()) {
                boolean finished = moveForwardBackward(300, rfdPID, lfdPID, lbdPID, rbdPID, 10, 0.5);
                telemetry.addLine("Current phase: 2");
                if (finished) {
                    cfg.setPhase2Check(false);
                }
                cfg.setHappenedCycle(true);
            }

            // ID 2
            if (cfg.isPhase3Check() && !cfg.isHappenedCycle()) {
                boolean finished = moveForwardBackward(500, rfdPID, lfdPID, lbdPID, rbdPID, 10, 0.5);
                telemetry.addLine("Current phase: 3");
                if (finished) {
                    cfg.setPhase3Check(false);
                }
                cfg.setHappenedCycle(true);
            }

            // ID 3
            if (cfg.isPhase4Check() && !cfg.isHappenedCycle()) {
                boolean finished = strafeRight(800, rfdPID, lfdPID, lbdPID, rbdPID, 10, 0.5);
                telemetry.addLine("Current phase: 4");
                if (finished) {
                    cfg.setPhase4Check(false);
                    cfg.setPhase5Check(true);
                }
                cfg.setHappenedCycle(true);
            }

            if (cfg.isPhase5Check() && !cfg.isHappenedCycle()) {
                boolean finished = moveForwardBackward(300, rfdPID, lfdPID, lbdPID, rbdPID, 10, 0.5);
                telemetry.addLine("Current phase: 5");
                if (finished) {
                    cfg.setPhase5Check(false);
                }
                cfg.setHappenedCycle(true);
            }
        }
    }

    /* Driving handler
     * Driving motor - 312rpm = 537.7 pulses
     */


    // Reps = Repetitions | Power1 - Starting power | Power2 - End Power | IF POWER1 HIGHER THEN DESCENDING
    /*public double gradualPower(int reps, double power1, double power2) {
        if (power1 < power2) {
            return (power2 - power1) / reps;
        } else if (power1 > power2) {
            return (power1 - power2) / reps;
        }
        return power1;
    }
    NEXT COMPETITION
    */


    /*
    public boolean moveForwardBackward(int position, int reps, PID rfd, PID lfd, PID lbd, PID rbd, int deadZone) { // 1 sec is about 9ft
        telemetry.addData("Motor positions (RFD, LFD, RBD, LBD)", "%d,%d,%d,%d", cfg.getRfD().getCurrentPosition(),cfg.getLfD().getCurrentPosition(), cfg.getRbD().getCurrentPosition(), cfg.getLbD().getCurrentPosition());
        telemetry.addData("Motor target positions (RFD, LFD, RBD, LBD)", "%d,%d,%d,%d", cfg.getTargetPosRFD(),cfg.getTargetPosLFD(), cfg.getTargetPosRBD(), cfg.getTargetPosLBD());
        telemetry.addData("PID powers (RFD, LFD, LBD, RBD)", "%4.2f,%4.2f,%4.2f,%4.2f", rfd.getOutputFromError(cfg.getTargetPosRFD(), cfg.getRfD().getCurrentPosition()),lfd.getOutputFromError(cfg.getTargetPosLFD(), cfg.getLfD().getCurrentPosition()), lbd.getOutputFromError(cfg.getTargetPosLBD(), cfg.getLbD().getCurrentPosition()),rbd.getOutputFromError(cfg.getTargetPosRBD(), cfg.getRbD().getCurrentPosition()));

        telemetry.addData("Boolean", "%b, %b", cfg.isAtPos(), cfg.isSetStartPosForwardBackward());
        telemetry.addData("I dont even know at this point", "%b", isAtPos(cfg.getRfD().getCurrentPosition(), cfg.getRfdTargetPos(), deadZone));
        updateTele("Going forward to " + position + " pulses", 0);
        if (!cfg.isSetStartPosForwardBackward()) {
            cfg.setCurrentRep(reps);
            cfg.setAtPos(false);

            cfg.setTargetPosRFD(cfg.getRfD().getCurrentPosition() + position);
            cfg.setTargetPosLFD(cfg.getLfD().getCurrentPosition() + position);
            cfg.setTargetPosRBD(cfg.getRbD().getCurrentPosition() + position);
            cfg.setTargetPosLBD(cfg.getLbD().getCurrentPosition() + position);

            cfg.setSetStartPosForwardBackward(true);

        }
        if (!cfg.isAtPos()) {
            cfg.getRfD().setPower(rfd.getOutputFromError(cfg.getTargetPosRFD(), cfg.getRfD().getCurrentPosition()) * .5);

            cfg.getLfD().setPower(lfd.getOutputFromError(cfg.getTargetPosLFD(), cfg.getLfD().getCurrentPosition()) * .5);

            cfg.getLbD().setPower(lbd.getOutputFromError(cfg.getTargetPosLBD(), cfg.getLbD().getCurrentPosition()) * .5);

            cfg.getRbD().setPower(rbd.getOutputFromError(cfg.getTargetPosRBD(), cfg.getRbD().getCurrentPosition()) * .5);

            if (cfg.getCurrentRep() > 1) {
                cfg.setCurrentRep(cfg.getCurrentRep() - 1);
            }
            if (isAtPos(cfg.getRfD().getCurrentPosition(), cfg.getRfdTargetPos(), deadZone)) {

                cfg.getRfD().setPower(0);
                cfg.getLfD().setPower(0);
                cfg.getLbD().setPower(0);
                cfg.getRbD().setPower(0);

                cfg.setSetStartPosForwardBackward(false);

                cfg.setAtPos(true);
                return true;

            }
        }
        return false;
    }*/

    public boolean moveForwardBackward(int position, PID rfd, PID lfd, PID lbd, PID rbd, int deadZone, double speed) { // 1 sec is about 9ft
        telemetry.addData("Motor positions (RFD, LFD, RBD, LBD)", "%d,%d,%d,%d", cfg.getRfD().getCurrentPosition(),cfg.getLfD().getCurrentPosition(), cfg.getRbD().getCurrentPosition(), cfg.getLbD().getCurrentPosition());

        updateTele("Going forward to " + position + " pulses", 0);
        if (!cfg.isSetStartPosForwardBackward()) {
            lfdTarget = cfg.getLfD().getCurrentPosition() + position;
            rfdTarget = cfg.getRfD().getCurrentPosition() + position;
            lbdTarget = cfg.getLbD().getCurrentPosition() + position;
            rbdTarget = cfg.getRbD().getCurrentPosition() + position;

            cfg.setAtPos(false);
            cfg.setSetStartPosForwardBackward(true);

        }
        if (!cfg.isAtPos()) {
            cfg.getRfD().setPower(rfd.getOutputFromError(rfdTarget, cfg.getRfD().getCurrentPosition()) * speed);
            cfg.getLfD().setPower(lfd.getOutputFromError(lfdTarget, cfg.getLfD().getCurrentPosition()) * speed);
            cfg.getLbD().setPower(lbd.getOutputFromError(lbdTarget, cfg.getLbD().getCurrentPosition()) * speed);
            cfg.getRbD().setPower(rbd.getOutputFromError(rbdTarget, cfg.getRbD().getCurrentPosition()) * speed);

            if (isAtPos(cfg.getRfD().getCurrentPosition(), rfdTarget, deadZone) && isAtPos(cfg.getLfD().getCurrentPosition(), lfdTarget, deadZone) && isAtPos(cfg.getLbD().getCurrentPosition(), lbdTarget, deadZone) && isAtPos(cfg.getRbD().getCurrentPosition(), rbdTarget, deadZone)) {

                cfg.getRfD().setPower(0);
                cfg.getLfD().setPower(0);
                cfg.getLbD().setPower(0);
                cfg.getRbD().setPower(0);

                cfg.setSetStartPosForwardBackward(false);
                cfg.setAtPos(true);
                return true;

            }
        }
        return false;
    }

    public boolean strafeLeft(int position, PID rfd, PID lfd, PID lbd, PID rbd, int deadZone, double speed) { // 1 sec is about 9ft
        telemetry.addData("Motor positions (RFD, LFD, RBD, LBD)", "%d,%d,%d,%d", cfg.getRfD().getCurrentPosition(),cfg.getLfD().getCurrentPosition(), cfg.getRbD().getCurrentPosition(), cfg.getLbD().getCurrentPosition());
        updateTele("Strafing left to " + position + " pulses", 0);
        if (!cfg.isInitStrafeLeft()) {

            lfdTarget = cfg.getLfD().getCurrentPosition() - position;
            rfdTarget = cfg.getRfD().getCurrentPosition() + position;
            lbdTarget = cfg.getLbD().getCurrentPosition() - position;
            rbdTarget = cfg.getRbD().getCurrentPosition() + position;

            cfg.setAtPos(false);
            cfg.setInitStrafeLeft(true);
        }
        if (!cfg.isAtPos()) {
            cfg.getRfD().setPower(rfd.getOutputFromError(rfdTarget, cfg.getRfD().getCurrentPosition()) * speed);
            cfg.getLfD().setPower(lfd.getOutputFromError(lfdTarget, cfg.getLfD().getCurrentPosition()) * -speed);
            cfg.getLbD().setPower(lbd.getOutputFromError(lbdTarget, cfg.getLbD().getCurrentPosition()) * speed);
            cfg.getRbD().setPower(rbd.getOutputFromError(rbdTarget, cfg.getRbD().getCurrentPosition()) * -speed);

            if (isAtPos(cfg.getRfD().getCurrentPosition(), rfdTarget, deadZone) && isAtPos(cfg.getLfD().getCurrentPosition(), lfdTarget, deadZone) && isAtPos(cfg.getLbD().getCurrentPosition(), lbdTarget, deadZone) && isAtPos(cfg.getRbD().getCurrentPosition(), rbdTarget, deadZone)) {
                cfg.getRfD().setPower(0);
                cfg.getLfD().setPower(0);
                cfg.getLbD().setPower(0);
                cfg.getRbD().setPower(0);

                cfg.setInitStrafeLeft(true); //(false);
                cfg.setAtPos(true);
                return true;

            }
        }
        return false;
    }

    public boolean strafeRight(int position, PID rfd, PID lfd, PID lbd, PID rbd, int deadZone, double speed) { // 1 sec is about 9ft
        telemetry.addData("Motor positions (RFD, LFD, RBD, LBD)", "%d,%d,%d,%d", cfg.getRfD().getCurrentPosition(),cfg.getLfD().getCurrentPosition(), cfg.getRbD().getCurrentPosition(), cfg.getLbD().getCurrentPosition());
        updateTele("Strafing right to " + position + " pulses", 0);
        if (!cfg.isInitStrafeRight()) {

            lfdTarget = cfg.getLfD().getCurrentPosition() + position;
            rfdTarget = cfg.getRfD().getCurrentPosition() - position;
            lbdTarget = cfg.getLbD().getCurrentPosition() - position;
            rbdTarget = cfg.getRbD().getCurrentPosition() + position;

            cfg.setAtPos(false);
            cfg.setInitStrafeRight(true);
        }
        if (!cfg.isAtPos()) {
            cfg.getRfD().setPower(rfd.getOutputFromError(rfdTarget, cfg.getRfD().getCurrentPosition()) * -speed);
            cfg.getLfD().setPower(lfd.getOutputFromError(lfdTarget, cfg.getLfD().getCurrentPosition()) * speed);
            cfg.getLbD().setPower(lbd.getOutputFromError(lbdTarget, cfg.getLbD().getCurrentPosition()) * -speed);
            cfg.getRbD().setPower(rbd.getOutputFromError(rbdTarget, cfg.getRbD().getCurrentPosition()) * speed);

            if (isAtPos(cfg.getRfD().getCurrentPosition(), rfdTarget, deadZone) && isAtPos(cfg.getLfD().getCurrentPosition(), lfdTarget, deadZone) && isAtPos(cfg.getLbD().getCurrentPosition(), lbdTarget, deadZone) && isAtPos(cfg.getRbD().getCurrentPosition(), rbdTarget, deadZone)) {
                cfg.getRfD().setPower(0);
                cfg.getLfD().setPower(0);
                cfg.getLbD().setPower(0);
                cfg.getRbD().setPower(0);

                cfg.setInitStrafeRight(false);
                cfg.setAtPos(true);
                return true;

            }
        }
        return false;
    }

    public boolean turnleft(int position, PID rfd, PID lfd, PID lbd, PID rbd, int deadZone, double speed) { // 1 sec is about 9ft

        telemetry.addData("Motor positions (RFD, LFD, RBD, LBD)", "%d,%d,%d,%d", cfg.getRfD().getCurrentPosition(),cfg.getLfD().getCurrentPosition(), cfg.getRbD().getCurrentPosition(), cfg.getLbD().getCurrentPosition());
        updateTele("Going left to " + position + " pulses", 0);
        if (!cfg.isInitTurnLeft()) {

            rfdTarget = cfg.getRfD().getCurrentPosition() + position;
            rbdTarget = cfg.getRbD().getCurrentPosition() + position;

            cfg.setAtPos(false);
            cfg.setInitTurnLeft(true);

        }
        if (!cfg.isAtPos()) {
            cfg.getRfD().setPower(rfd.getOutputFromError(rfdTarget, cfg.getRfD().getCurrentPosition()) * speed);
            cfg.getRbD().setPower(rbd.getOutputFromError(rbdTarget, cfg.getRbD().getCurrentPosition()) * speed);

            if (isAtPos(cfg.getRfD().getCurrentPosition(), rfdTarget, deadZone) && isAtPos(cfg.getRbD().getCurrentPosition(), rbdTarget, deadZone)) {

                cfg.getRfD().setPower(0);
                cfg.getLfD().setPower(0);
                cfg.getLbD().setPower(0);
                cfg.getRbD().setPower(0);

                cfg.setInitTurnLeft(false);
                cfg.setAtPos(true);
                return true;

            }
        }
        return false;
    }

    public boolean turnRight(int position, PID rfd, PID lfd, PID lbd, PID rbd, int deadZone, double speed) { // 1 sec is about 9ft
        telemetry.addData("Motor positions (RFD, LFD, RBD, LBD)", "%d,%d,%d,%d", cfg.getRfD().getCurrentPosition(),cfg.getLfD().getCurrentPosition(), cfg.getRbD().getCurrentPosition(), cfg.getLbD().getCurrentPosition());

        updateTele("Turning right to " + position + " pulses", 0);
        if (!cfg.isInitTurnRight()) {

            lfdTarget = cfg.getLfD().getCurrentPosition() + position;
            lbdTarget = cfg.getLbD().getCurrentPosition() + position;

            cfg.setAtPos(false);
            cfg.setInitTurnRight(true);

        }
        if (!cfg.isAtPos()) {
            cfg.getLfD().setPower(lfd.getOutputFromError(lfdTarget, cfg.getLfD().getCurrentPosition()) * speed);
            cfg.getLbD().setPower(lbd.getOutputFromError(lbdTarget, cfg.getLbD().getCurrentPosition()) * speed);

            if (isAtPos(cfg.getLfD().getCurrentPosition(), lfdTarget, deadZone) && isAtPos(cfg.getLbD().getCurrentPosition(), lbdTarget, deadZone)) {

                cfg.getRfD().setPower(0);
                cfg.getLfD().setPower(0);
                cfg.getLbD().setPower(0);
                cfg.getRbD().setPower(0);

                cfg.setInitTurnRight(false);
                cfg.setAtPos(true);
                return true;

            }
        }
        return false;
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

    public void strafeLeft(double power, int time) {
        updateTele("Strafing left with power " + power + " for " + time + " ms.", 0);
        cfg.getRfD().setPower(power);
        cfg.getLfD().setPower(power * -1);
        cfg.getLbD().setPower(power);
        cfg.getRbD().setPower(power * -1);
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

    public void setArmMotorPos(int position) { // re-do
        updateTele("Arm turning to " + position, 0);
        cfg.setArmTargetPos(position);
    }

    public void setElbowMotorPos(int position) { // re-do
        updateTele("Elbow turning to " + position, 0);
        cfg.setElbowTargetPos(position);
    }

    public void waitForArmMotor() {
        int deathZone = 10;

    }

    public void waitForElbowMotor() {
        int deathZone = 10;
    }

    // advanced drive

    public void moveToPole(int team, int pos) { // A2:F2 - Pos 0 | A5:F5 - Pos 1
        //goForward(1,100);
        if (team == 0) { // BLUE CODE
            if (pos == 0) {
                strafeLeft(1, 111);
                //goForward(1, 222);
                turnRight(.2, 200);
            } else if (pos == 1) {
                strafeRight(1, 111);
                //goForward(1, 222);
                turnLeft(.2, 200);
            }
        } else if (team == 1) { // RED CODE
            if (pos == 0) {
                strafeRight(1, 400);
                //goForward(1, 500);
                turnLeft(.2, 200);
            } else if (pos == 1) {
                strafeLeft(1, 111);
                //goForward(1, 222);
                turnRight(.2, 200);
            }
        }
    }

    public void moveToSpawn(int team, int pos) {
        if (team == 0) { // BLUE CODE
            if (pos == 0) {
                turnLeft(.2, 200);
                //goBackward(1,222);
                strafeRight(1, 111);
            } else if (pos == 1) {
                turnRight(.2, 200);
                //goBackward(1,222);
                strafeLeft(1, 111);

            }
        } else if (team == 1) { // RED CODE
            if (pos == 0) {
                turnRight(.2, 200);
                //goBackward(1,222);
                strafeLeft(1, 111);
            } else if (pos == 1) {
                turnLeft(.2, 200);
                //goBackward(1,222);
                strafeRight(1, 111);
            }
        }
    }

    // cones

    // WRTIE THIS

    public void grabCone() {
        // open servo (claw)
        cfg.getA1().setPosition(.53);
        // turn motors to pickup zone
        // close servo (claw)
        cfg.getA1().setPosition(.12);


    }

    //    Arm1 - Grab servo
//    Arm2 - 180 degrees turn
//    Arm3 - Pivot
    // WRITE THIS
    public void placeCone(int level) { // ground = 0, small = 1, medium = 2, high = 3
        if (level == 0) { // GROUND
            // write MOTOR code in pseudocode aka comments
            // move motors above pole
            // rotate claw 180 degrees
            // open claw
            // repeat for all levels

            setArmMotorPos(713);
            setElbowMotorPos(69);
            cfg.getA2().setPosition(1);
            waitForElbowMotor();
            waitForArmMotor();
            cfg.getA1().setPosition(.53);


        } else if (level == 1) { // SMALL
            setArmMotorPos(713);
            setElbowMotorPos(69);
            cfg.getA2().setPosition(1);
            waitForElbowMotor();
            waitForArmMotor();
            cfg.getA1().setPosition(.53);

        } else if (level == 2) { // MEDIUM
            setArmMotorPos(713);
            setElbowMotorPos(69);
            cfg.getA2().setPosition(1);
            waitForElbowMotor();
            waitForArmMotor();
            cfg.getA1().setPosition(.53);

        } else if (level == 3) { // HIGH
            setArmMotorPos(420);
            setElbowMotorPos(69);
            cfg.getA2().setPosition(1);
            waitForElbowMotor();
            waitForArmMotor();
            cfg.getA1().setPosition(.53);

        }
    }

    public void conePhase(int lvl, int timeBeforeEnd) {
        while (30000 - cfg.getrTime().milliseconds() < timeBeforeEnd) {
            placeCone(lvl);
            grabCone();
        }
    }

    // vision

    // WRITE THIS
    public void goToAfterId(int id) {
        if (id == 440) { // Number 1
            // move to sector 1/2/3 depending on the id
            // JUST USE BASIC MOVE FUNCTIONS DOCS ARE ONLINE JUST IN CASE
            strafeLeft(1, 1000);
            // goForward(1, 1000);


        } else if (id == 373) { // Number 2

            strafeLeft(1, 1000);
            //goForward(1,2000);
            //goBackward(1,1000);

        } else if (id == 182) { // Number 3
            strafeLeft(1, 1000);
            //goForward(1,3000);
            strafeRight(1, 3000);
            //goBackward(1,1000);


        }

    }

    public boolean isAtPos(int currentPos, int targetPos, int deathZone) {
        int rnp = targetPos - deathZone;
        int rpp = targetPos + deathZone;
        if (rnp > currentPos || rpp < currentPos) {
            return true;
        }
        return false;
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
