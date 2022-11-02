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
    @Override
    public void runOpMode() {

        autonInit aiInit = new autonInit(cfg);

        PID lfdPID = new PID(.02,.0,.02,.008);
        PID lbdPID = new PID(.02,.0,.02,.008);
        PID rfdPID = new PID(.02,.0,.02,.008);
        PID rbdPID = new PID(.02,.0,.02,.008);

        lfdPID.getOutputFromError(0,0);
        lbdPID.getOutputFromError(0,0);
        rfdPID.getOutputFromError(0,0);
        rbdPID.getOutputFromError(0,0);

        aiInit.initAuton(hardwareMap, 1);
        cfg.getVision().initCamera(cfg.getCamera());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        cfg.getrTime().reset();

        // if nothing detected for 1.5s move a bit forward until 5s then stop searching
        // for cone id
        /*
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
        moveToPole(cfg.getTeamColor(), 0);
        conePhase(3, 10000); // Time before end in milliseconds
        moveToSpawn(cfg.getTeamColor(), 0);

*/
        moveForwardBackward(50, 5, rfdPID, lfdPID, lbdPID, rbdPID);
        moveForwardBackward(-50, 5, rfdPID, lfdPID, lbdPID, rbdPID);

        /*
         * while (opModeIsActive()) {
         * }
         */
    }

    /* Driving handler
    * Driving motor - 312rpm = 537.7 pulses
    */


// Reps = Repetitions | Power1 - Starting power | Power2 - End Power | IF POWER1 HIGHER THEN DESCENDING
    public double gradualPower(int reps, double power1, double power2) {
        if (power1 < power2) {
            return (power2-power1) / reps;
        } else if (power1 > power2) {
            return (power1-power2) / reps;
        }
        return power1;
    }



    public void moveForwardBackward(int position, int reps, PID rfd, PID lfd, PID lbd, PID rbd) { // 1 sec is about 9ft
        updateTele("Going forward to " + position + " pulses", 0);
        int deadZone = 10;
        boolean atPos = false;
        boolean goUp = false;
        int currentRep = reps;

        int targetPosRFD = cfg.getRfD().getCurrentPosition() + position;
        int targetPosLFD = cfg.getLfD().getCurrentPosition() + position;
        int targetPosLBD = cfg.getLbD().getCurrentPosition() + position;
        int targetPosRBD = cfg.getRbD().getCurrentPosition() + position;


        if (cfg.getRfD().getCurrentPosition() < cfg.getRfD().getCurrentPosition() + position) {
            goUp = true;
        }
        while (atPos == false && opModeIsActive()) {
            cfg.getRfD().setPower(gradualPower(currentRep, cfg.getRfD().getPower(), rfd.getOutputFromError(targetPosRFD, cfg.getRfD().getCurrentPosition())));
            cfg.setRfdTargetPos(cfg.getRfD().getCurrentPosition() + position);

            cfg.getLfD().setPower(gradualPower(currentRep, cfg.getLfD().getPower(), lfd.getOutputFromError(targetPosLFD, cfg.getLfD().getCurrentPosition())));
            cfg.setLfdTargetPos(cfg.getLfD().getCurrentPosition() + position);

            cfg.getLbD().setPower(gradualPower(currentRep, cfg.getLbD().getPower(), lbd.getOutputFromError(targetPosLBD, cfg.getLbD().getCurrentPosition())));
            cfg.setLbdTargetPos(cfg.getLbD().getCurrentPosition() + position);

            cfg.getRbD().setPower(gradualPower(currentRep, cfg.getRbD().getPower(), rbd.getOutputFromError(targetPosRBD, cfg.getRbD().getCurrentPosition())));
            cfg.setRbdTargetPos(cfg.getRbD().getCurrentPosition() + position);

            if (currentRep > 1) {
                currentRep--;
            }

            if (goUp == true) {
                if (cfg.getRfD().getCurrentPosition() + deadZone > cfg.getRfdTargetPos() && cfg.getRfD().getCurrentPosition() - deadZone < cfg.getRfdTargetPos())  {
                    int extraCurrentReps = 4;
                    while (extraCurrentReps > 0 && opModeIsActive() ) {
                        cfg.getRfD().setPower(gradualPower(4, cfg.getRfD().getPower(), 0));
                        cfg.getLfD().setPower(gradualPower(4, cfg.getLfD().getPower(), 0));
                        cfg.getLbD().setPower(gradualPower(4, cfg.getLbD().getPower(), 0));
                        cfg.getRbD().setPower(gradualPower(4, cfg.getRbD().getPower(), 0));
                        extraCurrentReps--;
                    }
                    break;
                }
            } else {

                if (cfg.getRfD().getCurrentPosition() + deadZone < cfg.getRfdTargetPos() && cfg.getRfD().getCurrentPosition() - deadZone > cfg.getRfdTargetPos())  {
                    int extraCurrentReps = 4;
                    while (extraCurrentReps > 0 && opModeIsActive() ) {
                        cfg.getRfD().setPower(gradualPower(4, cfg.getRfD().getPower(), 0));
                        cfg.getLfD().setPower(gradualPower(4, cfg.getLfD().getPower(), 0));
                        cfg.getLbD().setPower(gradualPower(4, cfg.getLbD().getPower(), 0));
                        cfg.getRbD().setPower(gradualPower(4, cfg.getRbD().getPower(), 0));
                        extraCurrentReps--;
                    }
                    break;
                }
            }
        }
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
        while (deathZone == 10) {
            if (cfg.getArm().getCurrentPosition() < cfg.getArmTargetPos() + deathZone && cfg.getArmTargetPos() > cfg.getArmTargetPos() - deathZone)
            {
                break;
            }

        }
    }

    public void waitForElbowMotor() {
        int deathZone = 10;
        while (deathZone == 10) {
            if (cfg.getElbow().getCurrentPosition() < cfg.getElbowTargetPos() + deathZone && cfg.getElbow().getCurrentPosition() > cfg.getElbowTargetPos() - deathZone)
            {
                break;
            }

        }
    }

    // advanced drive

    public void moveToPole(int team, int pos) { // A2:F2 - Pos 0 | A5:F5 - Pos 1
        //goForward(1,100);
        if (team == 0) { // BLUE CODE
            if (pos == 0) {
                strafeLeft(1,111);
                //goForward(1, 222);
                turnRight(.2, 200);
            } else if (pos == 1) {
                strafeRight(1,111);
                //goForward(1, 222);
                turnLeft(.2, 200);
            }
        } else if (team == 1) { // RED CODE
            if (pos == 0) {
                strafeRight(1,400);
                //goForward(1, 500);
                turnLeft(.2, 200);
            } else if (pos == 1) {
                strafeLeft(1,111);
                //goForward(1, 222);
                turnRight(.2, 200);
            }
        }
    }

    public void moveToSpawn(int team, int pos) {
        if (team == 0) { // BLUE CODE
            if (pos == 0) {
                turnLeft(.2,200);
                //goBackward(1,222);
                strafeRight(1,111);
            } else if (pos == 1) {
                turnRight(.2,200);
                //goBackward(1,222);
                strafeLeft(1,111);

            }
        } else if (team == 1) { // RED CODE
            if (pos == 0) {
                turnRight(.2,200);
                //goBackward(1,222);
                strafeLeft(1,111);
            } else if (pos == 1) {
                turnLeft(.2,200);
                //goBackward(1,222);
                strafeRight(1,111);
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

            strafeLeft(1,1000);
            //goForward(1,2000);
            //goBackward(1,1000);

        } else if (id == 182) { // Number 3
            strafeLeft(1,1000);
            //goForward(1,3000);
            strafeRight(1,3000);
            //goBackward(1,1000);


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
