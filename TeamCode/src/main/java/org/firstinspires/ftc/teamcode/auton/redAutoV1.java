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
    public boolean happenedCycle = false;

    public boolean phase1Check = false; // WHEN USING FULL STATE CHANGE TO TRUE
    public boolean phase2Check = false;
    public boolean phase3Check = false;
    public boolean phase4Check = false;
    public boolean phase5Check = false;
    boolean phase6Check = false;
    boolean phase7Check = false;
    boolean phase8Check = false;
    boolean phase9Check = false;
    boolean phase10Check = false;

    int lfdTarget = 0;
    int rfdTarget = 0;
    int lbdTarget = 0;
    int rbdTarget = 0;
    int firstRun = 1;
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
        while (!opModeIsActive() && !isStopRequested()) {
            //loop detection here
            cfg.getVision().updateTags(cfg);

            if (cfg.getVision().idGetter() != 0) {
                // ID Found
                int cConeId = cfg.getVision().idGetter();
                cfg.setConeId(cfg.getVision().idGetter());
                cfg.setPosition(cfg.getVision().position());
                cfg.setRotation(cfg.getVision().rotation());
                telemetry.addData("Cone id", "%d", cConeId);
                telemetry.update();
            }
        }
        waitForStart();
        cfg.getrTime().reset();

        while (opModeIsActive()) {
            //
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
