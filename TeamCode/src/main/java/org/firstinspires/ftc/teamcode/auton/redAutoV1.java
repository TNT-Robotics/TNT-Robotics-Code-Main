package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.misc.autonInit;
import org.firstinspires.ftc.teamcode.misc.autonFunc;
import org.firstinspires.ftc.teamcode.misc.config;


@Autonomous
public class redAutoV1 extends LinearOpMode {

    autonInit aiInit = new autonInit();
    config cfg = new config();
    autonFunc aiFunc = new autonFunc();

    @Override
    public void runOpMode() {
        aiInit.initAuton(hardwareMap, 1);
        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        cfg.getrTime().reset();

        // write test code here

        // if nothing detected for 1.5s move a bit forward until 5s then stop searching
        // for cone id
        while (cfg.getrTime().milliseconds() < 5000) {
            cfg.getVision().updateTags(cfg.getCamera());

            if (cfg.getVision().idGetter() != 0) {
                // ID Found
                cfg.setConeId(cfg.getVision().idGetter());
                cfg.setPosition(cfg.getVision().position());
                cfg.setRotation(cfg.getVision().rotation());

                updateTele("Found ID " + cfg.getConeId(), 0);
            }
            if (cfg.getrTime().milliseconds() > 1500) {
                aiFunc.goForward(.2, 100);
                aiFunc.dontMove(50);
                cfg.setCamCounter(cfg.getCamCounter()+1);
            } else {
                sleep(50);
            }
        }
        if (cfg.getCamCounter() > 0) {
            aiFunc.goBackward(1, (cfg.getCamCounter() * 100) / 5);
        }

        // grab cones
        for (int counter = 0; counter < 3; counter++) {
            aiFunc.moveToPole(cfg.getTeamColor());
            aiFunc.dontMove(200);
            aiFunc.placeCone(2);
            aiFunc.moveToCone(cfg.getTeamColor());
            aiFunc.grabCone();
        }

        aiFunc.goToAfterId(cfg.getConeId(), cfg.getPosition(), cfg.getRotation(), cfg.getTeamColor());

        /*
         * while (opModeIsActive()) {
         * }
         */
    }

    // statusNum > 0 = Nominal | 1 = Warning | 2 = Minor Error | 3 = Fatal Error
    public void updateTele(String action, int statusNum) {
        telemetry.addData("Status", statusNum);
        telemetry.addData("Action", action);
        telemetry.addData("Running for", cfg.getrTime().toString());
        telemetry.update();
    }

    public void slp(int time) {
        sleep(time);
    }
}
