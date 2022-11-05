package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vision.AprilTagDemo;
import org.openftc.easyopencv.OpenCvCamera;

public class config {
    int teamColor = 1; // 0 - Blue | 1 - Red
    // Phases
    boolean phase1Check = true;
    boolean phase2Check = false;
    boolean phase3Check = false;
    boolean phase4Check = false;
    boolean phase5Check = false;
    boolean phase6Check = false;
    boolean phase7Check = false;
    boolean phase8Check = false;
    boolean phase9Check = false;
    boolean phase10Check = false;

    boolean happenedCycle = false;

    boolean setStartPosForwardBackward = false;
    boolean setStartPosTurn = false;
    boolean setStartPosStrafe = false;

    boolean atPos = false;
    boolean goUp = false;
    int currentRep = 0;

    int targetPosRFD = 0;
    int targetPosLFD = 0;
    int targetPosLBD = 0;
    int targetPosRBD = 0;



    // Motors
    DcMotor lfD = null;
    DcMotor lbD = null;
    DcMotor rfD = null;
    DcMotor rbD = null;

    DcMotor arm = null;
    DcMotor elbow = null;

    // Runtime
    ElapsedTime rTime = new ElapsedTime();

    // Speed
    double spdMult = 1.0;
    double armElbowSpdMult = 1.0;


    int armPos = 0;
    int elbowPos = 0;

    // Claw
    Servo a1;
    Servo a2;
    Servo a3;

    // Vision
    int camCounter = 0;
    OpenCvCamera camera;

    AprilTagDemo vision = new AprilTagDemo();
    int coneId = 0;

    double[] position = { 0, 0, 0 }; // X,Y,Z
    double[] rotation = { 0, 0, 0 }; // YAW, PITCH, ROLL

    // Servos Misc
    final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    final double MAX_POS     =  1;     // Maximum rotational position
    final double MIN_POS     =  0;     // Minimum rotational position

    // targetPos
    int armTargetPos = 0;
    int elbowTargetPos = 0;

    int lfdTargetPos = 0;
    int lbdTargetPos = 0;
    int rfdTargetPos = 0;
    int rbdTargetPos = 0;

    // SETTERS AND GETTERS


    public DcMotor getLfD() {
        return lfD;
    }

    public void setLfD(DcMotor lfD) {
        this.lfD = lfD;
    }

    public DcMotor getLbD() {
        return lbD;
    }

    public void setLbD(DcMotor lbD) {
        this.lbD = lbD;
    }

    public DcMotor getRfD() {
        return rfD;
    }

    public void setRfD(DcMotor rfD) {
        this.rfD = rfD;
    }

    public DcMotor getRbD() {
        return rbD;
    }

    public void setRbD(DcMotor rbD) {
        this.rbD = rbD;
    }

    public DcMotor getArm() {
        return arm;
    }

    public void setArm(DcMotor arm) {
        this.arm = arm;
    }

    public DcMotor getElbow() {
        return elbow;
    }

    public void setElbow(DcMotor elbow) {
        this.elbow = elbow;
    }

    public ElapsedTime getrTime() {
        return rTime;
    }

    public void setrTime(ElapsedTime rTime) {
        this.rTime = rTime;
    }

    public double getSpdMult() {
        return spdMult;
    }

    public void setSpdMult(double spdMult) {
        this.spdMult = spdMult;
    }

    public double getArmElbowSpdMult() {
        return armElbowSpdMult;
    }

    public void setArmElbowSpdMult(double armElbowSpdMult) {
        this.armElbowSpdMult = armElbowSpdMult;
    }
    public int getArmPos() {
        return armPos;
    }

    public void setArmPos(int armPos) {
        this.armPos = armPos;
    }

    public int getElbowPos() {
        return elbowPos;
    }

    public void setElbowPos(int elbowPos) {
        this.elbowPos = elbowPos;
    }

    public Servo getA1() {
        return a1;
    }

    public void setA1(Servo a1) {
        this.a1 = a1;
    }

    public Servo getA2() {
        return a2;
    }

    public void setA2(Servo a2) {
        this.a2 = a2;
    }

    public Servo getA3() {
        return a3;
    }

    public void setA3(Servo a3) {
        this.a3 = a3;
    }

    public double getINCREMENT() {
        return INCREMENT;
    }

    public double getMAX_POS() {
        return MAX_POS;
    }

    public double getMIN_POS() {
        return MIN_POS;
    }

    public int getTeamColor() {
        return teamColor;
    }

    public void setTeamColor(int teamColor) {
        this.teamColor = teamColor;
    }

    public int getCamCounter() {
        return camCounter;
    }

    public void setCamCounter(int camCounter) {
        this.camCounter = camCounter;
    }

    public OpenCvCamera getCamera() {
        return camera;
    }

    public void setCamera(OpenCvCamera camera) {
        this.camera = camera;
    }

    public int getConeId() {
        return coneId;
    }

    public void setConeId(int coneId) {
        this.coneId = coneId;
    }

    public double[] getPosition() {
        return position;
    }

    public void setPosition(double[] position) {
        this.position = position;
    }

    public double[] getRotation() {
        return rotation;
    }

    public void setRotation(double[] rotation) {
        this.rotation = rotation;
    }

    public AprilTagDemo getVision() {
        return vision;
    }

    public int getArmTargetPos() {
        return armTargetPos;
    }

    public void setArmTargetPos(int armTargetPos) {
        this.armTargetPos = armTargetPos;
    }

    public int getElbowTargetPos() {
        return elbowTargetPos;
    }

    public void setElbowTargetPos(int elbowTargetPos) {
        this.elbowTargetPos = elbowTargetPos;
    }

    public int getLfdTargetPos() {
        return lfdTargetPos;
    }

    public void setLfdTargetPos(int lfdTargetPos) {
        this.lfdTargetPos = lfdTargetPos;
    }

    public int getLbdTargetPos() {
        return lbdTargetPos;
    }

    public void setLbdTargetPos(int lbdTargetPos) {
        this.lbdTargetPos = lbdTargetPos;
    }

    public int getRfdTargetPos() {
        return rfdTargetPos;
    }

    public void setRfdTargetPos(int rfdTargetPos) {
        this.rfdTargetPos = rfdTargetPos;
    }

    public int getRbdTargetPos() {
        return rbdTargetPos;
    }

    public void setRbdTargetPos(int rbdTargetPos) {
        this.rbdTargetPos = rbdTargetPos;
    }

    public void setPhase1Check(boolean phase1Check) {
        this.phase1Check = phase1Check;
    }

    public void setPhase2Check(boolean phase2Check) {
        this.phase2Check = phase2Check;
    }

    public void setPhase3Check(boolean phase3Check) {
        this.phase3Check = phase3Check;
    }

    public void setPhase4Check(boolean phase4Check) {
        this.phase4Check = phase4Check;
    }

    public void setPhase5Check(boolean phase5Check) {
        this.phase5Check = phase5Check;
    }

    public void setPhase6Check(boolean phase6Check) {
        this.phase6Check = phase6Check;
    }

    public void setPhase7Check(boolean phase7Check) {
        this.phase7Check = phase7Check;
    }

    public void setPhase8Check(boolean phase8Check) {
        this.phase8Check = phase8Check;
    }

    public void setPhase9Check(boolean phase9Check) {
        this.phase9Check = phase9Check;
    }

    public void setPhase10Check(boolean phase10Check) {
        this.phase10Check = phase10Check;
    }

    public void setHappenedCycle(boolean happenedCycle) {
        this.happenedCycle = happenedCycle;
    }

    public boolean isPhase1Check() {
        return phase1Check;
    }

    public boolean isPhase2Check() {
        return phase2Check;
    }

    public boolean isPhase3Check() {
        return phase3Check;
    }

    public boolean isPhase4Check() {
        return phase4Check;
    }

    public boolean isPhase5Check() {
        return phase5Check;
    }

    public boolean isPhase6Check() {
        return phase6Check;
    }

    public boolean isPhase7Check() {
        return phase7Check;
    }

    public boolean isPhase8Check() {
        return phase8Check;
    }

    public boolean isPhase9Check() {
        return phase9Check;
    }

    public boolean isPhase10Check() {
        return phase10Check;
    }

    public boolean isHappenedCycle() {
        return happenedCycle;
    }

    public void setSetStartPosForwardBackward(boolean setStartPosForwardBackward) {
        this.setStartPosForwardBackward = setStartPosForwardBackward;
    }

    public void setSetStartPosTurn(boolean setStartPosTurn) {
        this.setStartPosTurn = setStartPosTurn;
    }

    public void setSetStartPosStrafe(boolean setStartPosStrafe) {
        this.setStartPosStrafe = setStartPosStrafe;
    }

    public boolean isSetStartPosForwardBackward() {
        return setStartPosForwardBackward;
    }

    public boolean isSetStartPosTurn() {
        return setStartPosTurn;
    }

    public boolean isSetStartPosStrafe() {
        return setStartPosStrafe;
    }

    public void setAtPos(boolean atPos) {
        this.atPos = atPos;
    }

    public void setGoUp(boolean goUp) {
        this.goUp = goUp;
    }

    public void setCurrentRep(int currentRep) {
        this.currentRep = currentRep;
    }

    public boolean isAtPos() {
        return atPos;
    }

    public boolean isGoUp() {
        return goUp;
    }

    public int getCurrentRep() {
        return currentRep;
    }

    public void setTargetPosRFD(int targetPosRFD) {
        this.targetPosRFD = targetPosRFD;
    }

    public void setTargetPosLFD(int targetPosLFD) {
        this.targetPosLFD = targetPosLFD;
    }

    public void setTargetPosLBD(int targetPosLBD) {
        this.targetPosLBD = targetPosLBD;
    }

    public void setTargetPosRBD(int targetPosRBD) {
        this.targetPosRBD = targetPosRBD;
    }

    public int getTargetPosRFD() {
        return targetPosRFD;
    }

    public int getTargetPosLFD() {
        return targetPosLFD;
    }

    public int getTargetPosLBD() {
        return targetPosLBD;
    }

    public int getTargetPosRBD() {
        return targetPosRBD;
    }
}
