package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vision.AprilTagDemo;
import org.openftc.easyopencv.OpenCvCamera;

public class config {
    int teamColor = 1; // 0 - Blue | 1 - Red

    // Motors
    DcMotor lfD;
    DcMotor lbD;
    DcMotor rfD;
    DcMotor rbD;

    DcMotor arm;
    DcMotor elbow;

    // Runtime
    ElapsedTime rTime;

    // Speed
    double spdMult;
    double armElbowSpdMult;


    int armPos;
    int elbowPos;

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
}
