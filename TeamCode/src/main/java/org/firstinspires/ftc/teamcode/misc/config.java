package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vision.AprilTagDemo;
import org.openftc.easyopencv.OpenCvCamera;

public class config {
    int teamColor = 1; // 0 - Blue | 1 - Red

    // Motors (left front drive, left back drive, right front drive, right back drive)
    DcMotor lfD = null;
    DcMotor lbD = null;
    DcMotor rfD = null;
    DcMotor rbD = null;

    DcMotor slide1Motor = null;
    DcMotor slide2Motor = null;

    // Runtime
    ElapsedTime rTime = new ElapsedTime();

    // Speed
    double speedMultiplier = 1.0;


    int slide1Position = 0;
    int slide2Position = 0;

    // Claw (A1 - Claw, A2 - 180 turn, A3 - Pivot turn)
    Servo clawServo;
    Servo rotateServo;
    Servo pivotServo;

    OpenCvCamera camera;

    AprilTagDemo vision = new AprilTagDemo();
    int coneId = 0;


    // Servos Misc
    final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle

    // targetPos
    int slide1MotorTargetPosition = 0;
    int slide2MotorTargetPosition = 0;

    int lfdTargetPos = 0;
    int lbdTargetPos = 0;
    int rfdTargetPos = 0;
    int rbdTargetPos = 0;

    // SETTERS AND GETTERS


    public int getTeamColor() {
        return teamColor;
    }

    public void setTeamColor(int teamColor) {
        this.teamColor = teamColor;
    }

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

    public DcMotor getSlide1Motor() {
        return slide1Motor;
    }

    public void setSlide1Motor(DcMotor slide1Motor) {
        this.slide1Motor = slide1Motor;
    }

    public DcMotor getSlide2Motor() {
        return slide2Motor;
    }

    public void setSlide2Motor(DcMotor slide2Motor) {
        this.slide2Motor = slide2Motor;
    }

    public ElapsedTime getrTime() {
        return rTime;
    }

    public void setrTime(ElapsedTime rTime) {
        this.rTime = rTime;
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    public void setSpeedMultiplier(double speedMultiplier) {
        this.speedMultiplier = speedMultiplier;
    }

    public int getSlide1Position() {
        return slide1Position;
    }

    public void setSlide1Position(int slide1Position) {
        this.slide1Position = slide1Position;
    }

    public int getSlide2Position() {
        return slide2Position;
    }

    public void setSlide2Position(int slide2Position) {
        this.slide2Position = slide2Position;
    }

    public Servo getClawServo() {
        return clawServo;
    }

    public void setClawServo(Servo clawServo) {
        this.clawServo = clawServo;
    }

    public Servo getRotateServo() {
        return rotateServo;
    }

    public void setRotateServo(Servo rotateServo) {
        this.rotateServo = rotateServo;
    }

    public Servo getPivotServo() {
        return pivotServo;
    }

    public void setPivotServo(Servo pivotServo) {
        this.pivotServo = pivotServo;
    }

    public OpenCvCamera getCamera() {
        return camera;
    }

    public void setCamera(OpenCvCamera camera) {
        this.camera = camera;
    }

    public AprilTagDemo getVision() {
        return vision;
    }

    public int getConeId() {
        return coneId;
    }

    public void setConeId(int coneId) {
        this.coneId = coneId;
    }

    public double getINCREMENT() {
        return INCREMENT;
    }

    public int getSlide1MotorTargetPosition() {
        return slide1MotorTargetPosition;
    }

    public void setSlide1MotorTargetPosition(int slide1MotorTargetPosition) {
        this.slide1MotorTargetPosition = slide1MotorTargetPosition;
    }

    public int getSlide2MotorTargetPosition() {
        return slide2MotorTargetPosition;
    }

    public void setSlide2MotorTargetPosition(int slide2MotorTargetPosition) {
        this.slide2MotorTargetPosition = slide2MotorTargetPosition;
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
}
