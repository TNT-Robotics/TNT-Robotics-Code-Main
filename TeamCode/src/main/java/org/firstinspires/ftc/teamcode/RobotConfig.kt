package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.*

/**
 * This code has been modified by FTC team 12051 NotNotNerds**************************************
 * We do not guarantee that your robot will function correctly after you have used this code**********************
 * It is not recommended that you use our modifications with your robot********************************
 * Thank you to those teams who willingly contributed their github code so that I could create this file; I wouldn't have been able to make this otherwise
 */
class RobotConfig(aHardwareMap: HardwareMap?) {
    var fl: DcMotorEx
    var fr: DcMotorEx
    var bl: DcMotorEx
    var br: DcMotorEx

    private var hardwareMap: HardwareMap? = null //don't know why it needs to be null, but ok
    fun drive(fls: Double, frs: Double, brs: Double, bls: Double) {
        fl.power = fls
        fr.power = frs
        br.power = brs
        bl.power = bls
    }

    fun stopMoving() {
        fl.power = 0.0
        fr.power = 0.0
        br.power = 0.0
        bl.power = 0.0
    }

    init {
        hardwareMap = aHardwareMap
        /***
         * config=actual=ch port (actual locations missing currently)
         * fr=fr=0
         * bl=bl=1
         * fl=fl=2
         * br=br=3
         */
        fl = hardwareMap!!.get(DcMotorEx::class.java, "fl")
        bl = hardwareMap!!.get(DcMotorEx::class.java, "bl")
        fr = hardwareMap!!.get(DcMotorEx::class.java, "fr")
        br = hardwareMap!!.get(DcMotorEx::class.java, "br")
        fl.direction = DcMotorSimple.Direction.FORWARD
        bl.direction = DcMotorSimple.Direction.REVERSE
        fr.direction = DcMotorSimple.Direction.FORWARD
        br.direction = DcMotorSimple.Direction.REVERSE

        fl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        bl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        fr.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        br.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
}