package org.firstinspires.ftc.teamcode

import com.qualcomm.ftccommon.SoundPlayer
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs
import kotlin.random.Random





/**
 **************************************This code has been modified by FTC team 12051 NotNotNerds*************************************
 **********************We do not guarantee that your robot will function correctly after you have used this code*********************
 ********************************It is not recommended that you use our modifications with your robot********************************
 */

@TeleOp(name="Teleop", group="TeleOp")
class OurTeleOp : LinearOpMode() {
    private var robotConfig: RobotConfig?=null


    private var hubList: List<LynxModule>?=null //init hubs to change color

    override fun runOpMode() {
        telemetry.addLine("Robot has been turned on. Run for your life!")
        telemetry.update()
        robotConfig=RobotConfig(hardwareMap)
        hubList=hardwareMap.getAll(LynxModule::class.java)
        /*** enable bulk caching */
        for (hub in hubList!!) {
            hub.bulkCachingMode=LynxModule.BulkCachingMode.MANUAL
        }
        /*** controller settings  */
        val lightRumble = 0.2
        val strongRumble = 0.5

        /*** A few more variables  */
        var m1=.5 //Speed multiplier

        while(!opModeIsActive()) { //loop after Init, before Start
            rumble1("both",1000.0, 100) //run vibrator to tell drivers that they need to start the opmode
            rumble2("both",1000.0, 100)
        }
        telemetry.addLine("Robot Is ready")
        telemetry.update()
        waitForStart()
        SoundPlayer.getInstance().stopPlayingAll()

        /*** add timers for everything that needs them */
        var lastTime=System.currentTimeMillis()

        while(opModeIsActive()) {
            /*** loop frame time system */
            val currentSystemTime=System.currentTimeMillis()
            telemetry.addData("Time between frame: ", currentSystemTime-lastTime)
            lastTime=currentSystemTime
            for (hub in hubList!!) {
                hub.clearBulkCache()
            }

            /***************** drive stuff beneath here  */
            when {
                gamepad1.b -> { //drive speed multiplier modifiers
                    m1=.6
                    rumble1("r", lightRumble, 100)
                }
                gamepad1.a -> {
                    m1=.2
                    rumble1("r", lightRumble, 100)
                }
                gamepad1.y -> {
                    m1=1.0
                    rumble1("r", lightRumble, 100)
                }
                gamepad1.x -> {
                    m1=.8
                    rumble1("r", lightRumble, 100)
                }
            }

            /******************* Warning, math ahead  */
            var drive=gamepad1.right_stick_x.toDouble()
            var strafe=-gamepad1.left_stick_x.toDouble()
            var rotate=-gamepad1.right_stick_y.toDouble()

            robotConfig!!.bl.power=m1*(drive+rotate+strafe)
            robotConfig!!.br.power=m1*(drive-rotate+strafe)
            robotConfig!!.fr.power=m1*(-drive+rotate+strafe)
            robotConfig!!.fl.power=m1*(-drive-rotate+strafe)

            /**************** Telemetry Stuff *****************/
            /* telemetry.addData("ry", gamepad1.right_stick_y); //use when motor connections/commands messed up
            telemetry.addData("rx", gamepad1.right_stick_x);
            telemetry.addData("lx", gamepad1.left_stick_x);
            telemetry.addData("fl", robotConfig.fl.getPower());
            telemetry.addData("bl", robotConfig.bl.getPower());
            telemetry.addData("fr", robotConfig.fr.getPower());
            telemetry.addData("br", robotConfig.br.getPower());
            telemetry.addData("br", robotConfig.br.getCurrentPosition());
            telemetry.addData("fr", robotConfig.fr.getCurrentPosition()); */
            telemetry.addLine("Random Stuff \n")
            telemetry.update()

        }
    }
    /*** functions used to simplify long lines of code into slightly shorter ones */
    private fun rumble1(side: String, rumblePower : Double, duration : Int){
        when {
            side.lowercase() == "l" -> {
                gamepad1.rumble(rumblePower, 0.0, duration)
            }
            side.lowercase() == "r" -> {
                gamepad1.rumble(0.0, rumblePower, duration)
            }
            side.lowercase() == "both" -> {
                gamepad1.rumble(rumblePower/2, rumblePower/2, duration)
            }
            else -> {
                telemetry.addLine("One of the rumble commands has been set up incorrectly, make sure the side is only one letter, l or r")
            }
        }
    }

    private fun rumble2(side: String,rumblePower: Double, duration : Int){
        when {
            side.lowercase() == "l" -> {
                gamepad2.rumble(rumblePower, 0.0, duration)
            }
            side.lowercase() == "r" -> {
                gamepad2.rumble(0.0, rumblePower, duration)
            }
            side.lowercase() == "both" -> {
                gamepad2.rumble(rumblePower/2, rumblePower/2, duration)
            }
            else -> {
                telemetry.addLine("One of the rumble commands has been set up incorrectly, make sure the side is only one letter, l or r")
            }
        }
    }


}