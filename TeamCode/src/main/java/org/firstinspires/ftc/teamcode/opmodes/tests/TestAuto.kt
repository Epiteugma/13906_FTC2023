package org.firstinspires.ftc.teamcode.opmodes.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.AutonomousBase
import org.firstinspires.ftc.teamcode.Color
import org.firstinspires.ftc.teamcode.Side
import org.firstinspires.ftc.teamcode.lib.vision.detection.ItemDetector

@Autonomous(name = "TEST AUTONOMOUS", group = "FTC23")
class TestAuto: AutonomousBase(Color.RED, Side.RIGHT) {
    override fun run() {
        super.run()

        this.requestStop()
    }

}