package org.firstinspires.ftc.teamcode.opmodes.test

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.config.Field
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.lib.Vision

@Autonomous(name = "Vision Test", group = "FTC24")
class Vision : LinearOpMode() {
    val vision = Vision()

    override fun runOpMode() {
        Robot.initHardware(this.hardwareMap)
        vision.init(
                Field.lensInstrinsics,
                Field.decimation,
                Robot.camera,
                Field.fieldTags
        )
        waitForStart()

        while (opModeIsActive()) {
            val detections = vision.detections

            for(tag in detections) telemetry.addData("Detected tag", tag.id)

            telemetry.addData("Tag relative position", vision.tagRelativePosition)
            telemetry.addData("Field relative position", vision.fieldRelativePosition)
            telemetry.addData("Robot angle", vision.heading)

            telemetry.update()
        }
    }
}