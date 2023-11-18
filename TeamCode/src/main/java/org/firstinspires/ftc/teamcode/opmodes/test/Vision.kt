package org.firstinspires.ftc.teamcode.opmodes.test

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.config.Field
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.lib.Vision

@Autonomous(name = "Vision Test", group = "FTC24")
class Vision : LinearOpMode() {
    private val vision = Vision()

    override fun runOpMode() {
        Robot.initHardware(this.hardwareMap)
        this.telemetry = Robot.bindDashboard(this.telemetry)

        vision.init(
                Field.lensIntrinsics,
                Field.decimation,
                Robot.camera,
                Field.fieldTags
        )

        Robot.initDashboardStream(vision.instance, 30.0)
        waitForStart()

        while (opModeIsActive()) {
            val detections = vision.detections

            for(tag in detections) telemetry.addData("Detected tag", tag.id)

            telemetry.addData("Detections", detections.size)
            telemetry.addData("Tag relative position", vision.tagRelativePosition)
            telemetry.addData("Field relative position", vision.fieldRelativePosition)
            telemetry.addData("Robot angle", vision.heading)

            telemetry.update()
        }
    }
}