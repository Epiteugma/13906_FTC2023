package org.firstinspires.ftc.teamcode.lib

import android.graphics.Bitmap
import android.graphics.Canvas
import org.firstinspires.ftc.robotcore.external.function.Consumer
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl
import org.opencv.android.Utils
import org.opencv.core.Mat
import java.util.concurrent.atomic.AtomicReference

class AprilTagStream(fx: Double, fy: Double, cx: Double, cy: Double, outputUnitsLength:
DistanceUnit, outputUnitsAngle: AngleUnit, tagLibrary: AprilTagLibrary, drawAxes: Boolean,
                     drawCube: Boolean, drawOutline: Boolean, drawTagID: Boolean, tagFamily:
                     TagFamily, threads: Int, suppressCalibrationWarnings: Boolean = false) :
    AprilTagProcessorImpl(fx,
    fy, cx, cy,
    outputUnitsLength, outputUnitsAngle, tagLibrary, drawAxes, drawCube, drawOutline, drawTagID,
    tagFamily, threads, suppressCalibrationWarnings), CameraStreamSource {
    private val lastFrame = AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))
    private val lastDrawnFrame = AtomicReference(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565))

    override fun processFrame(input: Mat, captureTimeNanos: Long): Any {
        val bmp = Bitmap.createBitmap(input.width(), input.height(), Bitmap.Config.RGB_565)
        val map = input.clone()
        Utils.matToBitmap(map, bmp)
        this.lastFrame.set(bmp)
        return super.processFrame(input, captureTimeNanos)
    }

    override fun onDrawFrame(canvas: Canvas, onscreenWidth: Int, onscreenHeight: Int, scaleBmpPxToCanvasPx: Float, scaleCanvasDensity: Float, userContext: Any?) {
        val bmp = Bitmap.createBitmap(this.lastFrame.get())

        super.onDrawFrame(Canvas(bmp), onscreenWidth, onscreenHeight, 1f, scaleCanvasDensity, userContext)
        lastDrawnFrame.set(bmp)
        return super.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext)
    }

    override fun getFrameBitmap(continuation: Continuation<out Consumer<Bitmap>>) {
        continuation.dispatch { consumer -> consumer.accept(lastDrawnFrame.get()) }
    }

    class Builder {
        private var fx: Double = 0.0;
        private var fy: Double = 0.0;
        private var cx: Double = 0.0;
        private var cy: Double = 0.0;
        private var tagFamily = TagFamily.TAG_36h11
        private var tagLibrary = AprilTagGameDatabase.getCurrentGameTagLibrary()
        private var outputUnitsLength = DistanceUnit.INCH
        private var outputUnitsAngle = AngleUnit.DEGREES
        private var threads = THREADS_DEFAULT

        private var drawAxes = false
        private var drawCube = false
        private var drawOutline = true
        private var drawTagID = true

        fun setLensIntrinsics(fx: Double, fy: Double, cx: Double, cy: Double): Builder {
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;

            return this
        }

        fun setTagFamily(tagFamily: TagFamily): Builder {
            this.tagFamily = tagFamily
            return this
        }

        fun setTagLibrary(tagLibrary: AprilTagLibrary): Builder {
            this.tagLibrary = tagLibrary
            return this
        }

        fun setDrawAxes(drawAxes: Boolean): Builder {
            this.drawAxes = drawAxes
            return this
        }

        fun setDrawCubeProjection(drawCube: Boolean): Builder {
            this.drawCube = drawCube
            return this
        }

        fun setDrawTagOutline(drawOutline: Boolean): Builder {
            this.drawOutline = drawOutline
            return this
        }

        fun setDrawTagID(drawTagID: Boolean): Builder {
            this.drawTagID = drawTagID
            return this
        }

        fun setNumThreads(threads: Int): Builder {
            this.threads = threads
            return this
        }

        fun build(): AprilTagStream {
            return AprilTagStream(
                    this.fx,
                    this.fy,
                    this.cx,
                    this.cy,
                    this.outputUnitsLength,
                    this.outputUnitsAngle,
                    this.tagLibrary,
                    this.drawAxes,
                    this.drawCube,
                    this.drawOutline,
                    this.drawTagID,
                    this.tagFamily,
                    this.threads
            )
        }
    }
}