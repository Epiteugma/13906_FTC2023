package org.firstinspires.ftc.teamcode.lib.vision.detection;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

public class ItemDetectorJava extends OpenCvPipeline {
    private Location location = Location.NONE;
    private Mat mat = new Mat();
    private Scalar blueLowHSV = new Scalar(80.0, 200.0, 240.0); // (0-180, 0-255, 0-255)
    private Scalar blueHighHSV = new Scalar(180, 255.0, 255.0); // (0-180, 0-255, 0-255)

    private Scalar bothLowHSV = new Scalar(80.0, 100.0, 200.0); // (0-180, 0-255, 0-255)
    private Scalar bothHighHSV = new Scalar(180, 255.0, 255.0); // (0-180, 0-255, 0-255)

    private Scalar redLowHSV = new Scalar(165.0, 0.0, 200.0); // (0-180, 0-255, 0-255)
    private Scalar redHighHSV = new Scalar(180, 255.0, 255.0); // (0-180, 0-255, 0-255)

    private Scalar lowHSV = bothLowHSV;
    private Scalar highHSV = bothHighHSV;

    private int width = 640;
    private int height = 480;
    private double margin = 3.0;

    // top left corner is (0, 0)
    // Left is slightly lower as it is closer
    private Rect LEFT_ROI = new Rect(
            new Point(0.0 + margin, height * 0.4),
            new Point(width / 3.0 - margin, height * 0.8)
    );
    // center must be smaller as it is the furthest away
    private Rect CENTER_ROI = new Rect(
            new Point(width / 3.0, height * 0.25),
            new Point(width * 2.0 / 3.0, height * 0.6)
    );
    private Rect RIGHT_ROI = new Rect(
            new Point(width * 2.0 / 3.0 + margin, height * 0.25),
            new Point(width - margin, height * 0.75)
    );

    private Telemetry telemetry;

    public enum Location {
        NONE(0),
        LEFT(0.2),
        CENTER(0.1),
        RIGHT(0.2);

        private double value;

        private  Location(double i) {
            this.value = i;
        }

        public double getValue() {
            return value;
        }


    }

    public ItemDetectorJava(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(mat, lowHSV, highHSV, mat);
        Mat left = mat.submat(LEFT_ROI);
        Mat center = mat.submat(CENTER_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        this.telemetry.addData("Left", leftValue);
        this.telemetry.addData("Center", centerValue);
        this.telemetry.addData("Right", rightValue);
        left.release();
        right.release();
        center.release();
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        double[] values = {leftValue, centerValue, rightValue};
        Arrays.sort(values);
        // check if the value is above its threshold
        if(leftValue == values[2] && leftValue > Location.LEFT.getValue()) location = Location.LEFT;
        else if(centerValue == values[2] && centerValue > Location.CENTER.getValue()) location = Location.CENTER;
        else if(rightValue == values[2] && rightValue > Location.RIGHT.getValue()) location = Location.RIGHT;
        this.telemetry.addData("Location", location);
        this.telemetry.update();
        Scalar found = new Scalar(0.0, 255.0, 0.0);
        Scalar notFound = new Scalar(255.0, 0.0, 0.0);
        Imgproc.rectangle(mat, LEFT_ROI, (location == Location.LEFT) ? found : notFound);
        Imgproc.rectangle(mat, RIGHT_ROI, (location == Location.RIGHT) ? found : notFound);
        Imgproc.rectangle(mat, CENTER_ROI, (location == Location.CENTER) ? found : notFound);
        return mat;
    }
}
