package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class TeamMarkerDetector {

    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    boolean special;

    private static float rectHeight = 1.0f/8f;
    private static float rectWidth = 1.0f/8f;

    private static float offsetX = 0.0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 2.5f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] leftPos = {1.0f/8f+offsetX, 4f/8f+offsetY};
    private static float[] midPos = {4.0f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] rightPos = {7.0f/8f+offsetX, 4f/8f+offsetY};

    boolean Right;
    boolean Left;
    boolean Middle;
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera webcam;
    LinearOpMode op;

    private TeamMarkerDetectionPipeline pipeline;

    public enum TeamMarkerPosition {
        NOT_DETECTED,
        LEFT,
        MIDDLE,
        RIGHT
    }

    static public String PosToString(TeamMarkerPosition pos) {
        switch (pos) {
            case NOT_DETECTED:
                return "NOT_DETECTED";
            case LEFT:
                return "LEFT";
            case MIDDLE:
                return "MIDDLE";
            case RIGHT:
            default:
                return "RIGHT";
        }
    }


    public TeamMarkerDetector(LinearOpMode op) {
        this.op = op;
    }

    public TeamMarkerDetector(LinearOpMode op, boolean special) {
        this.op = op;
        this.special = special;
        if (special == true) {
            rectHeight = 1.0f/8f;
            rectWidth = 1.0f/8f;

            offsetX = 0.0f/8f;
            offsetY = 2.5f/8f;

            leftPos[0] = 4.0f/8f+offsetX;
            leftPos[1] = 4f/8f+offsetY;
            midPos[0] = 7.0f/8f+offsetX;
            midPos[1] = 4f/8f+offsetY;
            rightPos[0] = 1.0f/8f+offsetX;
            rightPos[1] = 4f/8f+offsetY;
        }
    }


    public void init() {
        int cameraMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                op.hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                op.hardwareMap.get(WebcamName.class, "astroEye"),
                cameraMonitorViewId);

        pipeline = new TeamMarkerDetectionPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public void clean() {
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    public TeamMarkerPosition getTeamMarkerPosition( ) {
        TeamMarkerPosition teamMarkerPosition = TeamMarkerPosition.NOT_DETECTED;

        op.telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
        op.telemetry.update();

        RobotLog.vv("AstroBot", "valLeft = %d, valMid = %d, valRight = %d", valLeft, valMid, valRight);

        if (valLeft == 255) {
            teamMarkerPosition = TeamMarkerPosition.LEFT;
        } else if (valMid == 255) {
            teamMarkerPosition = TeamMarkerPosition.MIDDLE;
        } else if (valRight == 255) {
            teamMarkerPosition = TeamMarkerPosition.RIGHT;
        }

        return teamMarkerPosition;
    }

    //detection pipeline
    static class TeamMarkerDetectionPipeline extends OpenCvPipeline
    {
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        Mat thresholdMat = new Mat();

        @Override
        public Mat processFrame(Mat input)
        {

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(YCrCb, Cb, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(Cb, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];


            // Original color values
            double[] CbMid = Cb.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            int pixCbMid = (int)CbMid[0];

            double[] CbLeft = Cb.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            int pixCbLeft = (int)CbLeft[0];

            double[] CbRight = Cb.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            int pixCbRight = (int)CbRight[0];

            RobotLog.vv("OriginalValue", "valLeft = %d, valMid = %d, valRight = %d", pixCbLeft, pixCbMid, pixCbRight);

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(input, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(input, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(input, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    input,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    input,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    input,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            RobotLog.vv("AstroBot", "valLeft = %d, valMid = %d, valRight = %d", valLeft, valMid, valRight);

            return input;
        }

    }

}