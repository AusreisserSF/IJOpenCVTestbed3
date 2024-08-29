package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.xml.RecognitionWindowMapping;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.EnumMap;

public class RecognitionWindowUtils {

    private static final String TAG = RecognitionWindowUtils.class.getSimpleName();

    public static RobotConstants.RecognitionResults lookThroughWindowsAtPixelCount(Mat pThresholdedObject, int pAllianceMinWhitePixelCount,
                                                                                   Mat pImageROI, String pOutputFilenamePreamble,
                                                                                   RecognitionWindowMapping pRecognitionWindowMapping) {
        Pair<Rect, RobotConstants.ObjectLocation> leftWindowData = pRecognitionWindowMapping.recognitionWindows.get(RobotConstants.RecognitionWindow.LEFT);
        Pair<Rect, RobotConstants.ObjectLocation> rightWindowData = pRecognitionWindowMapping.recognitionWindows.get(RobotConstants.RecognitionWindow.RIGHT);
        Pair<Rect, RobotConstants.ObjectLocation> nposWindowData = pRecognitionWindowMapping.recognitionWindows.get(RobotConstants.RecognitionWindow.WINDOW_NPOS);
        RobotConstants.ObjectLocation foundLocation;

        Mat leftWindowBoundary = pThresholdedObject.submat(leftWindowData.first);
        int leftNonZeroCount = Core.countNonZero(leftWindowBoundary);
        RobotLogCommon.d(TAG, "Left recognition window white pixel count " + leftNonZeroCount);

        String leftPixelCountFilename = pOutputFilenamePreamble + "_PXCL.png";
        Imgcodecs.imwrite(leftPixelCountFilename, leftWindowBoundary);
        RobotLogCommon.d(TAG, "Writing " + leftPixelCountFilename);

        Mat rightWindowBoundary = pThresholdedObject.submat(rightWindowData.first);
        int rightNonZeroCount = Core.countNonZero(rightWindowBoundary);
        RobotLogCommon.d(TAG, "Right recognition window white pixel count " + rightNonZeroCount);

        String rightPixelCountFilename = pOutputFilenamePreamble + "_PXCR.png";
        Imgcodecs.imwrite(rightPixelCountFilename, rightWindowBoundary);
        RobotLogCommon.d(TAG, "Writing " + rightPixelCountFilename);

        // If both counts are less than the minimum then we infer that
        // the object is in the third (non-visible) recognition window.
        if (leftNonZeroCount < pAllianceMinWhitePixelCount &&
                rightNonZeroCount < pAllianceMinWhitePixelCount) {
            RobotLogCommon.d(TAG, "White pixel counts for the left and right recognition windows were under the threshold");
            RobotLogCommon.d(TAG, "The object location is " + nposWindowData.second);
            RecognitionWindowUtils.drawRecognitionWindows(pImageROI.clone(), pOutputFilenamePreamble, pRecognitionWindowMapping.recognitionWindows);
            return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
        }

        // Compare the white pixel count in the left and right recognition
        // windows against each other.
        Mat pixelCountOut = pImageROI.clone();
        if (leftNonZeroCount >= rightNonZeroCount) {
            Point leftWindowCentroid = new Point((leftWindowData.first.x + leftWindowData.first.width) / 2.0,
                    (leftWindowData.first.y + leftWindowData.first.height) / 2.0);
            RobotLogCommon.d(TAG, "Center of left recognition window " + leftWindowCentroid);
            RobotLogCommon.d(TAG, "The object location is " + leftWindowData.second);

            Imgproc.circle(pixelCountOut, leftWindowCentroid, 10, new Scalar(0, 255, 0));
            RecognitionWindowUtils.drawRecognitionWindows(pixelCountOut, pOutputFilenamePreamble, pRecognitionWindowMapping.recognitionWindows);
            return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
        }

        // Go with the right recognition window.
        Point rightWindowCentroid = new Point(rightWindowData.first.x + (rightWindowData.first.width / 2.0),
                (rightWindowData.first.y + rightWindowData.first.height) / 2.0);
        RobotLogCommon.d(TAG, "The object location is " + rightWindowData.second);
        RobotLogCommon.d(TAG, "Center of right recognition window " + rightWindowCentroid);

        Imgproc.circle(pixelCountOut, rightWindowCentroid, 10, new Scalar(0, 255, 0));
        RecognitionWindowUtils.drawRecognitionWindows(pixelCountOut, pOutputFilenamePreamble, pRecognitionWindowMapping.recognitionWindows);

        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }

    //## Used in FtcPowerPlay for testing circle detection and bright spot
    // detection - neither was adopted.
    // Look through the left and right recognition windows and determine which
    // the selected object is in - or neither. Also draw the boundaries of the
    // windows.
    public static RobotConstants.RecognitionResults lookThroughWindowsAtCenterPoint(Point pCenterOfObject, Mat pRecognitionObjectOut, String pOutputFilenamePreamble,
                                                                                    EnumMap<RobotConstants.RecognitionWindow, Pair<Rect, RobotConstants.ObjectLocation>> pRecognitionWindows) {
        Pair<Rect, RobotConstants.ObjectLocation> leftWindowData = pRecognitionWindows.get(RobotConstants.RecognitionWindow.LEFT);
        Pair<Rect, RobotConstants.ObjectLocation> rightWindowData = pRecognitionWindows.get(RobotConstants.RecognitionWindow.RIGHT);
        Pair<Rect, RobotConstants.ObjectLocation> nposWindowData = pRecognitionWindows.get(RobotConstants.RecognitionWindow.WINDOW_NPOS);
        RobotConstants.ObjectLocation foundLocation;

        if (leftWindowData == null || rightWindowData == null || nposWindowData == null)
            throw new AutonomousRobotException(TAG, "Failed sanity check: no data for at least one of the recognition windows");

        // Try the left window.
        if (pCenterOfObject.x >= leftWindowData.first.x && pCenterOfObject.x < leftWindowData.first.x + leftWindowData.first.width) {
            foundLocation = leftWindowData.second;
            RobotLogCommon.d(TAG, "Success: Object found in the left recognition window: location " + foundLocation);
        } else
            // Try the right window.
            if (pCenterOfObject.x >= rightWindowData.first.x && pCenterOfObject.x < rightWindowData.first.x + rightWindowData.first.width) {
                foundLocation = rightWindowData.second;
                RobotLogCommon.d(TAG, "Success: Object found in the right recognition window: location " + foundLocation);

            } else {
                foundLocation = nposWindowData.second;
                RobotLogCommon.d(TAG, "Object not found in the left or recognition window: assuming location " + foundLocation);
            }

        // Draw the recognition windows on the ROI with the circles.
        drawRecognitionWindows(pRecognitionObjectOut, pOutputFilenamePreamble, pRecognitionWindows);

        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }

    // Returns the ROI from the full image with the recognition windows drawn in.
    //## No usages in FtcCenterStage. One usage IJCenterStageVision:
    // If the switch --spike_windows is present then its argument
    // must be the OpMode of an Autonomous starting position and the
    // switch --image_file must be present.
    /*
    public static Mat overlayRecognitionWindows(ImageProvider pImageProvider,
                                            String pImageFilename,
                                            RecognitionWindowMapping pRecognitionWindowMapping) throws InterruptedException {
        RobotLogCommon.d(TAG, "In " + TAG + ". overlayRecognitionWindows");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> ObjectImage = pImageProvider.getImage();
        if (ObjectImage == null)
            return null; // don't crash

        // The image is in OpenCV BGR order.
        String imageDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.IMAGE_DIR;
        String fileDate = TimeStamp.getLocalDateTimeStamp(ObjectImage.second); 
        String outputFilenamePreamble = pImageFilename + "_" + fileDate; // ImageUtils.createOutputFilePreamble(pImageFilename, imageDirectory, fileDate);
        Mat imageROI = ImageUtils.preProcessImage(ObjectImage.first, outputFilenamePreamble, pRecognitionWindowMapping.imageParameters);
        drawRecognitionWindows(imageROI, outputFilenamePreamble, pRecognitionWindowMapping.recognitionWindows);
        return imageROI;
    }
    */

    public static void drawRecognitionWindows(Mat pRecognitionObjectOut, String pOutputFilenamePreamble,
                                              EnumMap<RobotConstants.RecognitionWindow, Pair<Rect, RobotConstants.ObjectLocation>> pRecognitionWindows) {
        Pair<Rect, RobotConstants.ObjectLocation> leftWindowData = pRecognitionWindows.get(RobotConstants.RecognitionWindow.LEFT);
        Pair<Rect, RobotConstants.ObjectLocation> rightWindowData = pRecognitionWindows.get(RobotConstants.RecognitionWindow.RIGHT);

        // Draw the recognition windows on the ROI
        // so that we can see their placement during debugging.
        // params Mat, Point upperLeft, Point lowerRight, Scalar color, int thickness
        Point leftWindowUpperLeft = new Point(leftWindowData.first.x, leftWindowData.first.y);
        Point leftWindowLowerRight = new Point(leftWindowData.first.x + leftWindowData.first.width,
                leftWindowData.first.y + leftWindowData.first.height);

        Point rightWindowUpperLeft = new Point(rightWindowData.first.x, rightWindowData.first.y);
        Point rightWindowLowerRight = new Point(rightWindowData.first.x + rightWindowData.first.width,
                rightWindowData.first.y + rightWindowData.first.height);

        Imgproc.rectangle(pRecognitionObjectOut, leftWindowUpperLeft, leftWindowLowerRight, new Scalar(0, 255, 0), 3);
        Imgproc.rectangle(pRecognitionObjectOut, rightWindowUpperLeft, rightWindowLowerRight, new Scalar(0, 255, 0), 3);

        if (pOutputFilenamePreamble != null) {
            String objectFilename = pOutputFilenamePreamble + "_LOC.png";
            RobotLogCommon.d(TAG, "Writing " + objectFilename);
            Imgcodecs.imwrite(objectFilename, pRecognitionObjectOut);
        }
    }

}