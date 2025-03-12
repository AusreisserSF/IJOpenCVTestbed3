package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.TimeStamp;
import org.firstinspires.ftc.teamcode.auto.DebugImageCommon;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.xml.RedAllianceSampleParameters;
import org.firstinspires.ftc.teamcode.auto.xml.VisionParameters;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;

public class RedAllianceSampleRecognition {

    private static final String TAG = RedAllianceSampleRecognition.class.getSimpleName();

    public enum RecognitionPath {
        RED_CHANNEL_GRAYSCALE
    }

    private final String testCaseDirectory;

    public RedAllianceSampleRecognition(String pTestCaseDirectory) {
        testCaseDirectory = pTestCaseDirectory;
    }

    // FTC 2024-2025 IntoTheDeep.
    public RobotConstants.RecognitionResults recognizeRedAllianceSamples(ImageProvider pImageProvider,
                                                                         VisionParameters.ImageParameters pImageParameters,
                                                                         RecognitionPath pRecognitionPath) throws InterruptedException {

        RobotLogCommon.d(TAG, "In RedAllianceSampleRecognition.recognizeRedAllianceSamples");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> redAllianceSampleImage = pImageProvider.getImage();
        if (redAllianceSampleImage == null)
            return RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR; // don't crash

        // The image is in BGR order (OpenCV imread from a file).
        String fileDate = TimeStamp.getLocalDateTimeStamp(redAllianceSampleImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(pImageParameters.image_source, testCaseDirectory, fileDate);
        Mat imageROI = ImageUtils.preProcessImage(redAllianceSampleImage.first, outputFilenamePreamble, pImageParameters);

        RobotLogCommon.d(TAG, "Recognition path " + pRecognitionPath);
        if (pRecognitionPath != RecognitionPath.RED_CHANNEL_GRAYSCALE)
            throw new AutonomousRobotException(TAG, "Unrecognized recognition path");

        return redChannelPath(imageROI, outputFilenamePreamble);
    }

    private RobotConstants.RecognitionResults redChannelPath(Mat pImageROI, String pOutputFilenamePreamble) {

        // Extract the red channel and then use it as grayscale.
        // The red channel will pick up both the red and yellow samples.
        Mat selectedChannel = new Mat();
        Core.extractChannel(pImageROI, selectedChannel, 2);

        // Write out the red channel as grayscale.
        if (RobotLogCommon.isLoggable(RobotLogCommon.CommonLogLevel.d)) {
            DebugImageCommon.writeImage(pOutputFilenamePreamble + "_RED_CHANNEL.png", selectedChannel);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_RED_CHANNEL.png");
        }

        Mat thresholded = ImageUtils.performThresholdOnGray(selectedChannel,
                RedAllianceSampleParameters.redGrayParameters.median_target,
                RedAllianceSampleParameters.redGrayParameters.threshold_low,
                pOutputFilenamePreamble, "");

        //**TODO As part of filtering you'll have to get an idea of the proximity of the
        // camera to the samples; this will affect the pixel counts.

        // Sanitize the thresholded red alliance samples by eliminating contours
        // that are below the minimum area threshold.
        ImageUtils.FilteredContoursRecord filteredA = ImageUtils.filterContours(thresholded, pImageROI.rows(), pImageROI.cols(),
                RedAllianceSampleParameters.sampleCriteria.min_sample_area / 2.0,
                pOutputFilenamePreamble, "_A");

        // Log statistics.
        RobotLogCommon.d(TAG, "numUnfilteredAllianceContours " + filteredA.numUnfilteredContours);
        RobotLogCommon.d(TAG, "numFilteredAllianceContours " + filteredA.numFilteredContours);

        List<RotatedRect> rectanglesFound = new ArrayList<>();
        RotatedRect oneRotatedRect;
        for (MatOfPoint oneContour : filteredA.filteredContours) {
            oneRotatedRect = findRectangle(oneContour);
            if (oneRotatedRect != null) {
                rectanglesFound.add(oneRotatedRect);
                RobotLogCommon.d(TAG, "Found a rectangle with center x " + oneRotatedRect.center.x +
                        ", y " + oneRotatedRect.center.y);
                RobotLogCommon.d(TAG, "Rectangle width " + oneRotatedRect.size.width +
                        ", height " + oneRotatedRect.size.height +
                        ", area " + (oneRotatedRect.size.width * oneRotatedRect.size.height));
            }
        }

        Point[] rectPoints = new Point[4];
        Mat drawnRects = pImageROI.clone();
        List<MatOfPoint> rectContours = new ArrayList<>();
        for (RotatedRect oneRect : rectanglesFound) {
            // Draw a rotated rectangle around a sample.
            oneRect.points(rectPoints);
            rectContours.add(new MatOfPoint(rectPoints)); // List is required
            Imgproc.drawContours(drawnRects, rectContours, 0, new Scalar(0, 255, 0), 2);
            rectContours.clear();
        }

        // Write a file with the rectangles.
        if (RobotLogCommon.isLoggable(RobotLogCommon.CommonLogLevel.d)) {
            String fullFilename = pOutputFilenamePreamble + "_RRECT" + "_A" + ".png";
            DebugImageCommon.writeImage(fullFilename, drawnRects);
            RobotLogCommon.d(TAG, "Writing " + fullFilename);
        }

        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }

    // Based on https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
    private RotatedRect findRectangle(MatOfPoint thisContour) {

        RotatedRect retVal = null;

        MatOfPoint2f thisContour2f = new MatOfPoint2f();
        MatOfPoint2f approxContour2f = new MatOfPoint2f();

        thisContour.convertTo(thisContour2f, CvType.CV_32FC2);

        double perimeter = Imgproc.arcLength(thisContour2f, true);
        Imgproc.approxPolyDP(thisContour2f, approxContour2f, 0.04 * perimeter, true);

        double height = approxContour2f.size().height;
        double width = approxContour2f.size().width;
        RobotLogCommon.d(TAG, "Polygon height " + height + ", width " + width);
        if ((height >= 4) && (height <= 6))
            retVal = Imgproc.minAreaRect(approxContour2f);

        return retVal;
    }
}