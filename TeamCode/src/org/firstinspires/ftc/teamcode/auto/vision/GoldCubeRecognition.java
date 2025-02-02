package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.TimeStamp;
import org.firstinspires.ftc.teamcode.auto.DebugImageCommon;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.xml.GoldCubeParameters;
import org.firstinspires.ftc.teamcode.auto.xml.VisionParameters;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.Optional;

public class GoldCubeRecognition {

    private static final String TAG = GoldCubeRecognition.class.getSimpleName();

    public enum GoldCubeRecognitionPath {
        RED_CHANNEL_GRAYSCALE, COLOR
    }

    private final String testCaseDirectory;
    private final RobotConstants.Alliance alliance;

    // For drawing a red X on the image ROI in the event that no
    // contours were found or the largest contour is smaller than
    // a minimum or larger than a maximum.
    // The greater the factor, the smaller the red X.
    private final Scalar redXColor = new Scalar(0, 0, 255); // BGR

    public GoldCubeRecognition(String pTestCaseDirectory, RobotConstants.Alliance pAlliance) {
        testCaseDirectory = pTestCaseDirectory;
        alliance = pAlliance;
    }

    //## Inversion of the opposing alliance's BGR channel can yield
    // good results. See DistanceTransformRecognition.
    // Returns the result of image analysis.
     public RobotConstants.RecognitionResults recognizeGoldCubeWebcam(ImageProvider pImageProvider,
                                                                      VisionParameters.ImageParameters pImageParameters,
                                                                      GoldCubeParameters pGoldCubeParameters, GoldCubeRecognitionPath pGoldCubeRecognitionPath) throws InterruptedException {

        RobotLogCommon.d(TAG, "In GoldCubeRecognition.recognizeGoldCubeWebcam");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> goldCubeImage = pImageProvider.getImage();
        if (goldCubeImage == null)
            return RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR; // don't crash

        // The image is in BGR order (OpenCV imread from a file).
        String fileDate = TimeStamp.getLocalDateTimeStamp(goldCubeImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(pImageParameters.image_source, testCaseDirectory, fileDate);
        Mat imageROI = ImageUtils.preProcessImage(goldCubeImage.first, outputFilenamePreamble, pImageParameters);

        RobotLogCommon.d(TAG, "Recognition path " + pGoldCubeRecognitionPath);
        switch (pGoldCubeRecognitionPath) {
            case RED_CHANNEL_GRAYSCALE -> {
                return redChannelPathWebcam(imageROI, outputFilenamePreamble, pGoldCubeParameters);
            }
            case COLOR -> {
                return colorPathWebcam(imageROI, outputFilenamePreamble, pGoldCubeParameters);
            }
            default -> throw new AutonomousRobotException(TAG, "Unrecognized recognition path");
        }
    }

    private RobotConstants.RecognitionResults redChannelPathWebcam(Mat pImageROI, String pOutputFilenamePreamble,
                                                                   GoldCubeParameters pGoldCubeParameters) {

        // Extract the red channel and then use it as grayscale.
        Mat selectedChannel = new Mat();
        Core.extractChannel(pImageROI, selectedChannel, 2);

        // Write out the red channel as grayscale.
        if (RobotLogCommon.isLoggable(RobotLogCommon.CommonLogLevel.d)) {
            DebugImageCommon.writeImage(pOutputFilenamePreamble + "_RED_CHANNEL.png", selectedChannel);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_RED_CHANNEL.png");
        }

        Mat thresholded = ImageUtils.performThresholdOnGray(selectedChannel,
                pGoldCubeParameters.grayscaleParameters.median_target,
                pGoldCubeParameters.grayscaleParameters.threshold_low,
                pOutputFilenamePreamble, "");

        Optional<Pair<Integer, MatOfPoint>> targetContour = ImageUtils.getLargestContour(pImageROI, thresholded, pOutputFilenamePreamble);
        if (!targetContour.isPresent()) {
            ShapeDrawing.drawX(pImageROI, redXColor, pOutputFilenamePreamble);
            RobotLogCommon.d(TAG, "No contours found");
            return RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL; // don't crash
        }

        Rect contourBoundingRect = Imgproc.boundingRect(targetContour.get().second);
        RobotLogCommon.d(TAG, "Bounding box of largest contour: area " + contourBoundingRect.area());

        // Within the ROI draw a rectangle around the largest contour.
        Mat drawnRectangle = pImageROI.clone();
        ShapeDrawing.drawOneRectangle(contourBoundingRect, drawnRectangle, 2);

        // Check the size of the largest contour.
        if (contourBoundingRect.area() < pGoldCubeParameters.boundingBoxCriteria.minBoundingBoxArea ||
                contourBoundingRect.area() > pGoldCubeParameters.boundingBoxCriteria.maxBoundingBoxArea) {
            ShapeDrawing.drawX(drawnRectangle, redXColor, pOutputFilenamePreamble);
            RobotLogCommon.d(TAG, "The largest contour violates the size criteria");
            return RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL; // don't crash
        }

        if (RobotLogCommon.isLoggable(RobotLogCommon.CommonLogLevel.v)) {
            Imgcodecs.imwrite(pOutputFilenamePreamble + "_BRECT.png", drawnRectangle);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_BRECT.png");
        }

        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }

    // Analyze a color image.
    public RobotConstants.RecognitionResults colorPathWebcam(Mat pImageROI, String pOutputFilenamePreamble,
                                                             GoldCubeParameters pGoldCubeParameters) {

        Mat thresholded = ImageUtils.performInRange(pImageROI, pGoldCubeParameters.hsvParameters, pOutputFilenamePreamble, "");

        // Clean up the thresholded image via morphological opening.
        Mat morphed = new Mat();
        Imgproc.erode(thresholded, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
        Imgproc.dilate(morphed, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

        Optional<Pair<Integer, MatOfPoint>> targetContour = ImageUtils.getLargestContour(pImageROI, morphed, pOutputFilenamePreamble);
        if (!targetContour.isPresent()) {
            ShapeDrawing.drawX(pImageROI, redXColor, pOutputFilenamePreamble);
            RobotLogCommon.d(TAG, "No contours found");
            return RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL; // don't crash
        }

        Rect contourBoundingRect = Imgproc.boundingRect(targetContour.get().second);
        RobotLogCommon.d(TAG, "Bounding box of largest contour: area " + contourBoundingRect.area());

        // Within the ROI draw a rectangle around the largest contour.
        Mat drawnRectangle = pImageROI.clone();
        ShapeDrawing.drawOneRectangle(contourBoundingRect, drawnRectangle, 2);

        // Check the size of the largest contour.
        if (contourBoundingRect.area() < pGoldCubeParameters.boundingBoxCriteria.minBoundingBoxArea ||
                contourBoundingRect.area() > pGoldCubeParameters.boundingBoxCriteria.maxBoundingBoxArea) {
            ShapeDrawing.drawX(drawnRectangle, redXColor, pOutputFilenamePreamble);
            RobotLogCommon.d(TAG, "The largest contour violates the size criteria");
            return RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL; // don't crash
        }

        if (RobotLogCommon.isLoggable(RobotLogCommon.CommonLogLevel.d)) {
            DebugImageCommon.writeImage(pOutputFilenamePreamble + "_BRECT.png", drawnRectangle);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_BRECT.png");
        }

        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }

}