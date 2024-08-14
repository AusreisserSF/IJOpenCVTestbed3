package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.TimeStamp;
import org.firstinspires.ftc.teamcode.auto.xml.DistanceParameters;
import org.firstinspires.ftc.teamcode.auto.xml.RecognitionWindowMapping;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.xml.VisionParameters;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.ArrayList;

public class DistanceTransformRecognition {

    private static final String TAG = DistanceTransformRecognition.class.getSimpleName();

    public enum DistanceTransformRecognitionPath {
        COLOR_CHANNEL_BRIGHT_SPOT, COLOR_CHANNEL_PIXEL_COUNT
    }

    private final RobotConstants.Alliance alliance;
    private final String testCaseDirectory;

    public DistanceTransformRecognition(RobotConstants.Alliance pAlliance, String pTestCaseDirectory) {
        alliance = pAlliance;
        testCaseDirectory = pTestCaseDirectory;
    }

    // Use the OpenCV Distance Transform algorithm to detect an object.
    // Returns the result of image analysis.
    public RobotConstants.RecognitionResults performDistanceTransform(ImageProvider pImageProvider,
                                                                      VisionParameters.ImageParameters pImageParameters,
                                                                      DistanceTransformRecognitionPath pDistanceRecognitionPath,
                                                                      DistanceParameters pDistanceParameters,
                                                                      RecognitionWindowMapping pRecognitionWindowMapping) throws InterruptedException {
        RobotLogCommon.d(TAG, "In DistanceTransformRecognition.performDistanceTransform");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> transformImage = pImageProvider.getImage();
        if (transformImage == null)
            return RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR; // don't crash

        // The image is in BGR order (OpenCV imread from a file).
        String fileDate = TimeStamp.getLocalDateTimeStamp(transformImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(pImageParameters.image_source, testCaseDirectory, fileDate);
        Mat imageROI = ImageUtils.preProcessImage(transformImage.first, outputFilenamePreamble, pImageParameters);
        RobotLogCommon.d(TAG, "Recognition path " + pDistanceRecognitionPath);

        // Adapt the standard example to our environment.
        switch (pDistanceRecognitionPath) {
            case COLOR_CHANNEL_BRIGHT_SPOT -> {
                Mat distanceTransformImage = getDistanceTransformImage(imageROI, outputFilenamePreamble,
                        pDistanceParameters.colorChannelBrightSpotParameters.redGrayParameters,
                        pDistanceParameters.colorChannelBrightSpotParameters.blueGrayParameters);
                return colorChannelBrightSpot(imageROI, distanceTransformImage, outputFilenamePreamble,
                        pDistanceParameters.colorChannelBrightSpotParameters,
                        pRecognitionWindowMapping);

            }
            case COLOR_CHANNEL_PIXEL_COUNT -> {
                Mat distanceTransformImage = getDistanceTransformImage(imageROI, outputFilenamePreamble,
                        pDistanceParameters.colorChannelPixelCountParameters.redGrayParameters,
                        pDistanceParameters.colorChannelPixelCountParameters.blueGrayParameters);
                return colorChannelPixelCount(imageROI, distanceTransformImage, outputFilenamePreamble,
                        pDistanceParameters.colorChannelPixelCountParameters,
                        pRecognitionWindowMapping);
            }
            default -> throw new AutonomousRobotException(TAG, "Unrecognized recognition path");
        }
    }

    // Based on the OpenCV Python example for watershed:
    // https://docs.opencv.org/4.x/d2/dbd/tutorial_distance_transform.html
    private Mat getDistanceTransformImage(Mat pImageROI, String pOutputFilenamePreamble,
                                          VisionParameters.GrayParameters pRedGrayParameters,
                                          VisionParameters.GrayParameters pBlueGrayParameters) {

        VisionParameters.GrayParameters allianceGrayParameters;
        switch (alliance) {
            case RED -> allianceGrayParameters = pRedGrayParameters;
            case BLUE -> allianceGrayParameters = pBlueGrayParameters;
            default -> throw new AutonomousRobotException(TAG, "distance transform requires an alliance selection");
        }

        //##PY Apply a sharpening kernel to the color image.
        Mat sharp = sharpen(pImageROI, pOutputFilenamePreamble);

        // Extract the alliance channel from the BGR image and invert.
        // See the comments above the method.
        Mat invertedChannel = ImageUtils.extractAndInvertChannel(sharp, alliance, allianceGrayParameters, pOutputFilenamePreamble);

        // Follow the Python example and threshold the grayscale.
        //## Imgproc.THRESH_BINARY works better than OTSU here
        // on bright spot recognition.
        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(invertedChannel, thresholded,
                allianceGrayParameters.threshold_low,
                255,   // white
                Imgproc.THRESH_BINARY); // Imgproc.THRESH_BINARY | Imgproc.THRESH_OTSU); // thresholding type
        RobotLogCommon.v(TAG, "Threshold values: low " + allianceGrayParameters.threshold_low + ", high 255");

        String thrFilename = pOutputFilenamePreamble + "_THR.png";
        Imgcodecs.imwrite(thrFilename, thresholded);
        RobotLogCommon.d(TAG, "Writing " + thrFilename);

        // Perform the distance transform algorithm. Imgproc.DIST_L2
        // is a flag for Euclidean distance. Output is 32FC1.
        Mat dist = new Mat();
        Imgproc.distanceTransform(thresholded, dist, Imgproc.DIST_L2, 3);

        Core.normalize(dist, dist, 0.0, 255.0, Core.NORM_MINMAX);
        Mat dist_8u = new Mat();
        dist.convertTo(dist_8u, CvType.CV_8U);

        // Output the transformed image.
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_DIST.png", dist_8u);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_DIST.png");
        //! [dist]

        return dist_8u;
    }

    //!! BrightSpot recognition is more risky than pixel counting because if
    // just one white pixel sneaks through our filtering the result will be a
    // false positive. But keep this as an interesting technique.
    private RobotConstants.RecognitionResults colorChannelBrightSpot(Mat pImageROI, Mat pDistanceImage,
                                                                     String pOutputFilenamePreamble,
                                                                     DistanceParameters.ColorChannelBrightSpotParameters pBrightSpotParameters,
                                                                     RecognitionWindowMapping pRecognitionWindowMapping) {

        VisionParameters.GrayParameters allianceGrayParameters;
        switch (alliance) {
            case RED -> allianceGrayParameters = pBrightSpotParameters.redGrayParameters;
            case BLUE -> allianceGrayParameters = pBrightSpotParameters.blueGrayParameters;
            default -> throw new AutonomousRobotException(TAG, "findBrightSpot requires an alliance selection");
        }

        Core.MinMaxLocResult brightResult = Core.minMaxLoc(pDistanceImage);
        RobotLogCommon.d(TAG, "Bright spot location " + brightResult.maxLoc + ", value " + brightResult.maxVal);

        Mat brightSpotOut = pImageROI.clone();
        Imgproc.circle(brightSpotOut, brightResult.maxLoc, 10, new Scalar(0, 255, 0));

        Imgcodecs.imwrite(pOutputFilenamePreamble + "_BRIGHT.png", brightSpotOut);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_BRIGHT.png");

        // If the bright spot is under the threshold then assume no Team Prop is present.
        //## We need a lower threshold for the distance image since it has undergone two
        // morphological openings. Arbitrarily use 1/2 of the low threshold value for the
        // split channel grayscale.
        if (brightResult.maxVal < allianceGrayParameters.threshold_low / 2.0) {
            RobotLogCommon.d(TAG, "Bright spot value was under the threshold");
            return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
        }

        return RecognitionWindowUtils.lookThroughWindows(brightResult.maxLoc, brightSpotOut, pOutputFilenamePreamble,
                pRecognitionWindowMapping.recognitionWindows);
    }

    private RobotConstants.RecognitionResults colorChannelPixelCount(Mat pImageROI, Mat pDistanceImage,
                                                                     String pOutputFilenamePreamble,
                                                                     DistanceParameters.ColorChannelPixelCountParameters pPixelCountParameters,
                                                                     RecognitionWindowMapping pRecognitionWindowMapping) {

        // Use the threshold and pixel count criteria parameters for the current alliance.
        int allianceThresholdLow;
        int allianceMinWhitePixelCount;
        switch (alliance) {
            case RED -> {
                allianceThresholdLow = pPixelCountParameters.redGrayParameters.threshold_low;
                allianceMinWhitePixelCount = pPixelCountParameters.redMinWhitePixelCount;
            }
            case BLUE -> {
                allianceThresholdLow = pPixelCountParameters.blueGrayParameters.threshold_low;
                allianceMinWhitePixelCount = pPixelCountParameters.blueMinWhitePixelCount;
            }
            default ->
                    throw new AutonomousRobotException(TAG, "colorChannelPixelCountPath requires an alliance selection");
        }

        //## The distance transform tends to produce a diffuse result because the
        // values closest to white only occur near the center of an object such as
        // the team prop. But OTSU works fine here.
        Mat thresholded = new Mat();
        Imgproc.threshold(pDistanceImage, thresholded, 0, 255, Imgproc.THRESH_BINARY | Imgproc.THRESH_OTSU);

        //**TODO lookThroughWindowsAtCenterPoint vs lookThroughWindowsAtPixelCount?
        // Get the white pixel count for both the left and right recognition windows.
        Pair<Rect, RobotConstants.ObjectLocation> leftWindowData =
                pRecognitionWindowMapping.recognitionWindows.get(RobotConstants.RecognitionWindow.LEFT);
        Mat leftWindowBoundary = thresholded.submat(leftWindowData.first);
        int leftNonZeroCount = Core.countNonZero(leftWindowBoundary);
        RobotLogCommon.d(TAG, "Left recognition window white pixel count " + leftNonZeroCount);

        String leftPixelCountFilename = pOutputFilenamePreamble + "_PXCL.png";
        Imgcodecs.imwrite(leftPixelCountFilename, leftWindowBoundary);
        RobotLogCommon.d(TAG, "Writing " + leftPixelCountFilename);

        Pair<Rect, RobotConstants.ObjectLocation> rightWindowData =
                pRecognitionWindowMapping.recognitionWindows.get(RobotConstants.RecognitionWindow.RIGHT);
        Mat rightWindowBoundary = thresholded.submat(rightWindowData.first);
        int rightNonZeroCount = Core.countNonZero(rightWindowBoundary);
        RobotLogCommon.d(TAG, "Right recognition window white pixel count " + rightNonZeroCount);

        String rightPixelCountFilename = pOutputFilenamePreamble + "_PXCR.png";
        Imgcodecs.imwrite(rightPixelCountFilename, rightWindowBoundary);
        RobotLogCommon.d(TAG, "Writing " + rightPixelCountFilename);

        // If both counts are less than the minimum then we infer that
        // the object is in the third (non-visible) recognition window.
        if (leftNonZeroCount < allianceMinWhitePixelCount &&
                rightNonZeroCount < allianceMinWhitePixelCount) {
            Pair<Rect, RobotConstants.ObjectLocation> nposWindowData = pRecognitionWindowMapping.recognitionWindows.get(RobotConstants.RecognitionWindow.WINDOW_NPOS);
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

    //**TODO To ImageUtils ...
    //## Imported from IJCenterStageVision.
    //## This sharpening filter makes a difference in marginal cases.
    // From OpencvTestbed3 (cpp) GrayscaleTechnique
    // From https://stackoverflow.com/questions/27393401/opencv-in-java-for-image-filtering
    private Mat sharpen(Mat pDullMat, String pOutputFilenamePreamble) {
        int kernelSize = 3;
        Mat kernel = new Mat(kernelSize, kernelSize, CvType.CV_32F) {
            {
                put(0, 0, 0);
                put(0, 1, -1);
                put(0, 2, 0);

                put(1, 0, -1);
                put(1, 1, 5);
                put(1, 2, -1);

                put(2, 0, 0);
                put(2, 1, -1);
                put(2, 2, 0);
            }
        };

        Mat sharpMat = new Mat();
        Imgproc.filter2D(pDullMat, sharpMat, -1, kernel);

        String sharpFilename = pOutputFilenamePreamble + "_SHARP.png";
        RobotLogCommon.d(TAG, "Writing " + sharpFilename);
        Imgcodecs.imwrite(sharpFilename, sharpMat);

        return sharpMat;
    }

}