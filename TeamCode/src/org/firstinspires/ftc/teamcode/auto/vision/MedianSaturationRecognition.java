package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.TimeStamp;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.xml.VisionParameters;
import org.firstinspires.ftc.teamcode.auto.xml.WatershedParametersFtc;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.ArrayList;

public class MedianSaturationRecognition {

    private static final String TAG = MedianSaturationRecognition.class.getSimpleName();

    public enum MedianSaturationRecognitionPath {
        MEDIAN_SATURATION
    }

    private final RobotConstants.Alliance alliance;
    private final String testCaseDirectory;

    public MedianSaturationRecognition(RobotConstants.Alliance pAlliance, String pTestCaseDirectory) {
        alliance = pAlliance;
        testCaseDirectory = pTestCaseDirectory;
    }

    // Returns the result of image analysis.
    public RobotConstants.RecognitionResults performMedianSaturation(ImageProvider pImageProvider,
                                                                 VisionParameters.ImageParameters pImageParameters,
                                                                 MedianSaturationRecognitionPath pMedianSaturationRecognitionPath,
                                                                 //**TODO TEMP - use watershed parameters.
                                                                 WatershedParametersFtc pMedianSaturationParameters) throws InterruptedException {
        RobotLogCommon.d(TAG, "In MedianSaturationRecognition.performMedianSaturation");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> watershedImage = pImageProvider.getImage();
        if (watershedImage == null)
            return RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR; // don't crash

        // The image is in BGR order (OpenCV imread from a file).
        String fileDate = TimeStamp.getLocalDateTimeStamp(watershedImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(pImageParameters.image_source, testCaseDirectory, fileDate);
        Mat imageROI = ImageUtils.preProcessImage(watershedImage.first, outputFilenamePreamble, pImageParameters);
        RobotLogCommon.d(TAG, "Recognition path " + pMedianSaturationRecognitionPath);

        // Adapt the standard example to our environment.
        switch (pMedianSaturationRecognitionPath) {
            case MEDIAN_SATURATION -> {
                return evaluateHSVSaturationChannel(imageROI, outputFilenamePreamble, pMedianSaturationParameters.watershedDistanceParameters);
            }
            default -> throw new AutonomousRobotException(TAG, "Unrecognized recognition path");
        }
    }

    private RobotConstants.RecognitionResults evaluateHSVSaturationChannel(Mat pImageROI, String pOutputFilenamePreamble, WatershedParametersFtc.WatershedDistanceParameters pWatershedDistanceParameters) {

        // Use the grayscale and pixel count criteria parameters for the current alliance.
        VisionParameters.GrayParameters allianceGrayParameters;
        switch (alliance) {
            case RED -> allianceGrayParameters = pWatershedDistanceParameters.redGrayParameters;
            case BLUE -> allianceGrayParameters = pWatershedDistanceParameters.blueGrayParameters;
            default ->
                    throw new AutonomousRobotException(TAG, "colorChannelPixelCountPath requires an alliance selection");
        }

        //**TODO Probably don't need sharpening
        //Mat sharp = ImageUtils.sharpen(pImageROI, pOutputFilenamePreamble);

        //**TODO Convert to HSV and split the channels.
        // We're on the HSV path.
        Mat hsvROI = new Mat();
        Imgproc.cvtColor(pImageROI, hsvROI, Imgproc.COLOR_BGR2HSV);

        // Split the image into its constituent HSV channels
        ArrayList<Mat> channels = new ArrayList<>();
        Core.split(hsvROI, channels);

        //**TODO Write out the S channel as grayscale.

        // Get the median of the S channel.
        int medianSaturation = ImageUtils.getSingleChannelMedian(channels.get(1));

        RobotLogCommon.d(TAG, "HSV saturation channel median " + medianSaturation);

        // Normalize lighting to a known good value.
        Mat adjustedGray = ImageUtils.adjustGrayscaleMedian(channels.get(1), allianceGrayParameters.median_target);

        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }

}