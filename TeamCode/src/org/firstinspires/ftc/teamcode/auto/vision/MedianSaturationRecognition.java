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
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.ArrayList;

//**TODO Not "median" but "mean".
//**TODO Variation of distance transform?
//**TODO Which is the major category - pixel count, distance?
//**TODO Before you do mean saturation you can do distance, then pixel count or bright spot.
public class MedianSaturationRecognition {

    private static final String TAG = MedianSaturationRecognition.class.getSimpleName();

    public enum MedianSaturationRecognitionPath {
        SATURATION_MEDIAN //**TODO Wrong name

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
            case SATURATION_MEDIAN -> {
                return evaluateHSVSaturationChannel(imageROI, outputFilenamePreamble, pMedianSaturationParameters.watershedDistanceParameters);
            }
            default -> throw new AutonomousRobotException(TAG, "Unrecognized recognition path");
        }
    }

    private RobotConstants.RecognitionResults evaluateHSVSaturationChannel(Mat pImageROI, String pOutputFilenamePreamble, WatershedParametersFtc.WatershedDistanceParameters pWatershedDistanceParameters) {

        // Use the grayscale parameters for the current alliance.
        VisionParameters.GrayParameters allianceGrayParameters;
        switch (alliance) {
            case RED -> allianceGrayParameters = pWatershedDistanceParameters.redGrayParameters;
            case BLUE -> allianceGrayParameters = pWatershedDistanceParameters.blueGrayParameters;
            default ->
                    throw new AutonomousRobotException(TAG, "colorChannelPixelCountPath requires an alliance selection");
        }

        //**TODO Probably don't need sharpening
        //Mat sharp = ImageUtils.sharpen(pImageROI, pOutputFilenamePreamble);

        // Convert to HSV and split the channels.
        // We're on the HSV path.
        Mat hsvROI = new Mat();
        Imgproc.cvtColor(pImageROI, hsvROI, Imgproc.COLOR_BGR2HSV);

        //**TODO Adjust the median saturation and value of the HSV image
        // then split.

        // Split the image into its constituent HSV channels
        ArrayList<Mat> channels = new ArrayList<>();
        Core.split(hsvROI, channels);

        // Normalize lighting to a known good value.
        //Mat adjustedSat = ImageUtils.adjustGrayscaleMedian(channels.get(1), allianceGrayParameters.median_target);

        // Write out the S channel as grayscale.
        String satFilename = pOutputFilenamePreamble + "_SAT.png";
        Imgcodecs.imwrite(satFilename, channels.get(1));
        RobotLogCommon.d(TAG, "Writing " + satFilename);

        // Get the mean of the S channel.
        Scalar meanSaturation = Core.mean(channels.get(1));

        RobotLogCommon.d(TAG, "HSV saturation channel mean " + meanSaturation);

        // Do the same for the value channel.
        // Normalize lighting to a known good value.
        //Mat adjustedVal = ImageUtils.adjustGrayscaleMedian(channels.get(2), allianceGrayParameters.median_target);

        // Write out the V channel as grayscale.
        /*
        String valFilename = pOutputFilenamePreamble + "_VAL.png";
        Imgcodecs.imwrite(valFilename, channels.get(2));
        RobotLogCommon.d(TAG, "Writing " + valFilename);

        // Get the median of the V channel.
        int medianValue = ImageUtils.getSingleChannelMedian(channels.get(2));

        RobotLogCommon.d(TAG, "HSV value channel median " + medianValue);
        */

        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }

}