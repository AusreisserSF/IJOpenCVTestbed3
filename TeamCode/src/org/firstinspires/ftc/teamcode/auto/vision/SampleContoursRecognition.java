package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.TimeStamp;
import org.firstinspires.ftc.teamcode.auto.DebugImageCommon;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.xml.SampleContoursParameters;
import org.firstinspires.ftc.teamcode.auto.xml.VisionParameters;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;

public class SampleContoursRecognition {

    private static final String TAG = SampleContoursRecognition.class.getSimpleName();

    public enum SampleContoursRecognitionPath {
        RED_CHANNEL_GRAYSCALE, COLOR
    }

    private final String testCaseDirectory;
    private final RobotConstants.Alliance alliance;

    public SampleContoursRecognition(String pTestCaseDirectory, RobotConstants.Alliance pAlliance) {
        testCaseDirectory = pTestCaseDirectory;
        alliance = pAlliance;
    }


    // Returns the result of image analysis.
     public RobotConstants.RecognitionResults recognizeSampleContours(ImageProvider pImageProvider,
                                                                      VisionParameters.ImageParameters pImageParameters,
                                                                      SampleContoursParameters pSampleContoursParameters,
                                                                      SampleContoursRecognitionPath pSampleContoursRecognitionPath) throws InterruptedException {

        RobotLogCommon.d(TAG, "In SampleContours.recognizeSampleContours");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> sampleImage = pImageProvider.getImage();
        if (sampleImage == null)
            return RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR; // don't crash

        // The image is in BGR order (OpenCV imread from a file).
        String fileDate = TimeStamp.getLocalDateTimeStamp(sampleImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(pImageParameters.image_source, testCaseDirectory, fileDate);
        Mat imageROI = ImageUtils.preProcessImage(sampleImage.first, outputFilenamePreamble, pImageParameters);

        RobotLogCommon.d(TAG, "Recognition path " + pSampleContoursRecognitionPath);
        switch (pSampleContoursRecognitionPath) {
            case RED_CHANNEL_GRAYSCALE -> {
                return redChannelPath(imageROI, outputFilenamePreamble, pSampleContoursParameters);
            }
            case COLOR -> {
                return colorPath(imageROI, outputFilenamePreamble, pSampleContoursParameters);
            }
            default -> throw new AutonomousRobotException(TAG, "Unrecognized recognition path");
        }
    }

    private RobotConstants.RecognitionResults redChannelPath(Mat pImageROI, String pOutputFilenamePreamble,
                                                             SampleContoursParameters pSampleContourParameters) {

        // Extract the red channel and then use it as grayscale.
        Mat selectedChannel = new Mat();
        Core.extractChannel(pImageROI, selectedChannel, 2);

        // Write out the red channel as grayscale.
        if (RobotLogCommon.isLoggable(RobotLogCommon.CommonLogLevel.d)) {
            DebugImageCommon.writeImage(pOutputFilenamePreamble + "_RED_CHANNEL.png", selectedChannel);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_RED_CHANNEL.png");
        }

        Mat thresholded = ImageUtils.performThresholdOnGray(selectedChannel,
                pSampleContourParameters.rgbChannelGrayscaleParameters.redGrayParameters.median_target,
                pSampleContourParameters.rgbChannelGrayscaleParameters.redGrayParameters.threshold_low,
                pOutputFilenamePreamble, "");

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(thresholded, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (RobotLogCommon.isLoggable(RobotLogCommon.CommonLogLevel.d)) {
            Mat contoursOut = pImageROI.clone();
            ShapeDrawing.drawShapeContours(contours, contoursOut);
            Imgcodecs.imwrite(pOutputFilenamePreamble + "CON.png", contoursOut);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "CON.png");
        }

        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }

    // Analyze a color image.
    public RobotConstants.RecognitionResults colorPath(Mat pImageROI, String pOutputFilenamePreamble,
                                                             SampleContoursParameters pSampleContoursParameters) {

        Mat thresholded = ImageUtils.performInRange(pImageROI, pSampleContoursParameters.hsvColorParameters.blueHSVParameters, pOutputFilenamePreamble, "");

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(thresholded, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (RobotLogCommon.isLoggable(RobotLogCommon.CommonLogLevel.d)) {
            Mat contoursOut = pImageROI.clone();
            ShapeDrawing.drawShapeContours(contours, contoursOut);
            Imgcodecs.imwrite(pOutputFilenamePreamble + "CON.png", contoursOut);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "CON.png");
        }

        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }

}