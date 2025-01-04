package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.TimeStamp;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.xml.LABTesterParameters;
import org.firstinspires.ftc.teamcode.auto.xml.VisionParameters;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;

public class LABRecognition {

    private static final String TAG = LABRecognition.class.getSimpleName();

    public enum LABRecognitionPath {
        A_CHANNEL_GRAYSCALE, B_CHANNEL_GRAYSCALE, LAB_COLOR
    }

    private final String testCaseDirectory;

    public LABRecognition(String pTestCaseDirectory) {
        testCaseDirectory = pTestCaseDirectory;
    }

    public RobotConstants.RecognitionResults testLAB(ImageProvider pImageProvider,
                                                     VisionParameters.ImageParameters pImageParameters,
                                                     LABTesterParameters pLABTesterParameters,
                                                     LABRecognition.LABRecognitionPath pLABRecognitionPath) throws InterruptedException {

        RobotLogCommon.d(TAG, "In LABRecognition.testLAB");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> originalImage = pImageProvider.getImage();
        if (originalImage == null)
            return RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR; // don't crash

        // The image is in BGR order (OpenCV imread from a file).
        Mat labImage = new Mat();
        Imgproc.cvtColor(originalImage.first, labImage, Imgproc.COLOR_BGR2Lab);

        String fileDate = TimeStamp.getLocalDateTimeStamp(originalImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(pImageParameters.image_source, testCaseDirectory, fileDate);
        Mat imageROI = ImageUtils.preProcessImage(labImage, outputFilenamePreamble, pImageParameters);

        //**TODO Switch on A_CHANNEL_GRAYSCALE, B_CHANNEL_GRAYSCALE, LAB_COLOR
        switch (pLABRecognitionPath) {
            case LAB_COLOR -> {
                return labColor(imageROI, pLABTesterParameters, outputFilenamePreamble);
            }
            case A_CHANNEL_GRAYSCALE -> {
                //**TODO
            }
            case B_CHANNEL_GRAYSCALE -> {
                //**TODO
            }
            default -> throw new AutonomousRobotException(TAG, "Unrecognized LAB path " + pLABRecognitionPath);
        }

        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }
    
    private RobotConstants.RecognitionResults labColor(Mat pImageROI, LABTesterParameters pLABTesterParameters,
                                                       String outputFilenamePreamble) {

        // The original L*a*b* values have already been converted to
        // their OpenCV equivalents.
        Mat thresholded = new Mat();
        Core.inRange(pImageROI, new Scalar(pLABTesterParameters.labParameters.L_star_low,
                        pLABTesterParameters.labParameters.a_star_low,
                        pLABTesterParameters.labParameters.b_star_low),
                new Scalar(pLABTesterParameters.labParameters.L_star_high,
                        pLABTesterParameters.labParameters.a_star_high,
                        pLABTesterParameters.labParameters.b_star_high),
                thresholded);

        Imgcodecs.imwrite(outputFilenamePreamble + "_THR.png", thresholded);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_THR.png");
        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }

//        // Extract the "a" and then use it as grayscale.
//        Mat selectedChannel = new Mat();
//        Core.extractChannel(imageROI, selectedChannel, 1);
//
//        // Write out the "a" channel as grayscale.
//        Imgcodecs.imwrite(outputFilenamePreamble + "_A_CHANNEL.png", selectedChannel);
//        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_A_CHANNEL.png");
//
//        // The "b" channel.
//        Core.extractChannel(imageROI, selectedChannel, 2); // "b" channel
//
//        // Write out the "a" channel as grayscale.
//        Imgcodecs.imwrite(outputFilenamePreamble + "_B_CHANNEL.png", selectedChannel);
//        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_B_CHANNEL.png");

}