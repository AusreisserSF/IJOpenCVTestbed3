package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.TimeStamp;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.xml.GoldCubeParameters;
import org.firstinspires.ftc.teamcode.auto.xml.VisionParameters;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.Optional;

public class LABRecognition {

    private static final String TAG = LABRecognition.class.getSimpleName();

    public enum LABRecognitionPath {
        LAB
    }

    private final String testCaseDirectory;

    public LABRecognition(String pTestCaseDirectory) {
        testCaseDirectory = pTestCaseDirectory;
    }

    public RobotConstants.RecognitionResults testLAB(ImageProvider pImageProvider,
                                                     VisionParameters.ImageParameters pImageParameters,
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

        // Extract the "a" and then use it as grayscale.
        Mat selectedChannel = new Mat();
        Core.extractChannel(imageROI, selectedChannel, 1);

        // Write out the "a" channel as grayscale.
        Imgcodecs.imwrite(outputFilenamePreamble + "_A_CHANNEL.png", selectedChannel);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_A_CHANNEL.png");

        // The "b" channel.
        Core.extractChannel(imageROI, selectedChannel, 2); // "b" channel

        // Write out the "a" channel as grayscale.
        Imgcodecs.imwrite(outputFilenamePreamble + "_B_CHANNEL.png", selectedChannel);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_B_CHANNEL.png");

        //**TODO Mat thresholded = ImageUtils.performThresholdOnGray(selectedChannel, pOutputFilenamePreamble, pGoldCubeParameters.grayscaleParameters.median_target, pGoldCubeParameters.grayscaleParameters.threshold_low);

        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }

}