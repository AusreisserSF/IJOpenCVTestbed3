package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.TimeStamp;
import org.firstinspires.ftc.teamcode.auto.DebugImageCommon;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.xml.GoldCubeParameters;
import org.firstinspires.ftc.teamcode.auto.xml.RedAllianceSampleParameters;
import org.firstinspires.ftc.teamcode.auto.xml.VisionParameters;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.Optional;

import static org.firstinspires.ftc.teamcode.auto.vision.RedAllianceSampleRecognition.RecognitionPath.RED_CHANNEL_GRAYSCALE;

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
                                                                      RedAllianceSampleParameters pRedAllianceSampleParameters,
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

         return redChannelPath(imageROI, outputFilenamePreamble, pRedAllianceSampleParameters);
    }

    private RobotConstants.RecognitionResults redChannelPath(Mat pImageROI, String pOutputFilenamePreamble,
                                                             RedAllianceSampleParameters pRedAllianceSampleParameters) {

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
                pRedAllianceSampleParameters.redGrayParameters.median_target,
                pRedAllianceSampleParameters.redGrayParameters.threshold_low,
                pOutputFilenamePreamble, "");

        //**TODO Filter out artifacts using the method of IJIntoTheDeepVision.sampleRecognition().
        //**TODO As part of filtering you'll have to get an idea of the proximity of the
        // camera to the samples; this will affect the pixel counts.
        /*
                       // Sanitize the thresholded red alliance samples by eliminating contours
                // that are below the minimum area threshold.
                ImageUtils.FilteredContoursRecord filteredA = ImageUtils.filterContours(allianceBinaryMorphed, pImageROI.rows(), pImageROI.cols(),
                        sampleParameters.sampleCriteria.min_sample_area / 2.0,
                        pOutputFilenamePreamble, "_A");

                // Capture statistics.
                SampleRecognitionStatistics.numUnfilteredAllianceContours = filteredA.numUnfilteredContours;
                SampleRecognitionStatistics.numFilteredAllianceContours = filteredA.numFilteredContours;

         */

        //**TODO Classify the remaining samples based on the rectangle filtering
        // from pyimagesearch.

        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }

}