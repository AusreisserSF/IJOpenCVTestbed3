package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.TimeStamp;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.xml.GoldCubeParameters;
import org.firstinspires.ftc.teamcode.auto.xml.LABTesterParameters;
import org.firstinspires.ftc.teamcode.auto.xml.VisionParameters;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.Optional;

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

        // Full-on LAB color recognition on red.
        //!! Adjust the LAB values.
        // From https://docs.opencv.org/3.4/de/d25/imgproc_color_conversions.html
        //    8-bit images: L←L∗255/100,a←a+128,b←b+128
        // Low L 25.0 -> 63.75; a* 50.0 -> 178; b* 25.0 -> 153
        // High L 50.0 -> 127.5; a* 75.0 -> 203; b* 60.0 -> 188
             Mat thresholded = new Mat();
             Core.inRange(labImage, new Scalar(63.75, 178, 153), new Scalar(127.5, 203, 188), thresholded);

            Imgcodecs.imwrite(outputFilenamePreamble + "_THR.png", thresholded);
            RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_THR.png");

        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }

}