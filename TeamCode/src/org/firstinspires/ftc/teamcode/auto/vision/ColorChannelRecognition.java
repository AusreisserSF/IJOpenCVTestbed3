package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.TimeStamp;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.xml.VisionParameters;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;

// Investigate channel splitting of an HSV image as described in --
// https://stackoverflow.com/questions/75759216/how-do-i-reliably-classify-colours-as-red-yellow-or-other-colours-in-opencv

public class ColorChannelRecognition {

    private static final String TAG = ColorChannelRecognition.class.getSimpleName();

    private final RobotConstants.Alliance alliance;
    private final String testCaseDirectory;

    public ColorChannelRecognition(RobotConstants.Alliance pAlliance, String pTestCaseDirectory) {
        alliance = pAlliance;
        testCaseDirectory = pTestCaseDirectory;
    }

    // Returns the result of image analysis.
    public RobotConstants.RecognitionResults splitColorChannels(ImageProvider pImageProvider,
                                                                VisionParameters.ImageParameters pImageParameters) throws InterruptedException {
        RobotLogCommon.d(TAG, "In ColorChannelRecognition.splitColorChannels");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> inputImage = pImageProvider.getImage();
        if (inputImage == null)
            return RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR; // don't crash

        // The image is in BGR order (OpenCV imread from a file).
        String fileDate = TimeStamp.getLocalDateTimeStamp(inputImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(pImageParameters.image_source, testCaseDirectory, fileDate);
        Mat imageROI = ImageUtils.preProcessImage(inputImage.first, outputFilenamePreamble, pImageParameters);

        Mat hsvROI = new Mat();
        Imgproc.cvtColor(imageROI, hsvROI, Imgproc.COLOR_BGR2HSV);
        performHSVSplit(hsvROI, outputFilenamePreamble);

        Mat labROI = new Mat();
        Imgproc.cvtColor(imageROI, labROI, Imgproc.COLOR_BGR2Lab);
        performLABSplit(labROI, outputFilenamePreamble);

        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }

    private void performHSVSplit(Mat pHSVROI, String pOutputFilenamePreamble) {
        Mat hueChannel = new Mat();
        Core.extractChannel(pHSVROI, hueChannel, 0);

        // Write out the H channel as grayscale.
        String hueFilename = pOutputFilenamePreamble + "_HUE.png";
        Imgcodecs.imwrite(hueFilename, hueChannel);
        RobotLogCommon.d(TAG, "Writing " + hueFilename);

        Mat saturationChannel = new Mat();
        Core.extractChannel(pHSVROI, saturationChannel, 1);

        // Write out the S channel as grayscale.
        String satFilename = pOutputFilenamePreamble + "_SAT.png";
        Imgcodecs.imwrite(satFilename, saturationChannel);
        RobotLogCommon.d(TAG, "Writing " + satFilename);

        Mat valueChannel = new Mat();
        Core.extractChannel(pHSVROI, valueChannel, 2);

        // Write out the V channel as grayscale.
        String valFilename = pOutputFilenamePreamble + "_VAL.png";
        Imgcodecs.imwrite(valFilename, valueChannel);
        RobotLogCommon.d(TAG, "Writing " + valFilename);
    }

    private void performLABSplit(Mat pLABROI, String pOutputFilenamePreamble) {
        Mat lChannel = new Mat();
        Core.extractChannel(pLABROI, lChannel, 0);

        // Write out the L channel as grayscale.
        String lFilename = pOutputFilenamePreamble + "_L.png";
        Imgcodecs.imwrite(lFilename, lChannel);
        RobotLogCommon.d(TAG, "Writing " + lFilename);

        Mat aChannel = new Mat();
        Core.extractChannel(pLABROI, aChannel, 1);

        // Write out the A channel as grayscale.
        String aFilename = pOutputFilenamePreamble + "_A.png";
        Imgcodecs.imwrite(aFilename, aChannel);
        RobotLogCommon.d(TAG, "Writing " + aFilename);

        Mat bChannel = new Mat();
        Core.extractChannel(pLABROI, bChannel, 2);

        // Write out the V channel as grayscale.
        String bFilename = pOutputFilenamePreamble + "_B.png";
        Imgcodecs.imwrite(bFilename, bChannel);
        RobotLogCommon.d(TAG, "Writing " + bFilename);
    }

}