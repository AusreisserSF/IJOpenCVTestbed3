package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.TimeStamp;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.xml.VisionParameters;
import org.firstinspires.ftc.teamcode.auto.xml.WatershedParametersFtc;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;

public class WatershedRecognitionFtc {

    private static final String TAG = WatershedRecognitionFtc.class.getSimpleName();

    public enum WatershedRecognitionPath {
        WATERSHED_FTC
    }

    private final RobotConstants.Alliance alliance;
    private final String testCaseDirectory;

    public WatershedRecognitionFtc(RobotConstants.Alliance pAlliance, String pTestCaseDirectory) {
        alliance = pAlliance;
        testCaseDirectory = pTestCaseDirectory;
    }

    // See comments in WatershedRecognitionStd for background.

    //##From the testing so far it's not at all clear that the Watershed technique
    // will be useful in FTC; the watershed output of the blue team prop in CenterStage
    // merges the team prop and the blue spike - but the output of the distanceTransform
    // does look promising - it eliminates the blue spike. And it's not clear how you
    // would actually use the final output of the watershed in FTC.

    // Returns the result of image analysis.
    public RobotConstants.RecognitionResults performWatershedFtc(ImageProvider pImageProvider,
                                                                 VisionParameters.ImageParameters pImageParameters,
                                                                 WatershedRecognitionPath pWatershedRecognitionPath,
                                                                 WatershedParametersFtc pWatershedParametersFtc) throws InterruptedException {
        RobotLogCommon.d(TAG, "In WatershedRecognitionFtc.performWatershedFtc");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> watershedImage = pImageProvider.getImage();
        if (watershedImage == null)
            return RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR; // don't crash

        // The image is in BGR order (OpenCV imread from a file).
        String fileDate = TimeStamp.getLocalDateTimeStamp(watershedImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(pImageParameters.image_source, testCaseDirectory, fileDate);
        Mat imageROI = ImageUtils.preProcessImage(watershedImage.first, outputFilenamePreamble, pImageParameters);
        RobotLogCommon.d(TAG, "Recognition path " + pWatershedRecognitionPath);

        // Adapt the standard example to our environment.
        switch (pWatershedRecognitionPath) {
            case WATERSHED_FTC -> {
                return watershedHybridFTC(imageROI, outputFilenamePreamble, pWatershedParametersFtc.watershedDistanceParameters);
            }
            default -> throw new AutonomousRobotException(TAG, "Unrecognized recognition path");
        }
    }

    private RobotConstants.RecognitionResults watershedHybridFTC(Mat pImageROI, String pOutputFilenamePreamble, WatershedParametersFtc.WatershedDistanceParameters pWatershedDistanceParameters) {

        // Use the grayscale and pixel count criteria parameters for the current alliance.
        VisionParameters.GrayParameters allianceGrayParameters;
        switch (alliance) {
            case RED -> allianceGrayParameters = pWatershedDistanceParameters.redGrayParameters;
            case BLUE -> allianceGrayParameters = pWatershedDistanceParameters.blueGrayParameters;
            default ->
                    throw new AutonomousRobotException(TAG, "colorChannelPixelCountPath requires an alliance selection");
        }

        //##PY The Laplacian filtering and the sharpening in the OpenCV example
        // do make a difference but they can be replaced by a simple sharpening
        // kernel.
        Mat sharp = ImageUtils.sharpen(pImageROI, pOutputFilenamePreamble);

        //**TODO RE-TEST Extract the opposing alliance's BGR channel from the image and invert.
        Mat split = ImageUtils.extractAndInvertChannel(sharp, alliance, allianceGrayParameters, pOutputFilenamePreamble);

        // Normalize lighting to a known good value.
        Mat adjustedGray = ImageUtils.adjustGrayscaleMedian(split, allianceGrayParameters.median_target);

        return WatershedRecognitionStd.prepareAndExecuteWatershed(adjustedGray, pImageROI, sharp, allianceGrayParameters.threshold_low, Imgproc.THRESH_BINARY, pOutputFilenamePreamble);
    }

}