package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.TimeStamp;
import org.firstinspires.ftc.teamcode.auto.xml.WatershedParametersFtc;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.xml.VisionParameters;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

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

    // Use the OpenCV Watershed algorithm with FTC images. The code here is
    // based on a combination of the official c++ example at --
    // https://docs.opencv.org/4.x/d2/dbd/tutorial_distance_transform.html,
    // which is implemented in this project as WatershedRecognitionStd,
    //
    // a Python solution at --
    // https://medium.com/@jaskaranbhatia/exploring-image-segmentation-techniques-watershed-algorithm-using-opencv-9f73d2bc7c5a
    //
    // and a c++ solution from --
    //**TODO Remove?? OpenCV 3 Computer Vision Application Programming Cookbook,
    // Third Edition, by Robert Laganiere; pg. 162.
    // Note how he uses erosion to create the sure foreground and dilation to create
    // the sure background, how he adds foreground and background to get his markers,
    // and how he outputs the final results, including just the boundaries.

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

    //**TODO Keep (somewhere) for some good techniques - such as drawing
    // just the -1 boundaries.
    private RobotConstants.RecognitionResults watershedLaganiere(Mat pImageROI, String pOutputFilenamePreamble, WatershedParametersFtc.WatershedDistanceParameters pWatershedDistanceParameters) {

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

        // Split the BGR image into its components; see the comments above the method.
        Mat split = splitAndInvertChannels(sharp, alliance, allianceGrayParameters, pOutputFilenamePreamble);

        // Normalize lighting to a known good value.
        Mat adjustedGray = ImageUtils.adjustGrayscaleMedian(split, allianceGrayParameters.median_target);

        // Follow medium.com and threshold the grayscale (in our case adjusted)
        // to start the segmentation of the image.
        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(adjustedGray, thresholded,
                Math.abs(allianceGrayParameters.threshold_low),    // threshold value
                255,   // white
                Imgproc.THRESH_BINARY); // thresholding type
        RobotLogCommon.v(TAG, "Threshold values: low " + allianceGrayParameters.threshold_low + ", high 255");

        String thrFilename = pOutputFilenamePreamble + "_THR.png";
        Imgcodecs.imwrite(thrFilename, thresholded);
        RobotLogCommon.d(TAG, "Writing " + thrFilename);

        // Laganiere gets his foreground by eroding the thresholded image.
        Mat sure_fg_lg = new Mat();
        Mat erodeKernel_lg = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.erode(thresholded, sure_fg_lg, erodeKernel_lg, new Point(-1, -1), 4);

        String erodedFilename = pOutputFilenamePreamble + "_FG_LG.png";
        Imgcodecs.imwrite(erodedFilename, sure_fg_lg);
        RobotLogCommon.d(TAG, "Writing " + erodedFilename);

        // Laganiere gets his background by dilating the thresholded
        // binary image 4 times and then thresholding again.
        Mat sure_bg_lg = new Mat();
        Mat dilateKernel_lg = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.dilate(thresholded, sure_bg_lg, dilateKernel_lg, new Point(-1, -1), 4);

        // Threshold again but invert this time so that all zero bits
        // become gray (128) for visibility.
        Imgproc.threshold(sure_bg_lg, sure_bg_lg, 1, 128,
                Imgproc.THRESH_BINARY_INV); // thresholding type

        String bg_lgFilename = pOutputFilenamePreamble + "_BG_LG.png";
        Imgcodecs.imwrite(bg_lgFilename, sure_bg_lg);
        RobotLogCommon.d(TAG, "Writing " + bg_lgFilename);

        // Laganiere does not do a distance transform.
        // Laganiere obtains the markers by adding the sure foreground
        // and the sure background.
        //**TODO However, I don't think it will work to use the same
        // value for the sure foreground (255) when the objects are
        // touching, e.g. cards, coins, pills.
        Mat markers_lg = new Mat();
        Core.add(sure_fg_lg, sure_bg_lg, markers_lg);

        // Output the Laganiere markers.
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_MARK_LG.png", markers_lg);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_MARK_LG.png");

        // Convert the Laganiere markers to 32 bits as required by the watershed.
        Mat markers32_lg = new Mat();
        markers_lg.convertTo(markers32_lg, CvType.CV_32S);

        //! [watershed]
        // Perform the watershed algorithm
        Imgproc.watershed(sharp, markers32_lg);

        // Create the result image, which in this case will show the
        // outlines of the labeled objects.
        Mat dst = Mat.zeros(markers32_lg.size(), CvType.CV_32S);
        int[] dstData = new int[(int) (dst.total() * dst.channels())];
        dst.get(0, 0, dstData);

        int[] markersData = new int[(int) (markers32_lg.total() * markers32_lg.channels())];
        markers32_lg.get(0, 0, markersData);
        for (int i = 0; i < markers32_lg.rows(); i++) {
            for (int j = 0; j < markers32_lg.cols(); j++) {
                int index = markersData[i * markers32_lg.cols() + j];
                // watershed boundaries are -1.
                if (index == -1)
                    dstData[(i * dst.cols() + j)] = 255;
            }
        }

        dst.put(0, 0, dstData);
        dst.convertTo(dst, CvType.CV_8UC3);

        // Visualize the final image
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_WS_LG.png", dst);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_WS_LG.png");
        //! [watershed]

        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
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

        // Split the BGR image into its components; see the comments above the method.
        Mat split = splitAndInvertChannels(sharp, alliance, allianceGrayParameters, pOutputFilenamePreamble);

        // Normalize lighting to a known good value.
        Mat adjustedGray = ImageUtils.adjustGrayscaleMedian(split, allianceGrayParameters.median_target);

        return WatershedRecognitionStd.prepareAndExecuteWatershed(adjustedGray, pImageROI, sharp, allianceGrayParameters.threshold_low, pOutputFilenamePreamble);
    }

    //## Imported from IJCenterStageVision.
    // Split the original image ROI into its BGR channels. The alliance
    // determines which channel to pre-process and return. For better
    // contrast the RED alliance uses the inversion of the blue channel
    // and the BLUE alliance uses the inversion of the red channel.
    private Mat splitAndInvertChannels(Mat pImageROI, RobotConstants.Alliance pAlliance, VisionParameters.GrayParameters pGrayParameters, String pOutputFilenamePreamble) {
        ArrayList<Mat> channels = new ArrayList<>(3);
        Core.split(pImageROI, channels); // red or blue channel. B = 0, G = 1, R = 2
        Mat selectedChannel;
        switch (pAlliance) {
            case RED -> {
                // The inversion of the blue channel gives better contrast
                // than the red channel.
                selectedChannel = channels.get(0);
                Core.bitwise_not(selectedChannel, selectedChannel);
                Imgcodecs.imwrite(pOutputFilenamePreamble + "_BLUE_INVERTED.png", selectedChannel);
                RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_BLUE_INVERTED.png");
            }
            case BLUE -> {
                // The inversion of the red channel gives better contrast
                // than the blue channel.
                selectedChannel = channels.get(2);
                Core.bitwise_not(selectedChannel, selectedChannel);
                Imgcodecs.imwrite(pOutputFilenamePreamble + "_RED_INVERTED.png", selectedChannel);
                RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_RED_INVERTED.png");
            }
            default -> throw new AutonomousRobotException(TAG, "Alliance must be RED or BLUE");
        }

        // Always adjust the grayscale.
        Mat adjustedGray = ImageUtils.adjustGrayscaleMedian(selectedChannel,
                pGrayParameters.median_target);

        Imgproc.erode(adjustedGray, adjustedGray, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));
        Imgproc.dilate(adjustedGray, adjustedGray, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3)));

        return adjustedGray;
    }

}