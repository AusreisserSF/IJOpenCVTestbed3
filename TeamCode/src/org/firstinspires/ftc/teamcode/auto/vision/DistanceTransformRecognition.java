package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.TimeStamp;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;

public class DistanceTransformRecognition {

    private static final String TAG = DistanceTransformRecognition.class.getSimpleName();

    public enum DistanceTransformRecognitionPath {
        COLOR_CHANNEL_BRIGHT_SPOT, COLOR_CHANNEL_PIXEL_COUNT
    }

    private final RobotConstants.Alliance alliance;
    private final String testCaseDirectory;

    public DistanceTransformRecognition(RobotConstants.Alliance pAlliance, String pTestCaseDirectory) {
        alliance = pAlliance;
        testCaseDirectory = pTestCaseDirectory;
    }

    // Use the OpenCV Distance Transform algorithm to detect an object.

    // Returns the result of image analysis.
    //**TODO Use of WatershedParametersFtc is temporary.
    public RobotConstants.RecognitionResults performDistanceTransform(ImageProvider pImageProvider,
                                                                      VisionParameters.ImageParameters pImageParameters,
                                                                      DistanceTransformRecognitionPath pWatershedRecognitionPath,
                                                                      WatershedParametersFtc pWatershedParametersFtc) throws InterruptedException {
        RobotLogCommon.d(TAG, "In DistanceTransformRecognition.performDistanceTransform");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> transformImage = pImageProvider.getImage();
        if (transformImage == null)
            return RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR; // don't crash

        // The image is in BGR order (OpenCV imread from a file).
        String fileDate = TimeStamp.getLocalDateTimeStamp(transformImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(pImageParameters.image_source, testCaseDirectory, fileDate);
        Mat imageROI = ImageUtils.preProcessImage(transformImage.first, outputFilenamePreamble, pImageParameters);
        RobotLogCommon.d(TAG, "Recognition path " + pWatershedRecognitionPath);

        // Adapt the standard example to our environment.
        switch (pWatershedRecognitionPath) {
            // Use a switch by convention in case we have more paths in the future.
            case COLOR_CHANNEL_BRIGHT_SPOT -> {
                Mat distanceTransformImage = getDistanceTransformImage(imageROI, outputFilenamePreamble, pWatershedParametersFtc.watershedDistanceParameters);
                return findBrightSpot(imageROI, distanceTransformImage, outputFilenamePreamble, pWatershedParametersFtc.watershedDistanceParameters);
            }
            case COLOR_CHANNEL_PIXEL_COUNT -> {
                Mat distanceTransformImage = getDistanceTransformImage(imageROI, outputFilenamePreamble, pWatershedParametersFtc.watershedDistanceParameters);
                return colorChannelPixelCount(imageROI, distanceTransformImage, outputFilenamePreamble, pWatershedParametersFtc.watershedDistanceParameters);
            }
            default -> throw new AutonomousRobotException(TAG, "Unrecognized recognition path");
        }
    }

    private Mat getDistanceTransformImage(Mat pImageROI, String pOutputFilenamePreamble, WatershedParametersFtc.WatershedDistanceParameters pWwatershedDistanceParameters) {

        // Use the grayscale and pixel count criteria parameters for the current alliance.
        VisionParameters.GrayParameters allianceGrayParameters;
        switch (alliance) {
            case RED -> allianceGrayParameters = pWwatershedDistanceParameters.redGrayParameters;
            case BLUE -> allianceGrayParameters = pWwatershedDistanceParameters.blueGrayParameters;
            default -> throw new AutonomousRobotException(TAG, "distance transform requires an alliance selection");
        }

        //##PY Apply a sharpening kernel to the color image.
        Mat sharp = sharpen(pImageROI, pOutputFilenamePreamble);

        // Split the BGR image into its components; see the comments above the method.
        Mat split = splitAndInvertChannels(sharp, alliance, allianceGrayParameters, pOutputFilenamePreamble);

        // Normalize lighting to a known good value.
        Mat adjustedGray = ImageUtils.adjustGrayscaleMedian(split, allianceGrayParameters.median_target);

        // Follow medium.com and threshold the grayscale (in our case adjusted).
        // Threshold the image: set pixels over the threshold value to white.
        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(adjustedGray, thresholded,
                Math.abs(allianceGrayParameters.threshold_low),    // threshold value
                255,   // white
                Imgproc.THRESH_BINARY); // thresholding type
        RobotLogCommon.v(TAG, "Threshold values: low " + allianceGrayParameters.threshold_low + ", high 255");

        String thrFilename = pOutputFilenamePreamble + "_THR.png";
        Imgcodecs.imwrite(thrFilename, thresholded);
        RobotLogCommon.d(TAG, "Writing " + thrFilename);

        // Follow medium.com and perform two morphological openings on the grayscale image.
        Mat opening = new Mat();
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(thresholded, opening, Imgproc.MORPH_OPEN, kernel, new Point(-1, -1), 2);

        String openFilename = pOutputFilenamePreamble + "_OPEN.png";
        Imgcodecs.imwrite(openFilename, opening);
        RobotLogCommon.d(TAG, "Writing " + openFilename);

        // The distance transform identifies regions that are likely to be in
        // the foreground.
        //! [dist]
        // Perform the distance transform algorithm. Imgproc.DIST_L2
        // is a flag for Euclidean distance. Output is 32FC1.
        Mat dist = new Mat();
        Imgproc.distanceTransform(opening, dist, Imgproc.DIST_L2, 3);

        Core.normalize(dist, dist, 0.0, 255.0, Core.NORM_MINMAX);
        Mat dist_8u = new Mat();
        dist.convertTo(dist_8u, CvType.CV_8U);

        // Output the transformed image.
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_DIST.png", dist_8u);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_DIST.png");
        //! [dist]

        return dist_8u;
    }

    private RobotConstants.RecognitionResults findBrightSpot(Mat pImageROI, Mat pDistanceImage,
                                                             String pOutputFilenamePreamble,
                                                             WatershedParametersFtc.WatershedDistanceParameters pWwatershedDistanceParameters) {
        Core.MinMaxLocResult brightResult = Core.minMaxLoc(pDistanceImage);
        VisionParameters.GrayParameters allianceGrayParameters;
        switch (alliance) {
            case RED -> allianceGrayParameters = pWwatershedDistanceParameters.redGrayParameters;
            case BLUE -> allianceGrayParameters = pWwatershedDistanceParameters.blueGrayParameters;
            default ->
                    throw new AutonomousRobotException(TAG, "colorChannelPixelCountPath requires an alliance selection");
        }

        RobotLogCommon.d(TAG, "Bright spot location " + brightResult.maxLoc + ", value " + brightResult.maxVal);

        Mat brightSpotOut = pImageROI.clone();
        Imgproc.circle(brightSpotOut, brightResult.maxLoc, 10, new Scalar(0, 255, 0));

        String brightSpotFilename = pOutputFilenamePreamble + "_BRIGHT.png";
        RobotLogCommon.d(TAG, "Writing " + brightSpotFilename);
        Imgcodecs.imwrite(brightSpotFilename, brightSpotOut);

        // If the bright spot is under the threshold then assume no Team Prop is present.
        if (brightResult.maxVal < allianceGrayParameters.threshold_low) {
            RobotLogCommon.d(TAG, "Bright spot value was under the threshold");
        }

        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }

    //**TODO Need pixel count limits by alliance.
    private RobotConstants.RecognitionResults colorChannelPixelCount(Mat pImageROI, Mat pDistanceImage,
                                                                     String pOutputFilenamePreamble,
                                                                     WatershedParametersFtc.WatershedDistanceParameters pWwatershedDistanceParameters) {
        //! [peaks]
        // Threshold to obtain the peaks.
        // These will be the markers for the foreground objects.
        //##PY Since we've already normalized to a range of 0 - 255 we can replace this
        // Imgproc.threshold(dist, dist, 0.4, 1.0, Imgproc.THRESH_BINARY);
        //**TODO foreground threshold low limit is hardcoded.
        Mat sure_fg = new Mat();
        Imgproc.threshold(pDistanceImage, sure_fg, 100, 255, Imgproc.THRESH_BINARY);

        // Find the sure foreground objects.
        //## medium.com uses connectedComponents; the standard example uses findContours.
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(sure_fg, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        //**TODO Get the centroid and log.
        //#PY added - output the contours.
        Mat contoursOut = pImageROI.clone();
        ShapeDrawing.drawShapeContours(contours, contoursOut);
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_CON.png", contoursOut);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_CON.png");
        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
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

    //## Imported from IJCenterStageVision.
    //## This sharpening filter makes a difference in marginal cases.
    // From OpencvTestbed3 (cpp) GrayscaleTechnique
    // From https://stackoverflow.com/questions/27393401/opencv-in-java-for-image-filtering
    private Mat sharpen(Mat pDullMat, String pOutputFilenamePreamble) {
        int kernelSize = 3;
        Mat kernel = new Mat(kernelSize, kernelSize, CvType.CV_32F) {
            {
                put(0, 0, 0);
                put(0, 1, -1);
                put(0, 2, 0);

                put(1, 0, -1);
                put(1, 1, 5);
                put(1, 2, -1);

                put(2, 0, 0);
                put(2, 1, -1);
                put(2, 2, 0);
            }
        };

        Mat sharpMat = new Mat();
        Imgproc.filter2D(pDullMat, sharpMat, -1, kernel);

        String sharpFilename = pOutputFilenamePreamble + "_SHARP.png";
        RobotLogCommon.d(TAG, "Writing " + sharpFilename);
        Imgcodecs.imwrite(sharpFilename, sharpMat);

        return sharpMat;
    }

}