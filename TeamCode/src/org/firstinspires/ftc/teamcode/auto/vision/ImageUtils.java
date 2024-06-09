// Partial port of ImageUtils.cpp: only those functions that are
// needed for ring recognition.
package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.*;
import java.util.stream.Collectors;

// 5/20/2023 Added support for not writing out files via a null test
// on the output file name. This feature is used by the EasyOpenCV
// calibration tests, which display their output on the Driver Station.
public class ImageUtils {

    public static final String TAG = ImageUtils.class.getSimpleName();

    // Load an image.
    public static Mat loadImage(String pInputFilename) {
        RobotLogCommon.i(TAG, "File name " + pInputFilename);

        // Sanity check on the file name extension.
        if (!(pInputFilename.endsWith(".png") || pInputFilename.endsWith(".jpg")))
            throw new AutonomousRobotException(TAG, "Invalid image file name");

        Mat imageOut = Imgcodecs.imread(pInputFilename, Imgcodecs.IMREAD_COLOR);
        if (imageOut.empty())
            throw new AutonomousRobotException(TAG, "Could not find or open the image " + pInputFilename);

        RobotLogCommon.d(TAG, "Image width " + imageOut.cols() + ", height " + imageOut.rows());
        return imageOut;
    }

    // Define a region of interest.
    public static Mat getImageROI(Mat pSrcImage, Rect pROIDefinition) {
        if ((pROIDefinition.height == 0) || (pROIDefinition.width == 0))
            throw new AutonomousRobotException(TAG, "At least one ROI dimension was 0");

        Mat roi = new Mat(pSrcImage, pROIDefinition);
        RobotLogCommon.d(TAG, "Image ROI x " + pROIDefinition.x + ", y " + pROIDefinition.y + ", width " + pROIDefinition.width + ", height " + pROIDefinition.height);
        return roi;
    }

    public static String createOutputFilePreamble(String pInputFilename, String pWorkingDirectory, String pFileDate) {
        // Sanity check on the file name extension.
        if (!(pInputFilename.endsWith(".png") || pInputFilename.endsWith(".jpg")))
            throw new AutonomousRobotException(TAG, "Invalid image file name");

        // Strip the extension from the end of the file name and append an underscore and time stamp.
        String originalFN = pInputFilename.substring(0, pInputFilename.lastIndexOf('.'));
        return pWorkingDirectory + originalFN + "_" + pFileDate;
    }

    public static Mat preProcessImage(Mat pOriginalImage,
                               String pPreamble, VisionParameters.ImageParameters pImageParameters) {
        if ((pOriginalImage.cols() != pImageParameters.resolution_width) ||
                (pOriginalImage.rows() != pImageParameters.resolution_height))
            throw new AutonomousRobotException(TAG,
                    "Mismatch between actual image width and expected image width " + pImageParameters.resolution_width +
                            ", height " + pImageParameters.resolution_height);

        // Crop the image to reduce distractions.
        Mat imageROI = getImageROI(pOriginalImage,
                new Rect(pImageParameters.image_roi.x,
                        pImageParameters.image_roi.y,
                        pImageParameters.image_roi.width,
                        pImageParameters.image_roi.height));

        if (pPreamble != null) {
            String imageFilename = pPreamble + "_ROI.png";
            RobotLogCommon.d(TAG, "Writing image ROI " + imageFilename);
            Imgcodecs.imwrite(imageFilename, imageROI);
        }

        return imageROI;
    }

    // Adjust the median of a grayscale image.
    public static Mat adjustGrayscaleMedian(Mat pGray, int pTarget) {
        int medianGray = getSingleChannelMedian(pGray);
        RobotLogCommon.d(TAG, "Original image: grayscale median " + medianGray);
        RobotLogCommon.d(TAG, "Grayscale median target " + pTarget);

        // adjustment = target - median;
        int adjustment = pTarget - medianGray;
        Mat adjustedGray = new Mat();
        pGray.convertTo(adjustedGray, -1, 1, adjustment);
        RobotLogCommon.d(TAG, "Grayscale adjustment " + adjustment);

        return adjustedGray;
    }

    // Adjust image saturation and value levels in the image to match the targets.
    public static Mat adjustSaturationAndValueMedians(Mat pHSVImage, int pSatMedianTarget, int pValMedianTarget) {
        // Split the image into its constituent HSV channels
        ArrayList<Mat> channels = new ArrayList<>();
        Core.split(pHSVImage, channels);

        // Get the median of the S channel.
        int medianSaturation = getColorChannelMedian(channels.get(1), new Mat());

        // Get the median of the V channel.
        int medianValue = getColorChannelMedian(channels.get(2), new Mat());

        RobotLogCommon.d(TAG, "HSV saturation channel median " + medianSaturation);
        RobotLogCommon.d(TAG, "HSV value channel median " + medianValue);

        // adjustment = target - median;
        int satAdjustment = pSatMedianTarget - medianSaturation;
        int valAdjustment = pValMedianTarget - medianValue;
        channels.get(1).convertTo(channels.get(1), -1, 1, satAdjustment);
        channels.get(2).convertTo(channels.get(2), -1, 1, valAdjustment);

        RobotLogCommon.d(TAG, "Adjust HSV saturation by " + satAdjustment + " to " + pSatMedianTarget);
        RobotLogCommon.d(TAG, "Adjust HSV value by " + valAdjustment + " to " + pValMedianTarget);

        // Merge the channels back together.
        Mat adjustedImage = new Mat();
        Core.merge(channels, adjustedImage);
        return adjustedImage;
    }

    // See https://docs.opencv.org/3.4/d8/dbc/tutorial_histogram_calculation.html
    public static int getDominantHSVHue(Mat pHSVImageIn, Mat pMask) {
        List<Mat> channelsHSV = new ArrayList<>();
        Core.split(pHSVImageIn, channelsHSV);

        // Set the number of bins and range for HSV hue in OpenCV.
        int hueHistSize = 180; // number of bins
        float[] hueRange = {0, 180};
        MatOfFloat hueHistRange = new MatOfFloat(hueRange);

        Mat hueHist = new Mat();
        // MatOfInt(0) indicates channel 0 -> hue
        Imgproc.calcHist(channelsHSV, new MatOfInt(0), pMask, hueHist, new MatOfInt(hueHistSize), hueHistRange, false);

        // Normalize the result to [ 0, hue_hist.rows ]
        //## Normalization is done for graphings, which we don't need.
        // normalize(hue_hist, hue_hist, 0, hue_hist.rows, NORM_MINMAX, -1, Mat());

        //## DEBUG
        //RobotLogCommon.d(TAG, "Hue histogram: bins");
        //for (int i = 0; i < 180; i++)
        //  if (hue_hist.at<float>(i) != 0.0f)
        //	RobotLogCommon.d(TAG, "Bin " + to_string(i) + " = " + to_string(hue_hist.at<float>(i)));

        // Get the bin with the greatest pixel count.
        Core.MinMaxLocResult mmlResult = Core.minMaxLoc(hueHist);
        RobotLogCommon.d(TAG, "Hue histogram: largest bin x " + mmlResult.maxLoc.x + ", y " + mmlResult.maxLoc.y + ", count " + mmlResult.maxVal);

        // The y-coordinate of the maxLoc Point contains the index to the bin
        // with the greatest value. The index to the bin is the hue itself.
        int dominantHue = (int) mmlResult.maxLoc.y;
        RobotLogCommon.d(TAG, "HSV dominant hue " + dominantHue);

        return dominantHue;
    }

    // The input image is BGR. This function converts it to HSV, adjusts the saturation and value according
    // to the targets in the HSVParameters, and applies the OpenCV thresholding function inRange using
    // the hue range in the HSVParameters.
    // In the OpenCV tutorial, no blurring is applied before inRange (unlike grayscale thresholding).
    // https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html
    public static Mat performInRange(Mat pInputROI, String pOutputFilenamePreamble,
                                     VisionParameters.HSVParameters pHSVParameters) {
        return performInRange(pInputROI, pOutputFilenamePreamble, "", pHSVParameters);
    }

    // The file name suffix prevents the overwriting of files in those rate cases
    // where performInRange is called multiple times in the same run.
    public static Mat performInRange(Mat pInputROI, String pOutputFilenamePreamble, String pFilenameSuffix,
                                     VisionParameters.HSVParameters pHSVParameters) {
        // We're on the HSV path.
        Mat hsvROI = new Mat();
        Imgproc.cvtColor(pInputROI, hsvROI, Imgproc.COLOR_BGR2HSV);

        // Adjust the HSV saturation and value levels in the image to match the targets.
        Mat adjusted = adjustSaturationAndValueMedians(hsvROI, pHSVParameters.saturation_median_target,  pHSVParameters.value_median_target);

        // Convert back to BGR.
        if (pOutputFilenamePreamble != null && RobotLogCommon.isLoggable("v")) {
            Mat adjustedBGR = new Mat();
            Imgproc.cvtColor(adjusted, adjustedBGR, Imgproc.COLOR_HSV2BGR);
            Imgcodecs.imwrite(pOutputFilenamePreamble + "_ADJ" + pFilenameSuffix + ".png", adjustedBGR);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_ADJ" + pFilenameSuffix + ".png");
        }

         Mat thresholded = applyInRange(adjusted,  pHSVParameters.hue_low, pHSVParameters.hue_high,
                pHSVParameters.saturation_threshold_low, pHSVParameters.value_threshold_low);

        if (pOutputFilenamePreamble != null) {
            Imgcodecs.imwrite(pOutputFilenamePreamble + "_ADJ_THR" + pFilenameSuffix + ".png", thresholded);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_ADJ_THR" + pFilenameSuffix + ".png");
        }

        return thresholded;
    }

    // Use inRange to threshold a median-adjusted HSV image to binary.
    // Account for hue ranges that cross the 180 degree boundary.
    // Red, for example, might have a hueLow of 170 and a hueHigh of 10.
    // See https://stackoverflow.com/questions/32522989/opencv-better-detection-of-red-color
    public static Mat applyInRange(Mat pAdjustedMedianROI,
                                   int pHueLow, int pHueHigh,
                                   int pSatLowThreshold, int pValLowThreshold) {
        RobotLogCommon.d(TAG, "Actual inRange HSV arguments: hue low " + pHueLow + ", hue high " + pHueHigh);
        RobotLogCommon.d(TAG, "Actual inRange HSV arguments: saturation low " + pSatLowThreshold + ", value low " + pValLowThreshold);

        // Sanity check for hue.
        if (!((pHueLow >= 0 && pHueLow <= 180) && (pHueHigh >= 0 && pHueHigh <= 180) &&
                (pHueLow != pHueHigh)))
            throw new AutonomousRobotException(TAG, "Hue out of range");

        // Normal hue range.
        Mat thresholded = new Mat();
        if (pHueLow < pHueHigh)
            Core.inRange(pAdjustedMedianROI, new Scalar(pHueLow, pSatLowThreshold, pValLowThreshold), new Scalar(pHueHigh, 255, 255), thresholded);
        else {
            // For a hue range from the XML file of low 170, high 10
            // the following yields two new ranges: 170 - 180 and 0 - 10.
            Mat range1 = new Mat();
            Mat range2 = new Mat();
            Core.inRange(pAdjustedMedianROI, new Scalar(pHueLow, pSatLowThreshold, pValLowThreshold), new Scalar(180, 255, 255), range1);
            Core.inRange(pAdjustedMedianROI, new Scalar(0, pSatLowThreshold, pValLowThreshold), new Scalar(pHueHigh, 255, 255), range2);
            Core.bitwise_or(range1, range2, thresholded);
        }

        return thresholded;
    }

    // Combine the frequently associated steps of applying inRange to
    // an HSV image and then finding contours in the thresholded output.
    //!! findContours works without blurring and morphological opening.
    //!! But there are fewer artifacts in the contour recognition if only
    //!! morphological opening is included.
    public static List<MatOfPoint> performInRangeAndFindContours(Mat pInputROI,
                                                                 String pOutputFilenamePreamble, VisionParameters.HSVParameters pHSVParameters) {
        Mat thresholded = performInRange(pInputROI, pOutputFilenamePreamble, pHSVParameters);
        Mat morphed = new Mat();
        Imgproc.erode(thresholded, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
        Imgproc.dilate(morphed, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

        // Identify the contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(morphed, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        return contours;
    }

    // The target low hue may be greater than the target high hue. For example,
    // target low 170, target high 10.
    public static boolean hueInRange(int pHue, int pTargetLow, int pTargetHigh) {
        // Sanity check for hue.
        if (!((pTargetLow >= 0 && pTargetLow <= 180) && (pTargetHigh >= 0 && pTargetHigh <= 180) &&
                (pTargetLow != pTargetHigh)))
            throw new AutonomousRobotException(TAG, "Hue out of range");

        if (pTargetLow < pTargetHigh)
            return (pHue >= pTargetLow && pHue <= pTargetHigh);

        return (pHue >= pTargetLow && pHue <= 180) ||
                (pHue >= 0 && pHue <= pTargetHigh);
    }

    // Common path for converting an OpenCV BGR image to grayscale,
    // adjusting the grayscale image to a target, performing
    // morphological opening, blurring the image, and thresholding it.
    public static Mat convertToGrayAndThreshold(Mat pBGRInputROI, String pOutputFilenamePreamble,
                                       int pGrayscaleMedianTarget, int pLowThreshold) {
        // We're on the grayscale path.
        Mat grayROI = new Mat();
        Imgproc.cvtColor(pBGRInputROI, grayROI, Imgproc.COLOR_BGR2GRAY);

        if (pOutputFilenamePreamble != null && RobotLogCommon.isLoggable("v")) {
            Imgcodecs.imwrite(pOutputFilenamePreamble + "_GRAY.png", grayROI);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_GRAY.png");
        }

        return performThresholdOnGray(grayROI, pOutputFilenamePreamble, pGrayscaleMedianTarget, pLowThreshold);
    }

    public static Mat performThresholdOnGray(Mat pGrayInputROI, String pOutputFilenamePreamble,
                                             int pGrayscaleMedianTarget, int pLowThreshold) {
        Mat adjustedGray = adjustGrayscaleMedian(pGrayInputROI, pGrayscaleMedianTarget);
        if (pOutputFilenamePreamble != null && RobotLogCommon.isLoggable("v")) {
            Imgcodecs.imwrite(pOutputFilenamePreamble + "_ADJ.png", adjustedGray);
            RobotLogCommon.d(TAG, "Writing adjusted grayscale image " + pOutputFilenamePreamble + "_ADJ.png");
        }

        Mat thresholded = applyGrayThreshold(adjustedGray, pLowThreshold);

        if (pOutputFilenamePreamble != null) {
            Imgcodecs.imwrite(pOutputFilenamePreamble + "_ADJ_THR.png", thresholded);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_ADJ_THR.png");
        }

        return thresholded;
    }

    // Note that pyimagesearch always blurs grayscale images before thresholding:
    // https://www.pyimagesearch.com/2021/04/28/opencv-thresholding-cv2-threshold/
    // This OpenCV tutorial does not:
    // https://docs.opencv.org/3.4/db/d8e/tutorial_threshold.html
    // But this one does with convincing results:
    // https://docs.opencv.org/4.x/d7/d4d/tutorial_py_thresholding.html
    public static Mat applyGrayThreshold(Mat pGrayInputROI, int pGrayLowThreshold) {
        Mat morphed = new Mat();
        Imgproc.erode(pGrayInputROI, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
        Imgproc.dilate(morphed, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

        // Remove noise by Gaussian blurring.
        Mat blurred = new Mat();
        Imgproc.GaussianBlur(morphed, blurred, new Size(5, 5), 0);

        RobotLogCommon.v(TAG, "Threshold values: low " + pGrayLowThreshold + ", high 255");

        // Threshold the image: set pixels over the threshold value to white.
        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(blurred, thresholded,
                Math.abs(pGrayLowThreshold),    // threshold value
                255,   // white
                pGrayLowThreshold >= 0 ? Imgproc.THRESH_BINARY : Imgproc.THRESH_BINARY_INV); // thresholding type
        return thresholded;
    }

    // Get the median of any single-channel Mat.
    public static int getSingleChannelMedian(Mat pSingleChannelMat) {
        if ((pSingleChannelMat.dims() != 2) || (!pSingleChannelMat.isContinuous()))
            throw new AutonomousRobotException(TAG, "Expected a single-channel Mat");

        byte[] byteBuff = new byte[(int) pSingleChannelMat.total()];
        int[] intBuff = new int[(int) pSingleChannelMat.total()];
        int buffLength = byteBuff.length;
        pSingleChannelMat.get(0, 0, byteBuff);

        // !! Since Java does not have an unsigned char data type, the byte values
        // may come out as negative. So we have to use a separate array of ints and
        // copy in the bytes with the lower 8 bytes preserved.
        // https://stackoverflow.com/questions/9581530/converting-from-byte-to-int-in-java
        for (int i = 0; i < buffLength; i++)
            // Requires Android API level 26 = Byte.toUnsignedInt(byteBuff[i]);
            intBuff[i] = byteBuff[i] & 0xFF;

        Arrays.sort(intBuff);
        return (intBuff[buffLength / 2] + (intBuff[(buffLength / 2) - 1])) / 2;
    }

    // Sort contours by area in descending order.
    public static List<MatOfPoint> sortContoursByArea(List<MatOfPoint> pContours) {
        List<MatOfPoint> sortedContours = pContours.stream()
                .sorted(Comparator.comparing(Imgproc::contourArea))
                .collect(Collectors.toList());

        // .sorted(Comparator.reverseOrder(Imgproc::contourArea)) is ambiguous
        Collections.reverse(sortedContours);
        return sortedContours;
    }

    // Get the contours from a thresholded image, optionally draw them, and return a
    // Pair of the number of contours and the largest contour.
    public static Optional<Pair<Integer, MatOfPoint>> getLargestContour(Mat pImageROI, Mat pThresholded,
                                                         String pOutputFilenamePreamble) {
        // Identify the contours.
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(pThresholded, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.size() == 0)
            return Optional.empty();

        // Within the ROI draw all of the contours.
        RobotLogCommon.d(TAG, "Number of contours " + contours.size());
        if (pOutputFilenamePreamble != null && RobotLogCommon.isLoggable("v")) {
            Mat contoursDrawn = pImageROI.clone();
            ShapeDrawing.drawShapeContours(contours, contoursDrawn);
            Imgcodecs.imwrite(pOutputFilenamePreamble + "_CON.png", contoursDrawn);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_CON.png");
        }

        // See how this works here:
        // https://docs.oracle.com/javase/8/docs/api/java/util/Optional.html#map-java.util.function.Function-
        Optional<MatOfPoint> largestContour = getLargestContour(contours);
        return largestContour.map(matOfPoint -> Pair.create(contours.size(), matOfPoint));
    }

    // From https://stackoverflow.com/questions/40669684/opencv-sorting-contours-by-area-in-java
    // See lambda in https://stackoverflow.com/questions/24378646/finding-max-with-lambda-expression-in-java
    public static Optional<MatOfPoint> getLargestContour(List<MatOfPoint> pContours) {
        return pContours.stream().max(Comparator.comparing(Imgproc::contourArea));
    }

    public static Point getContourCentroid(MatOfPoint pOneContour) {
        Moments moments = Imgproc.moments(pOneContour);
        return new Point(moments.get_m10() / moments.get_m00(),
                moments.get_m01() / moments.get_m00());
    }

    public static Pair<Integer, Integer> getMedianSaturationAndValue(Mat pHSVImage) {
        // Split the image into its constituent HSV channels
        ArrayList<Mat> channels = new ArrayList<>();
        Core.split(pHSVImage, channels);

        // Get the median of the S channel.
        int medianSaturation = getColorChannelMedian(channels.get(1), new Mat());

        // Get the median of the V channel.
        int medianValue = getColorChannelMedian(channels.get(2), new Mat());

        RobotLogCommon.d(TAG, "HSV saturation channel median " + medianSaturation);
        RobotLogCommon.d(TAG, "HSV value channel median " + medianValue);

        return Pair.create(medianSaturation, medianValue);
    }

    // Get the median of a color channel.
    private static int getColorChannelMedian(Mat pChannel, Mat pMask) {
        // If we're dealing with a non-masked image then we just take the median
        // of all the pixels.
        if (pMask.total() == 0) {
            return getSingleChannelMedian(pChannel);
        } else
            throw new AutonomousRobotException(TAG, "getColorChannelMedian with mask is not supported at this time");
    }

}

