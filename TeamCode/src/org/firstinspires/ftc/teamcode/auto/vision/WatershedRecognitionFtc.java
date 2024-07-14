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
        DISTANCE
    }

    private final RobotConstants.Alliance alliance;
    private final String testCaseDirectory;

    public WatershedRecognitionFtc(RobotConstants.Alliance pAlliance, String pTestCaseDirectory) {
        alliance = pAlliance;
        testCaseDirectory = pTestCaseDirectory;
    }

    // Use the OpenCV Watershed algorithm with FTC images. The code here is based
    // on a combination of the official example at
    // https://docs.opencv.org/4.x/d2/dbd/tutorial_distance_transform.html,
    // which is implemented in this project as WatershedRecognitionStd, and
    // a Python solution at
    // https://medium.com/@jaskaranbhatia/exploring-image-segmentation-techniques-watershed-algorithm-using-opencv-9f73d2bc7c5a

    //##From the testing so far it's not at all clear that the Watershed technique
    // will be useful in FTC; the watershed output of the blue team prop in CenterStage
    // merges the team prop and the blue spike - but the output of the distanceTransform
    // does look promising - it eliminates the blue spike. And it's not clear how you would
    // actually use the final output of the watershed in FTC.

    //##For the sake of completeness keep going with the watershed. Introduce a
    // third reference: the OpenCV 3 Computer Vision Application Programming Cookbook,
    // Third Edition, by Robert Laganiere; pg. 162. Note how he uses erosion to create
    // the sure foreground and dilation to create the sure background, how he adds
    // foreground and background to get his markers, and how he outputs the final results,
    // including just the boundaries.

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
            // Use a switch by convention in case we have more paths in the future.
            case DISTANCE -> {
                return watershedFromDistance(imageROI, outputFilenamePreamble, pWatershedParametersFtc.watershedDistanceParameters);
            }
            default -> throw new AutonomousRobotException(TAG, "Unrecognized recognition path");
        }
    }

    private RobotConstants.RecognitionResults watershedFromDistance(Mat pImageROI, String pOutputFilenamePreamble, WatershedParametersFtc.WatershedDistanceParameters pWwatershedDistanceParameters) {

        // Use the grayscale and pixel count criteria parameters for the current alliance.
        VisionParameters.GrayParameters allianceGrayParameters;
        switch (alliance) {
            case RED -> allianceGrayParameters = pWwatershedDistanceParameters.redGrayParameters;
            case BLUE -> allianceGrayParameters = pWwatershedDistanceParameters.blueGrayParameters;
            default ->
                    throw new AutonomousRobotException(TAG, "colorChannelPixelCountPath requires an alliance selection");
        }

        //##PY The Laplacian filtering and the sharpening in the OpenCV example
        // do make a difference but they can be replaced by a simple sharpening
        // kernel.
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

        // Follow medium.com and perform two morphological openings on the thresholded image.
        Mat opening = new Mat();
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(thresholded, opening, Imgproc.MORPH_OPEN, kernel, new Point(-1, -1), 2);

        String openFilename = pOutputFilenamePreamble + "_OPEN.png";
        Imgcodecs.imwrite(openFilename, opening);
        RobotLogCommon.d(TAG, "Writing " + openFilename);

        // Follow medium.com and perform
        // dilation for background identification: input = opening, output -> sure_bg
        //# sure background area [##PY i.e. the black portions of the sure_bg image]
        //        sure_bg = cv2.dilate(opening, kernel, iterations=3)
        Mat sure_bg = new Mat();
        Imgproc.dilate(opening, sure_bg, kernel, new Point(-1, -1), 3);

        String bgFilename = pOutputFilenamePreamble + "_BG.png";
        Imgcodecs.imwrite(bgFilename, sure_bg);
        RobotLogCommon.d(TAG, "Writing " + bgFilename);

        // Follow medium.com and find the sure foreground area
        //        dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2,5)
        //        ret, sure_fg = cv2.threshold(dist_transform, 0.7 * dist_transform.max(), 255, 0)

        // The distance identifies regions that are likely to be in
        // the foreground.
        //! [dist]
        // Perform the distance transform algorithm. Imgproc.DIST_L2
        // is a flag for Euclidean distance. Output is 32FC1.
        Mat dist = new Mat();
        Imgproc.distanceTransform(opening, dist, Imgproc.DIST_L2, 3);

        //##PY The normalization steps in the OpenCV example are not necessary
        // - just normalize to the range of 0 - 255.

        Core.normalize(dist, dist, 0.0, 255.0, Core.NORM_MINMAX);
        Mat dist_8u = new Mat();
        dist.convertTo(dist_8u, CvType.CV_8U);

        // Output the transformed image.
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_DIST.png", dist_8u);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_DIST.png");
        //! [dist]

        // Follow medium.com
        // find the sure foreground area

        //! [peaks]
        // Threshold to obtain the peaks.
        // These will be the markers for the foreground objects.
        //##PY Since we've already normalized to a range of 0 - 255 we can replace this
        // Imgproc.threshold(dist, dist, 0.4, 1.0, Imgproc.THRESH_BINARY);
        Mat sure_fg = new Mat();
        Imgproc.threshold(dist_8u, sure_fg, 100, 255, Imgproc.THRESH_BINARY);

        //##PY The dilation steps in the OpenCV example are not necessary.

        // Output the foreground peaks.
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_FG.png", sure_fg);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_FG.png");
        //! [peaks]

        //! [seeds]
        //##PY Skip the conversion steps in the OpenCV example because we've
        // already created the 8-bit Mat dist_8u.

        // Find the sure foreground objects.
        //## medium.com uses connectedComponents; the standard example uses findContours.
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(sure_fg, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        //#PY added - output the contours.
        Mat contoursOut = pImageROI.clone();
        ShapeDrawing.drawShapeContours(contours, contoursOut);
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_CON.png", contoursOut);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_CON.png");

        // Follow medium.com
        // find unknown regions
        //  sure_fg = np.uint8(sure_fg)
        //  unknown = cv2.subtract(sure_bg, sure_fg)
        Mat unknown = new Mat();
        Core.subtract(sure_bg, sure_fg, unknown);

        Imgcodecs.imwrite(pOutputFilenamePreamble + "_UNK.png", unknown);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_UNK.png");

        //**TODO Comments look suspect ...
        // Follow medium.com
        // but instead of connected components use the method from //**TODO Laganiere? the standard example.
        /*
        Label the sure_bg, sure_fg and unknown regions
        # Marker labelling
        # Connected Components determines the connectivity of blob-like regions in a binary image.
        ret, markers = cv2.connectedComponents(sure_fg)

        # Add one to all labels so that sure background is not 0, but 1
        markers = markers+1

        # Now, mark the region of unknown with zero
        markers[unknown==255] = 0
         */

        /*
        Also, we want the sure background to be labeled differently from
        the sure foreground, we add 1 to all the labels in the marker image.
        After this operation, sure background pixels are labeled as 1, and
        the sure foreground pixels are labeled starting from 2.
         */

        // Create the marker image for the watershed algorithm
        Mat markers = Mat.ones(dist.size(), CvType.CV_32S);
        // # Add one to all labels so that sure background is not 0, but 1
        // markers = markers+1
        // Draw the foreground markers
        for (int i = 0; i < contours.size(); i++) {
            Imgproc.drawContours(markers, contours, i, new Scalar(i + 2), -1);
        }

        // Follow medium.com
        // # Now, mark the region of unknown with zero
        // markers[unknown==255] = 0

        // Since we don't have that nice Python syntax,
        // we need to iterate through the Mat of unknowns and for every
        // white (255) value, set the marker at the some location to 0.
        // See https://answers.opencv.org/question/5/how-to-get-and-modify-the-pixel-of-mat-in-java/?answer=8#post-id-8
        /*
        Mat m = ...  // assuming it's of CV_8U type
        byte buff[] = new byte[m.total() * m.channels()];
        m.get(0, 0, buff);
        // working with buff
        // ...
        m.put(0, 0, buff);
         */

        //?? Shouldn't both channel values be 1??
        // The number of elements in these two arrays should be the same.
        byte[] unknownData = new byte[(int) (unknown.total() * unknown.channels())];
        int[] markerData = new int[(int) (markers.total() * markers.channels())];
        int numMarkerRows = markers.rows();
        int numMarkerCols = markers.cols();
        unknown.get(0, 0, unknownData);
        markers.get(0, 0, markerData);
        int sharedIndex;
        for (int i = 0; i < numMarkerRows; i++) {
            for (int j = 0; j < numMarkerCols; j++) {
                sharedIndex = (i * numMarkerCols) + j;
                if ((unknownData[sharedIndex] & 0xff) == 255)
                    markerData[sharedIndex] = 0;
            }
        }

        markers.put(0, 0, markerData); // back into Mat

        //## The way of choosing the background marker below looks suspect.
        /*
        // Draw the background marker
        Mat markersScaled = new Mat();
        markers.convertTo(markersScaled, CvType.CV_32F);
        Core.normalize(markersScaled, markersScaled, 0.0, 255.0, Core.NORM_MINMAX);
        Imgproc.circle(markersScaled, new Point(5, 5), 3, new Scalar(255, 255, 255), -1);
        Mat markersDisplay = new Mat();
        markersScaled.convertTo(markersDisplay, CvType.CV_8U);

        Imgcodecs.imwrite(pOutputFilenamePreamble + "_MARK.png", markersDisplay);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_MARK.png");

        Imgproc.circle(markers, new Point(5, 5), 3, new Scalar(255, 255, 255), -1);
        //! [seeds]
        */

        //! [watershed]
        // Perform the watershed algorithm
        Imgproc.watershed(sharp, markers);

        //##PY This so-called "Markers_V2" image is not used in any further
        // processing.
        /*
        Mat mark = Mat.zeros(markers.size(), CvType.CV_8U);
        markers.convertTo(mark, CvType.CV_8UC1);
        Core.bitwise_not(mark, mark);

        // imshow("Markers_v2", mark); // uncomment this if you want to see how the mark
        // image looks like at that point
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_MARK2.png",mark);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_MARK2.png");
        */

        // Generate random colors
        Random rng = new Random(12345);
        List<Scalar> colors = new ArrayList<>(contours.size());
        for (int i = 0; i < contours.size(); i++) {
            int b = rng.nextInt(256);
            int g = rng.nextInt(256);
            int r = rng.nextInt(256);

            colors.add(new Scalar(b, g, r));
        }

        // Create the result image
        Mat dst = Mat.zeros(markers.size(), CvType.CV_8UC3);
        byte[] dstData = new byte[(int) (dst.total() * dst.channels())];
        dst.get(0, 0, dstData);

        // Fill labeled objects with random colors
        int[] markersData = new int[(int) (markers.total() * markers.channels())];
        markers.get(0, 0, markersData);
        for (int i = 0; i < markers.rows(); i++) {
            for (int j = 0; j < markers.cols(); j++) {
                int index = markersData[i * markers.cols() + j];
                // watershed object markers start at 2
                if (index >= 2) {
                    dstData[(i * dst.cols() + j) * 3 + 0] = (byte) colors.get(index - 2).val[0];
                    dstData[(i * dst.cols() + j) * 3 + 1] = (byte) colors.get(index - 2).val[1];
                    dstData[(i * dst.cols() + j) * 3 + 2] = (byte) colors.get(index - 2).val[2];
                } else {
                    dstData[(i * dst.cols() + j) * 3 + 0] = 0;
                    dstData[(i * dst.cols() + j) * 3 + 1] = 0;
                    dstData[(i * dst.cols() + j) * 3 + 2] = 0;
                }
            }
        }

        dst.put(0, 0, dstData);

        // Visualize the final image
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_WS.png", dst);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "WS.png");
        //! [watershed]

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