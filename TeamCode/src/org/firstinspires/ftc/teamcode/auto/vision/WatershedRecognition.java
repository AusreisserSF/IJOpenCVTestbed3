package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.TimeStamp;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.xml.VisionParameters;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class WatershedRecognition {

    private static final String TAG = WatershedRecognition.class.getSimpleName();

    public enum WatershedRecognitionPath {
        WATERSHED_CARDS_STD,
        WATERSHED_CARDS_HYBRID, WATERSHED_COINS_HYBRID
    }

    private final String testCaseDirectory;

    public WatershedRecognition(String pTestCaseDirectory) {
        testCaseDirectory = pTestCaseDirectory;
    }

    // OpenCV has two examples: a standard example (c++, Java, Python)
    // that works with an image of cards and a Python-only example that
    // works with an image of coins.

    // Reproduce the standard Java example and also create a hybrid of
    // that works with both cards and coins. The hybrid includes parts
    // of the two OpenCV examples as well as one from pyimagesearch
    // that targets coins.

    //!! Note that the standard Java example misses the boundary between
    // the two cards at the center right; our hybrid method is much
    // better.

    // The OpenCV standard Java example is here --
    // https://docs.opencv.org/4.x/d2/dbd/tutorial_distance_transform.html,
    // which is implemented in this project as WatershedRecognitionStd,
    //
    // The OpenCV Python example is here --
    // https://docs.opencv.org/4.x/d3/db4/tutorial_py_watershed.html
    //
    // The pyimagesearch example is here --
    // https://pyimagesearch.com/2015/11/02/watershed-opencv/
    //
    // Returns the result of image analysis.
    public RobotConstants.RecognitionResults performWatershed(ImageProvider pImageProvider,
                                                              VisionParameters.ImageParameters pImageParameters,
                                                              WatershedRecognitionPath pWatershedRecognitionPath) throws InterruptedException {
        RobotLogCommon.d(TAG, "In WatershedRecognition.performWatershed");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> watershedImage = pImageProvider.getImage();
        if (watershedImage == null)
            return RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR; // don't crash

        // The image is in BGR order (OpenCV imread from a file).
        String fileDate = TimeStamp.getLocalDateTimeStamp(watershedImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(pImageParameters.image_source, testCaseDirectory, fileDate);
        Mat imageROI = ImageUtils.preProcessImage(watershedImage.first, outputFilenamePreamble, pImageParameters);
        RobotLogCommon.d(TAG, "Recognition path " + pWatershedRecognitionPath);

        // Adapt the examples to our environment.
        switch (pWatershedRecognitionPath) {
            case WATERSHED_CARDS_STD -> {
                return watershedCardsStd(imageROI, outputFilenamePreamble);
            }
            case WATERSHED_CARDS_HYBRID -> {
                return watershedCardsHybrid(imageROI, outputFilenamePreamble);
            }
            case WATERSHED_COINS_HYBRID -> {
                return watershedCoinsHybrid(imageROI, outputFilenamePreamble);
            }
            default -> throw new AutonomousRobotException(TAG, "Unrecognized recognition path");
        }
    }

    // Standard OpenCV Watershed example from --
    // https://docs.opencv.org/4.x/d2/dbd/tutorial_distance_transform.html
    // Adapt the standard Java example to our environment.
    //!! Note that the example misses the card in the upper right.
    private RobotConstants.RecognitionResults watershedCardsStd(Mat pImageROI, String pOutputFilenamePreamble) {
        //! [black_bg]
        // Change the background from white to black, since that will help later to
        // extract better results during the use of Distance Transform
        //##PY This works because the cards are R 248, G 245, B 245.
        //##PY Shared by both paths.
        Mat blk = invertCardsBackground(pImageROI, pOutputFilenamePreamble);

        //##PY Try a sharpening kernel I got from stackoverflow.
        // The results are nearly identical - actually both methods
        // miss-classify the empty space just under the card in the
        // upper-right.
        Mat imgResult = ImageUtils.sharpen(blk, pOutputFilenamePreamble);

        //##PY The Laplacian filtering and the sharpening do make a difference
        // but the results are fine with just sharpening.
        /*
        //! [sharp]
        // Create a kernel that we will use to sharpen our image
        Mat kernel = new Mat(3, 3, CvType.CV_32F);
        // an approximation of second derivative, a quite strong kernel
        float[] kernelData = new float[(int) (kernel.total() * kernel.channels())];
        kernelData[0] = 1; kernelData[1] = 1; kernelData[2] = 1;
        kernelData[3] = 1; kernelData[4] = -8; kernelData[5] = 1;
        kernelData[6] = 1; kernelData[7] = 1; kernelData[8] = 1;
        kernel.put(0, 0, kernelData);

        // do the laplacian filtering as it is
        // well, we need to convert everything in something more deeper then CV_8U
        // because the kernel has some negative values,
        // and we can expect in general to have a Laplacian image with negative values
        // BUT a 8bits unsigned int (the one we are working with) can contain values
        // from 0 to 255
        // so the possible negative number will be truncated
        Mat imgLaplacian = new Mat();
        Imgproc.filter2D(src, imgLaplacian, CvType.CV_32F, kernel);

        Mat sharp = new Mat();
        src.convertTo(sharp, CvType.CV_32F);

        Mat imgResult = new Mat();
        Core.subtract(sharp, imgLaplacian, imgResult);

        // imshow( "Laplace Filtered Image", imgLaplacian );
        imgLaplacian.convertTo(imgLaplacian, CvType.CV_8UC3); // convert back to 8bits gray scale
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_LAP.png", imgLaplacian);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_LAP.png");

        // Output the sharpened image
        imgResult.convertTo(imgResult, CvType.CV_8UC3); // convert back to 8bits gray scale
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_SHARP.png", imgResult);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_SHARP.png");
        //! [sharp]
        */

        //! [bin]
        // Create binary image from source image
        Mat bw = new Mat();
        Imgproc.cvtColor(imgResult, bw, Imgproc.COLOR_BGR2GRAY);
        Imgproc.threshold(bw, bw, 40, 255, Imgproc.THRESH_BINARY | Imgproc.THRESH_OTSU);

        // Output the thresholded image.
        String thrFilename = pOutputFilenamePreamble + "_THR.png";
        Imgcodecs.imwrite(thrFilename, bw);
        RobotLogCommon.v(TAG, "Writing " + thrFilename);
        //! [bin]

        //! [dist]
        // Perform the distance transform algorithm
        Mat dist = new Mat();
        Imgproc.distanceTransform(bw, dist, Imgproc.DIST_L2, 3);

        // Normalize the distance image for range = {0.0, 1.0}
        // so we can visualize and threshold it
        Core.normalize(dist, dist, 0.0, 1.0, Core.NORM_MINMAX);
        Mat distDisplayScaled = new Mat();
        Core.multiply(dist, new Scalar(255), distDisplayScaled);
        Mat distDisplay = new Mat();
        distDisplayScaled.convertTo(distDisplay, CvType.CV_8U);

        // Output the transformed image.
        String distFilename = pOutputFilenamePreamble + "_DIST.png";
        Imgcodecs.imwrite(distFilename, distDisplay);
        RobotLogCommon.d(TAG, "Writing " + distFilename);
        //! [dist]

        //! [peaks]
        // Threshold to obtain the peaks
        // This will be the markers for the foreground objects
        Imgproc.threshold(dist, dist, 0.4, 1.0, Imgproc.THRESH_BINARY);

        // Dilate a bit the dist image
        Mat kernel1 = Mat.ones(3, 3, CvType.CV_8U);
        Imgproc.dilate(dist, dist, kernel1);
        Mat distDisplay2 = new Mat();
        dist.convertTo(distDisplay2, CvType.CV_8U);
        Core.multiply(distDisplay2, new Scalar(255), distDisplay2);

        // Output the foreground peaks.
        String peakFilename = pOutputFilenamePreamble + "_PEAK.png";
        Imgcodecs.imwrite(peakFilename, distDisplay2);
        RobotLogCommon.d(TAG, "Writing " + peakFilename);
        //! [peaks]

        //! [seeds]
        // Create the CV_8U version of the distance image
        // It is needed for findContours()
        Mat dist_8u = new Mat();
        dist.convertTo(dist_8u, CvType.CV_8U);

        // Find total markers
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(dist_8u, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        //#PY added - output the contours.
        Mat contoursOut = pImageROI.clone();
        ShapeDrawing.drawShapeContours(contours, contoursOut);
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_CON.png", contoursOut);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_CON.png");

        // Create the marker image for the watershed algorithm
        Mat markers = Mat.zeros(dist.size(), CvType.CV_32S);

        // Draw the foreground markers
        for (int i = 0; i < contours.size(); i++) {
            Imgproc.drawContours(markers, contours, i, new Scalar(i + 1), -1);
        }

        // Draw the background marker
        //??PY I don't know why the comment refers to a "background marker"
        // and then draws a small white circle in the upper left.
        Mat markersScaled = new Mat();
        markers.convertTo(markersScaled, CvType.CV_32F);
        Core.normalize(markersScaled, markersScaled, 0.0, 255.0, Core.NORM_MINMAX);
        Imgproc.circle(markersScaled, new Point(5, 5), 3, new Scalar(255, 255, 255), -1);
        Mat markersDisplay = new Mat();
        markersScaled.convertTo(markersDisplay, CvType.CV_8U);

        // Output the markers.
        String markFilename = pOutputFilenamePreamble + "_MARK.png";
        Imgcodecs.imwrite(markFilename, markersDisplay);
        RobotLogCommon.d(TAG, "Writing " + markFilename);

        Imgproc.circle(markers, new Point(5, 5), 3, new Scalar(255, 255, 255), -1);
        //! [seeds]

        //! [watershed]
        // Perform the watershed algorithm
        Imgproc.watershed(imgResult, markers);

        /*
        Mat mark = Mat.zeros(markers.size(), CvType.CV_8U);
        markers.convertTo(mark, CvType.CV_8UC1);
        Core.bitwise_not(mark, mark);

        // imshow("Markers_v2", mark); // uncomment this if you want to see how the mark
        // image looks like at that point
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_MARK2.png",mark);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_MARK2.png");
        */

        showWatershedColor(markers, pOutputFilenamePreamble);
        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }

    private RobotConstants.RecognitionResults watershedCardsHybrid(Mat pImageROI, String pOutputFilenamePreamble) {

        Mat blk = invertCardsBackground(pImageROI, pOutputFilenamePreamble);

        // The Python example does not use a sharpening Kernel.
        // The standard Java example uses a multi-step sharpening
        // pass but the sharpening kernel I got from stackoverflow
        // produces nearly identical results.
        Mat sharp = ImageUtils.sharpen(blk, pOutputFilenamePreamble);

        // Unlike both official samples we will use the red channel
        // of the sharpened cards image.
        ArrayList<Mat> channels = new ArrayList<>(3);
        Core.split(sharp, channels); // red or blue channel. B = 0, G = 1, R = 2
        Mat redChannel = channels.get(2);

        String redFilename = pOutputFilenamePreamble + "_RED.png";
        Imgcodecs.imwrite(redFilename, redChannel);
        RobotLogCommon.d(TAG, "Writing " + redFilename);

        // Both standard examples use OTSU but we get better results
        // (the interiors of the cards go to white) with a binary
        // threshold - either inverted or not.
        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(redChannel, thresholded,
                Math.abs(175),
                255,   // white
                Imgproc.THRESH_BINARY);

        String thrFilename = pOutputFilenamePreamble + "_THR.png";
        Imgcodecs.imwrite(thrFilename, thresholded);
        RobotLogCommon.v(TAG, "Writing " + thrFilename);

        Mat watershedMarkers = WatershedUtils.applyWatershedHybrid(thresholded, pImageROI, sharp,
                pOutputFilenamePreamble, "");
        showWatershedColor(watershedMarkers, pOutputFilenamePreamble);
        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }

    private RobotConstants.RecognitionResults watershedCoinsHybrid(Mat pImageROI, String pOutputFilenamePreamble) {

        Mat sharp = ImageUtils.sharpen(pImageROI, pOutputFilenamePreamble);

        // From pyimagesearch
        Mat shifted = new Mat();
        Imgproc.pyrMeanShiftFiltering(sharp, shifted, 21, 51);

        Mat gray = new Mat();
        Imgproc.cvtColor(shifted, gray, Imgproc.COLOR_BGR2GRAY);

        // Output the grayscale image.
        String grayFilename = pOutputFilenamePreamble + "_GRAY.png";
        Imgcodecs.imwrite(grayFilename, gray);
        RobotLogCommon.d(TAG, "Writing " + grayFilename);

        //**TODO Parameterize the thresholding types; default to Imgproc.THRESH_BINARY.
        // Including the type in the XML is better than our current method of negating
        // the low threshold value to indicate THRESH_BINARY_INV.

        //!! The standard image of coins is best with Imgproc.THRESH_BINARY_INV | Imgproc.THRESH_OTSU
        //!! pyimagesearch_coins_02.png is best with Imgproc.THRESH_BINARY | Imgproc.THRESH_OTSU
        //!! pyimagesearch_coins_01.png is best with Imgproc.THRESH_BINARY | Imgproc.THRESH_OTSU
        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(gray, thresholded,
                Math.abs(100),
                255,   // white
                Imgproc.THRESH_BINARY | Imgproc.THRESH_OTSU);

        String thrFilename = pOutputFilenamePreamble + "_THR.png";
        Imgcodecs.imwrite(thrFilename, thresholded);
        RobotLogCommon.d(TAG, "Writing " + thrFilename);

        Mat watershedMarkers = WatershedUtils.applyWatershedHybrid(thresholded, pImageROI, sharp,
                pOutputFilenamePreamble, "");
        showWatershedColor(watershedMarkers, pOutputFilenamePreamble);
        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }

    // Source: standard Java example - specific to the cards image.
    //! [black_bg]
    // Change the background from white to black, since that will help later to
    // extract better results during the use of Distance Transform
    //##PY This works because the cards are R 248, G 245, B 245
    private Mat invertCardsBackground(Mat pImageROI, String pOutputFilenamePreamble) {
        Mat src = pImageROI.clone();
        byte[] srcData = new byte[(int) (src.total() * src.channels())];
        src.get(0, 0, srcData);
        for (int i = 0; i < src.rows(); i++) {
            for (int j = 0; j < src.cols(); j++) {
                if (srcData[(i * src.cols() + j) * 3] == (byte) 255 && srcData[(i * src.cols() + j) * 3 + 1] == (byte) 255
                        && srcData[(i * src.cols() + j) * 3 + 2] == (byte) 255) {
                    srcData[(i * src.cols() + j) * 3] = 0;
                    srcData[(i * src.cols() + j) * 3 + 1] = 0;
                    srcData[(i * src.cols() + j) * 3 + 2] = 0;
                }
            }
        }

        src.put(0, 0, srcData);

        // Output the image with a black background.
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_BLK.png", src);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_BLK.png");

        return src;
    }

    private void showWatershedColor(Mat pMarkers, String pOutputFilenamePreamble) {
        // Generate random colors
        Random rng = new Random(12345);
        List<Scalar> colors = new ArrayList<>(pMarkers.rows());
        for (int i = 0; i < pMarkers.rows(); i++) {
            int b = rng.nextInt(256);
            int g = rng.nextInt(256);
            int r = rng.nextInt(256);
            colors.add(new Scalar(b, g, r));
        }

        // Create the result image
        Mat dst = Mat.zeros(pMarkers.size(), CvType.CV_8UC3);
        byte[] dstData = new byte[(int) (dst.total() * dst.channels())];
        dst.get(0, 0, dstData);

        // Fill labeled objects with random colors.
        int[] markersData = new int[(int) (pMarkers.total() * pMarkers.channels())];
        pMarkers.get(0, 0, markersData);
        for (int i = 0; i < pMarkers.rows(); i++) {
            for (int j = 0; j < pMarkers.cols(); j++) {
                int index = markersData[i * pMarkers.cols() + j];
                // watershed object markers start at 2
                if (index >= 2) {
                    dstData[(((i * dst.cols()) + j) * 3) + 0] = (byte) colors.get(index - 2).val[0];
                    dstData[(((i * dst.cols()) + j) * 3) + 1] = (byte) colors.get(index - 2).val[1];
                    dstData[(((i * dst.cols()) + j) * 3) + 2] = (byte) colors.get(index - 2).val[2];
                } else {
                    dstData[(((i * dst.cols()) + j) * 3) + 0] = 0;
                    dstData[(((i * dst.cols()) + j) * 3) + 1] = 0;
                    dstData[(((i * dst.cols()) + j) * 3) + 2] = 0;
                }
            }
        }

        dst.put(0, 0, dstData);

        // Visualize the final image
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_WS.png", dst);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_WS.png");
    }

}