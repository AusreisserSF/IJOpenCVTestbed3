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

public class WatershedRecognitionStd {

    private static final String TAG = WatershedRecognitionStd.class.getSimpleName();

    public enum WatershedRecognitionPath {
        WATERSHED_CARDS_STD, WATERSHED_CARDS_HYBRID
    }

    private final String testCaseDirectory;

    //**TODO Currently limited to the cards image. Will it work with coins?

    public WatershedRecognitionStd(String pTestCaseDirectory) {
        testCaseDirectory = pTestCaseDirectory;
    }

    // OpenCV has two examples: a standard example (c++, Java, Python)
    // that works with an image of cards and a Python-only example that
    // works with an image of coins.

    // Reproduce the standard Java example and also create a hybrid of
    // both examples that works with cards.

    //!! Note that the standard example misses the boundary between the
    // two cards at the center right; our hybrid method is better but
    // not perfect.

    // The OpenCV standard example is here --
    // https://docs.opencv.org/4.x/d2/dbd/tutorial_distance_transform.html,
    // which is implemented in this project as WatershedRecognitionStd,
    //
    // The OpenCV Python example is here --
    // https://docs.opencv.org/4.x/d3/db4/tutorial_py_watershed.html
    //
    // Returns the result of image analysis.
    public RobotConstants.RecognitionResults performWatershedStd(ImageProvider pImageProvider,
                                                                 VisionParameters.ImageParameters pImageParameters,
                                                                 WatershedRecognitionPath pWatershedRecognitionPath) throws InterruptedException {
        RobotLogCommon.d(TAG, "In WatershedRecognitionStd.performWatershedStd");

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
            case WATERSHED_CARDS_STD -> {
                return watershedCardsCPP(imageROI, outputFilenamePreamble);
            }
            case WATERSHED_CARDS_HYBRID -> {
                return watershedCardsHybrid(imageROI, outputFilenamePreamble);
            }
            default -> throw new AutonomousRobotException(TAG, "Unrecognized recognition path");
        }
    }

    private RobotConstants.RecognitionResults watershedCardsCPP(Mat pImageROI, String pOutputFilenamePreamble) {

        // Adapt the standard example to our environment.
        //!! Note that the example misses the card in the upper right.

        //! [black_bg]
        // Change the background from white to black, since that will help later to
        // extract better results during the use of Distance Transform
        //##PY This works because the cards are R 248, G 245, B 245.
        //##PY Shared by both paths.
        Mat blk = new Mat();
        blk = invertCardsBackground(pImageROI, pOutputFilenamePreamble);

        //##PY Try a sharpening kernel I got from stackoverflow.
        // The results are nearly identical - actually both methods
        // miss-classify the empty space just under the card in the
        // upper-right.
        Mat imgResult = ImageUtils.sharpen(blk, pOutputFilenamePreamble);

        //##PY The Laplacian filtering and the sharpening do make a difference.
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
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_THR.png", bw);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_THR.png");
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
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_DIST.png", distDisplay);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_DIST.png");
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
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_PEAK.png", distDisplay2);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_PEAK.png");
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
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_MARK.png", markersDisplay);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_MARK.png");

        Imgproc.circle(markers, new Point(5, 5), 3, new Scalar(255, 255, 255), -1);
        //! [seeds]

        //! [watershed]
        // Perform the watershed algorithm
        Imgproc.watershed(imgResult, markers);

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

        //**TODO Is this part shared also?
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
                if (index > 0 && index <= contours.size()) {
                    dstData[(i * dst.cols() + j) * 3 + 0] = (byte) colors.get(index - 1).val[0];
                    dstData[(i * dst.cols() + j) * 3 + 1] = (byte) colors.get(index - 1).val[1];
                    dstData[(i * dst.cols() + j) * 3 + 2] = (byte) colors.get(index - 1).val[2];
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

    private RobotConstants.RecognitionResults watershedCardsHybrid(Mat pImageROI, String pOutputFilenamePreamble) {

        Mat blk = new Mat();
        blk = invertCardsBackground(pImageROI, pOutputFilenamePreamble);

        // The Python example does not use a sharpening Kernel.
        // The standard example uses a multi-step sharpening pass
        // but the sharpening kernel I got from stackoverflow
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

        // Both standard examples use OTSU but we get better results (the
        // interiors of the cards go to white) with a straight binary
        // threshold.
        Mat bw = new Mat();
        Imgproc.threshold(redChannel, bw, 175, 255, Imgproc.THRESH_BINARY);

        return prepareAndExecuteWatershed(redChannel, pImageROI, sharp, 175, pOutputFilenamePreamble);
    }

    // Source: c++ example - specific to the cards image.
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

    public static RobotConstants.RecognitionResults prepareAndExecuteWatershed(Mat pGrayImage, Mat pImageROI, Mat pSharp,
                                                                               int pThresholdLow,
                                                                               String pOutputFilenamePreamble) {

        // Both standard examples use OTSU but we get better results (the
        // interiors of the cards go to white) with a straight binary
        // threshold.
        Mat bw = new Mat();
        Imgproc.threshold(pGrayImage, bw, pThresholdLow, 255, Imgproc.THRESH_BINARY);

        // Output the thresholded image.
        String thrFilename = pOutputFilenamePreamble + "_THR.png";
        Imgcodecs.imwrite(thrFilename, bw);
        RobotLogCommon.d(TAG, "Writing " + thrFilename);
        //! [bin]

        // Both Python examples perform two morphological openings but the
        // c++ example does not.

        // Follow the Python example and perform dilation for background identification.
        Mat sure_bg = new Mat();
        Mat dilateKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.dilate(bw, sure_bg, dilateKernel, new Point(-1, -1), 3);

        String bgFilename = pOutputFilenamePreamble + "_BG.png";
        Imgcodecs.imwrite(bgFilename, sure_bg);
        RobotLogCommon.d(TAG, "Writing " + bgFilename);

        //! [dist]
        // Follow both examples and perform the distance transform
        // algorithm. Imgproc.DIST_L2 is a flag for Euclidean distance.
        // Output is 32FC1.
        Mat dist = new Mat();
        Imgproc.distanceTransform(bw, dist, Imgproc.DIST_L2, 3);

        //##PY The normalization steps in the c++ example are not necessary
        // - just normalize to the range of 0 - 255.
        Core.normalize(dist, dist, 0.0, 255.0, Core.NORM_MINMAX);
        Mat dist_8u = new Mat();
        dist.convertTo(dist_8u, CvType.CV_8U);

        // Output the transformed image.
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_DIST.png", dist_8u);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_DIST.png");
        //! [dist]

        //! [peaks]
        // Follow the c++ example and threshold to obtain the peaks.
        // These will be the markers for the foreground objects.
        //##PY Since we've already normalized to a range of 0 - 255 we can replace this
        // Imgproc.threshold(dist, dist, 0.4, 1.0, Imgproc.THRESH_BINARY);
        Mat sure_fg = new Mat();
        Imgproc.threshold(dist_8u, sure_fg, 100, 255, Imgproc.THRESH_BINARY);

        // From the c++ example. The Python example does not do this.
        // Dilate a bit the thresholded image.
        Mat dilationKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.dilate(sure_fg, sure_fg, dilationKernel);

        // Output the foreground peaks.
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_FG.png", sure_fg);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_FG.png");
        //! [peaks]

        //! [seeds]
        //##PY Skip the conversion steps in the c++ example because we've
        // already created the 8-bit Mat dist_8u.

        // At last find the sure foreground objects.
        // The Python example uses connectedComponents; the c++ example uses findContours.
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(sure_fg, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        //#PY added - output the contours.
        Mat contoursOut = pImageROI.clone();
        ShapeDrawing.drawShapeContours(contours, contoursOut);
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_CON.png", contoursOut);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_CON.png");

        // Follow the Python example to find the unknown regions
        //  sure_fg = np.uint8(sure_fg)
        //  unknown = cv2.subtract(sure_bg, sure_fg)
        Mat unknown = new Mat();
        Core.subtract(sure_bg, sure_fg, unknown);

        Imgcodecs.imwrite(pOutputFilenamePreamble + "_UNK.png", unknown);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_UNK.png");

        // Create the markers for the watershed algorithm. From the comments
        // in the Python example: "The regions we know for sure (whether
        // foreground or background) are labelled with any positive integers,
        // but different integers, and the areas we don't know for sure are
        // just left as zero." So we'll start with markers initialized to 1
        // for the sure background.
        Mat markers = Mat.ones(dist.size(), CvType.CV_32S);

        // Follow the c++ example and dDraw the foreground markers.
        for (int i = 0; i < contours.size(); i++) {
            Imgproc.drawContours(markers, contours, i, new Scalar(i + 2), -1);
        }

        // Follow the Python example --
        // # Now, mark the region of unknown with zero
        // markers[unknown==255] = 0

        // Since we don't have that nice Python syntax,
        // we need to iterate through the Mat of unknowns and for every
        // white (255) value, set the marker at the some location to 0.
        // See https://answers.opencv.org/question/5/how-to-get-and-modify-the-pixel-of-mat-in-java/?answer=8#post-id-8

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
                if ((unknownData[sharedIndex] & 0xff) == 255) // Java doesn't have an unsigned byte!
                    markerData[sharedIndex] = 0;
            }
        }

        markers.put(0, 0, markerData); // back into Mat

        //! [watershed]
        // Perform the watershed algorithm
        Imgproc.watershed(pSharp, markers);

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

        // Fill labeled objects with random colors.
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
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_WS.png");
        //! [watershed]

        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }
}