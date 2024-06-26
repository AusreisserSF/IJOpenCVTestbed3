package org.firstinspires.ftc.teamcode.auto.vision;

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
import java.util.Random;

public class WatershedRecognitionStd {

    private static final String TAG = WatershedRecognitionStd.class.getSimpleName();

    public enum WatershedRecognitionPath {
        DISTANCE
    }

    private final String testCaseDirectory;

    public WatershedRecognitionStd(String pTestCaseDirectory) {
        testCaseDirectory = pTestCaseDirectory;
    }

    // Port the standard OpenCV Watershed algorithm from the sample at --
    // https://docs.opencv.org/4.x/d2/dbd/tutorial_distance_transform.html
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
        //!! Note that the example misses the card in the upper right.

        //! [black_bg]
        // Change the background from white to black, since that will help later to
        // extract better results during the use of Distance Transform
        //##PY This works because the cards are R 248, G 245, B 245
        Mat src = imageROI.clone();
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
        Imgcodecs.imwrite(outputFilenamePreamble + "_BLK.png", src);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_BLK.png");

        //##PY Try a sharpening kernel I got from stackoverflow.
        // The results are nearly identical - actually both methods
        // miss-classify the empty space just under the card in the
        // upper-right.
        Mat imgResult = sharpen(src, outputFilenamePreamble);

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
        Imgcodecs.imwrite(outputFilenamePreamble + "_THR.png", bw);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_THR.png");
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
        Imgcodecs.imwrite(outputFilenamePreamble + "_DIST.png", distDisplay);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_DIST.png");
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
        Imgcodecs.imwrite(outputFilenamePreamble + "_PEAK.png", distDisplay2);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_PEAK.png");
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
        Mat contoursOut = imageROI.clone();
        ShapeDrawing.drawShapeContours(contours, contoursOut);
        Imgcodecs.imwrite(outputFilenamePreamble + "_CON.png", contoursOut);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_CON.png");

        // Create the marker image for the watershed algorithm
        Mat markers = Mat.zeros(dist.size(), CvType.CV_32S);

        // Draw the foreground markers
        for (int i = 0; i < contours.size(); i++) {
            Imgproc.drawContours(markers, contours, i, new Scalar(i + 1), -1);
        }

        // Draw the background marker
        //**TODO What is the point of both calls to Imgproc.circle
        Mat markersScaled = new Mat();
        markers.convertTo(markersScaled, CvType.CV_32F);
        Core.normalize(markersScaled, markersScaled, 0.0, 255.0, Core.NORM_MINMAX);
        Imgproc.circle(markersScaled, new Point(5, 5), 3, new Scalar(255, 255, 255), -1);
        Mat markersDisplay = new Mat();
        markersScaled.convertTo(markersDisplay, CvType.CV_8U);

        // Output the markers.
        Imgcodecs.imwrite(outputFilenamePreamble + "_MARK.png", markersDisplay);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_MARK.png");

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
        Imgcodecs.imwrite(outputFilenamePreamble + "_WS.png", dst);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "WS.png");
        //! [watershed]

        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
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