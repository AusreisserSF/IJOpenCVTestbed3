package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class WatershedUtils {
    private static final String TAG = WatershedUtils.class.getSimpleName();

    // Applies the watershed algorithm and returns the watershed markers.
    // For an explanation of "hybrid" vs "standard" see the comments in
    // WatershedRecognition.java. The input Mat is a thresholded binary
    // image.
    public static Mat applyWatershedHybrid(Mat pBinaryImage, Mat pImageROI, Mat pSharp,
                                           String pOutputFilenamePreamble,
                                           String pOutputFilenameSuffix) {
        //! [bin]
        // Both Python examples perform two morphological openings but the
        // standard Java example does not.
        /*
        # noise removal
        kernel = np.ones((3,3),np.uint8)
        opening = cv.morphologyEx(thresh,cv.MORPH_OPEN,kernel, iterations = 2)

        Mat openKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(bw, bw, Imgproc.MORPH_OPEN, openKernel, new Point(-1, -1), 2);

        String openFilename = pOutputFilenamePreamble + "_OPEN.png";
        Imgcodecs.imwrite(openFilename, bw);
        RobotLogCommon.d(TAG, "Writing " + openFilename);
         */

        // Follow the Python example and perform dilation for background identification.
        Mat sure_bg = new Mat();
        Mat dilateKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.dilate(pBinaryImage, sure_bg, dilateKernel, new Point(-1, -1), 3);

        if (RobotLogCommon.isLoggable(RobotLogCommon.CommonLogLevel.vv)) {
            String bgFilename = pOutputFilenamePreamble + "_BG" + pOutputFilenameSuffix + ".png";
            Imgcodecs.imwrite(bgFilename, sure_bg);
            RobotLogCommon.vv(TAG, "Writing " + bgFilename);
        }

        //! [dist]
        // Follow both examples and perform the distance transform
        // algorithm. Imgproc.DIST_L2 is a flag for Euclidean distance.
        // Output is 32FC1.
        Mat dist = new Mat();
        Imgproc.distanceTransform(pBinaryImage, dist, Imgproc.DIST_L2, 3);

        //##PY The normalization steps in the c++ example are not necessary
        // - just normalize to the range of 0 - 255.
        Core.normalize(dist, dist, 0.0, 255.0, Core.NORM_MINMAX);
        Mat dist_8u = new Mat();
        dist.convertTo(dist_8u, CvType.CV_8U);

        // Output the transformed image.
        if (RobotLogCommon.isLoggable(RobotLogCommon.CommonLogLevel.v)) {
            Imgcodecs.imwrite(pOutputFilenamePreamble + "_DIST" + pOutputFilenameSuffix + ".png", dist_8u);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_DIST.png");
        }
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
        if (RobotLogCommon.isLoggable(RobotLogCommon.CommonLogLevel.vv)) {
            Imgcodecs.imwrite(pOutputFilenamePreamble + "_FG" + pOutputFilenameSuffix + ".png", sure_fg);
            RobotLogCommon.vv(TAG, "Writing " + pOutputFilenamePreamble + "_FG.png");
        }
        //! [peaks]

        //! [seeds]
        //##PY Skip the conversion steps in the standard Java example because
        // we've already created the 8-bit Mat dist_8u.

        // At last find the sure foreground objects.
        // The Python example uses connectedComponents; the c++ example uses findContours.
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(sure_fg, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        //#PY added - output the contours.
        Mat contoursOut = pImageROI.clone();
        ShapeDrawing.drawShapeContours(contours, contoursOut);
        if (RobotLogCommon.isLoggable(RobotLogCommon.CommonLogLevel.v)) {
            String conFilename = pOutputFilenamePreamble + "_CON" + pOutputFilenameSuffix + ".png";
            Imgcodecs.imwrite(conFilename, contoursOut);
            RobotLogCommon.v(TAG, "Writing " + conFilename);
        }

        // Follow the Python example to find the unknown regions
        //  sure_fg = np.uint8(sure_fg)
        //  unknown = cv2.subtract(sure_bg, sure_fg)
        Mat unknown = new Mat();
        Core.subtract(sure_bg, sure_fg, unknown);

        if (RobotLogCommon.isLoggable(RobotLogCommon.CommonLogLevel.vv)) {
            String unkFilename = pOutputFilenamePreamble + "_UNK" + pOutputFilenameSuffix + ".png";
            Imgcodecs.imwrite(unkFilename, unknown);
            RobotLogCommon.vv(TAG, "Writing " + unkFilename);
        }

        // Create the markers for the watershed algorithm. From the comments
        // in the Python example: "The regions we know for sure (whether
        // foreground or background) are labelled with any positive integers,
        // but different integers, and the areas we don't know for sure are
        // just left as zero." So we'll start with markers initialized to 1
        // for the sure background.
        Mat markers = Mat.ones(dist.size(), CvType.CV_32S);

        // Follow the standard Java example and draw the foreground markers.
        for (int i = 0; i < contours.size(); i++) {
            Imgproc.drawContours(markers, contours, i, new Scalar(i + 2), -1);
        }

        // Follow the Python example --
        // # Now, mark the region of unknown with zero
        // markers[unknown==255] = 0

        // Since we don't have that nice Python syntax,
        // we need to iterate through the Mat of unknowns and for every
        // white (255) value, set the marker at the same location to 0.
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
                if (((int) unknownData[sharedIndex] & 0xff) == 255) // Java doesn't have an unsigned byte!
                    markerData[sharedIndex] = 0;
            }
        }

        markers.put(0, 0, markerData); // back into Mat

        // Draw the markers - scaled so that they show - and with the
        // unknowns merged in. Note that there is only a small difference
        // between the unknown regions (at level 0) and the background.
        Mat markersScaled = new Mat();
        markers.convertTo(markersScaled, CvType.CV_32F);
        Core.normalize(markersScaled, markersScaled, 0.0, 255.0, Core.NORM_MINMAX);
        Mat markersDisplay = new Mat();
        markersScaled.convertTo(markersDisplay, CvType.CV_8U);

        // Output the markers.
        if (RobotLogCommon.isLoggable(RobotLogCommon.CommonLogLevel.vv)) {
            String markFilename = pOutputFilenamePreamble + "_MARK" + pOutputFilenameSuffix + ".png";
            Imgcodecs.imwrite(markFilename, markersDisplay);
            RobotLogCommon.vv(TAG, "Writing " + markFilename);
        }

        //! [watershed]
        // Perform the watershed algorithm
        Imgproc.watershed(pSharp, markers);
        return markers;
    }
}
