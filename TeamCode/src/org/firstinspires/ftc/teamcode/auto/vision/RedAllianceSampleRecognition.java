package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.TimeStamp;
import org.firstinspires.ftc.teamcode.auto.DebugImageCommon;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.xml.RedAllianceSampleParameters;
import org.firstinspires.ftc.teamcode.auto.xml.VisionParameters;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;

public class RedAllianceSampleRecognition {

    private static final String TAG = RedAllianceSampleRecognition.class.getSimpleName();

    public enum RecognitionPath {
        RED_CHANNEL_GRAYSCALE
    }

    private final String testCaseDirectory;

    public RedAllianceSampleRecognition(String pTestCaseDirectory) {
        testCaseDirectory = pTestCaseDirectory;
    }

    // FTC 2024-2025 IntoTheDeep.
    public RobotConstants.RecognitionResults recognizeRedAllianceSamples(ImageProvider pImageProvider,
                                                                         VisionParameters.ImageParameters pImageParameters,
                                                                         RecognitionPath pRecognitionPath) throws InterruptedException {

        RobotLogCommon.d(TAG, "In RedAllianceSampleRecognition.recognizeRedAllianceSamples");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> redAllianceSampleImage = pImageProvider.getImage();
        if (redAllianceSampleImage == null)
            return RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR; // don't crash

        // The image is in BGR order (OpenCV imread from a file).
        String fileDate = TimeStamp.getLocalDateTimeStamp(redAllianceSampleImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(pImageParameters.image_source, testCaseDirectory, fileDate);
        Mat imageROI = ImageUtils.preProcessImage(redAllianceSampleImage.first, outputFilenamePreamble, pImageParameters);

        RobotLogCommon.d(TAG, "Recognition path " + pRecognitionPath);
        if (pRecognitionPath != RecognitionPath.RED_CHANNEL_GRAYSCALE)
            throw new AutonomousRobotException(TAG, "Unrecognized recognition path");

        return redChannelPath(imageROI, outputFilenamePreamble);
    }

    private RobotConstants.RecognitionResults redChannelPath(Mat pImageROI, String pOutputFilenamePreamble) {

        // Extract the red channel and then use it as grayscale.
        // The red channel will pick up both the red and yellow samples.
        Mat selectedChannel = new Mat();
        Core.extractChannel(pImageROI, selectedChannel, 2);

        // Write out the red channel as grayscale.
        if (RobotLogCommon.isLoggable(RobotLogCommon.CommonLogLevel.d)) {
            DebugImageCommon.writeImage(pOutputFilenamePreamble + "_RED_CHANNEL.png", selectedChannel);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_RED_CHANNEL.png");
        }

        Mat thresholded = ImageUtils.performThresholdOnGray(selectedChannel,
                RedAllianceSampleParameters.redGrayParameters.median_target,
                RedAllianceSampleParameters.redGrayParameters.threshold_low,
                pOutputFilenamePreamble, "_RED_THR");

        // Sanitize the thresholded red alliance samples by eliminating contours
        // that are below the minimum area threshold.
        // Using 1/2 of the default min_sample_area is somewhat imprecise
        // because this value is based on SampleParameters.DEFAULT_PX_PER_IN
        // but we don't know the current value for px/in yet.
        ImageUtils.FilteredContoursRecord filteredA = ImageUtils.filterContours(thresholded, pImageROI.rows(), pImageROI.cols(),
                RedAllianceSampleParameters.sampleCriteria.min_sample_area / 2.0,
                pOutputFilenamePreamble, "_A");

        // Log statistics.
        RobotLogCommon.d(TAG, "numUnfilteredAllianceContours " + filteredA.numUnfilteredContours);
        RobotLogCommon.d(TAG, "numFilteredAllianceContours " + filteredA.numFilteredContours);

        // Collect complete rectangles and irregular shapes
        // i.e. those contours that fail the rectangle test.
        List<RotatedRect> rectanglesFound = new ArrayList<>();
        List<MatOfPoint> irregularShapes = new ArrayList<>();
        RotatedRect oneRotatedRect;
        for (MatOfPoint oneContour : filteredA.filteredContours) {
            oneRotatedRect = findRectangle(oneContour);
            if (oneRotatedRect != null) {
                rectanglesFound.add(oneRotatedRect);
            } else { // irregular shape
                Point irregularShapeCentroid = ImageUtils.getContourCentroid(oneContour);
                double irregularContourArea = Imgproc.contourArea(oneContour);
                RobotLogCommon.d(TAG, "Found an irregular shape with center x " +
                        irregularShapeCentroid.x + ", y " + irregularShapeCentroid.y);
                RobotLogCommon.d(TAG, "Irregular shape area " + irregularContourArea);

                // Check against the default minimum sample area.
                if (irregularContourArea < RedAllianceSampleParameters.sampleCriteria.min_sample_area / 2.0)
                    RobotLogCommon.d(TAG, "Irregular shape is under the minimum area; skipping");
                else
                    irregularShapes.add(oneContour);
            }
        }

        RobotLogCommon.d(TAG, "Number of irregular shapes " + irregularShapes.size());

        // Skip the remaining steps if we haven't found any rectangles
        // at all.
        if (rectanglesFound.isEmpty()) {
            RobotLogCommon.d(TAG, "No rectangles found in the image");
            return RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL;
        }

        // Filter the rectangles on their proximity to the edges of
        // the image and on their aspect ratio.
        Point[] rectPoints = new Point[4];
        List<RotatedRect> filteredPass1 = new ArrayList<>();
        for (RotatedRect oneFoundRect : rectanglesFound) {
            oneFoundRect.points(rectPoints);

            RobotLogCommon.d(TAG, "Found a rectangle with center x " + oneFoundRect.center.x +
                    ", y " + oneFoundRect.center.y);
            RobotLogCommon.d(TAG, "Rectangle width " + oneFoundRect.size.width +
                    ", height " + oneFoundRect.size.height +
                    ", area " + (oneFoundRect.size.width * oneFoundRect.size.height));
            RobotLogCommon.d(TAG, "Rotated rect points 0 " + rectPoints[0] +
                    ", 1 " + rectPoints[1] + ", 2 " + rectPoints[2] + ", 3 " + rectPoints[3]);

            int imageWidth = pImageROI.cols();
            int imageHeight = pImageROI.rows();
            boolean outOfBoundsRectFound = false;
            for (Point rectPoint : rectPoints) {
                // If any of the four points has a negative x or y coordinate
                // then part of the rectangle is out of the image area.
                if (rectPoint.x < 0 || rectPoint.y < 0) {
                    outOfBoundsRectFound = true;
                    break;
                }

                // Sometimes the boundary of a rotated rectangle is placed
                // in close proximity to an edge of the image. Check each
                // vertex against 1/2 of the default px/in value.
                double proximityToEdge = RedAllianceSampleParameters.DEFAULT_PX_PER_IN / 2.0;
                if (rectPoint.x < proximityToEdge || rectPoint.x > (imageWidth - proximityToEdge) ||
                        (rectPoint.y < proximityToEdge || rectPoint.y > (imageHeight - proximityToEdge))) {
                    outOfBoundsRectFound = true;
                    break;
                }
            }

            if (outOfBoundsRectFound) {
                RobotLogCommon.d(TAG, "Rectangle has a vertex that is out of bounds for recognition; skipping");
                continue;
            }

            // Filter on aspect ratio of the long side of a sample / short side
            double longSide = Math.max(oneFoundRect.size.width, oneFoundRect.size.height);
            double shortSide = Math.min(oneFoundRect.size.width, oneFoundRect.size.height);
            double aspectRatio = longSide / shortSide;
            if (aspectRatio < RedAllianceSampleParameters.sampleCriteria.min_sample_aspect_ratio ||
                    aspectRatio > RedAllianceSampleParameters.sampleCriteria.max_sample_aspect_ratio) {
                RobotLogCommon.d(TAG, "Aspect ratio of " + aspectRatio + " is outside the bounds of a sample; skipping");
            } else
                filteredPass1.add(oneFoundRect);
        }

        // Skip the remaining steps if no rectangles have passed
        // the first filter.
        if (filteredPass1.isEmpty()) {
            RobotLogCommon.d(TAG, "No rectangles passed the first filter");
            return RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL;
        }

        // Now we have a collection of complete and in-bounds rectangles.
        // Loop through them and make sure they pass the area checks.
        List<RotatedRect> filteredPass2 = new ArrayList<>();
        for (RotatedRect oneFiltered1Rect : filteredPass1) {
            // Get px/in
            double longSide = Math.max(oneFiltered1Rect.size.width, oneFiltered1Rect.size.height);
            double shortSide = Math.min(oneFiltered1Rect.size.width, oneFiltered1Rect.size.height);

            // Validate
            if (longSide < RedAllianceSampleParameters.MIN_SAMPLE_WIDTH_PX ||
                    longSide > RedAllianceSampleParameters.MAX_SAMPLE_WIDTH_PX) {
                RobotLogCommon.d(TAG, "Px/in of rectangle with center x " +
                        oneFiltered1Rect.center.x +
                        ", y " + oneFiltered1Rect.center.y + " is out of range");
                continue;
            }

            // Scale the long and short sides of the rectangle to the default px/in.
            double filtered1PxPerIn = longSide / RedAllianceSampleParameters.SAMPLE_WIDTH_IN;
            double scaledLong = (longSide * RedAllianceSampleParameters.DEFAULT_PX_PER_IN) / filtered1PxPerIn;
            double scaledShort = (shortSide * RedAllianceSampleParameters.DEFAULT_PX_PER_IN) / filtered1PxPerIn;
            double scaledArea = scaledLong * scaledShort;

            if (scaledArea < RedAllianceSampleParameters.sampleCriteria.min_sample_area) {
                RobotLogCommon.d(TAG, "Area of rotated rectangle is below the minimum");
                continue;
            }

            if (scaledArea > RedAllianceSampleParameters.sampleCriteria.max_sample_area) {
                RobotLogCommon.d(TAG, "Area of rotated rectangle is above the maximum");
                continue;
            }

            filteredPass2.add(oneFiltered1Rect);
        }

        // We need at least one rectangle that passed all of the filters.
        if (filteredPass2.isEmpty()) {
            RobotLogCommon.d(TAG, "No rectangles passed the second filter");
            return RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL;
        }

        // Getting close - draw outlines around each filtered rectangle.
        Mat drawnRects = pImageROI.clone();
        List<MatOfPoint> rectContours = new ArrayList<>();
        for (RotatedRect drawOneRect : filteredPass2) {
            drawOneRect.points(rectPoints);
            rectContours.add(new MatOfPoint(rectPoints)); // List is required
            Imgproc.drawContours(drawnRects, rectContours, 0, new Scalar(0, 255, 0), 2);
            rectContours.clear();
        }

        // Write a file with the rectangles.
        if (RobotLogCommon.isLoggable(RobotLogCommon.CommonLogLevel.d)) {
            String fullFilename = pOutputFilenamePreamble + "_VRECT" + "_A" + ".png";
            DebugImageCommon.writeImage(fullFilename, drawnRects);
            RobotLogCommon.d(TAG, "Writing " + fullFilename);
        }

        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }

    // Based on https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
    private RotatedRect findRectangle(MatOfPoint thisContour) {

        RotatedRect retVal = null;

        MatOfPoint2f thisContour2f = new MatOfPoint2f();
        MatOfPoint2f approxContour2f = new MatOfPoint2f();

        thisContour.convertTo(thisContour2f, CvType.CV_32FC2);

        double perimeter = Imgproc.arcLength(thisContour2f, true);
        Imgproc.approxPolyDP(thisContour2f, approxContour2f, 0.04 * perimeter, true);

        double height = approxContour2f.size().height;
        double width = approxContour2f.size().width;
        RobotLogCommon.d(TAG, "Polygon height " + height + ", width " + width);
        if ((height >= 4) && (height <= 6))
            retVal = Imgproc.minAreaRect(approxContour2f);

        return retVal;
    }

}