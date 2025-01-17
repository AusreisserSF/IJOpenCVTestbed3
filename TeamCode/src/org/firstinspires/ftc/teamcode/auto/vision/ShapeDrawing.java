package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.DebugImageCommon;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.List;

// Subset of the c++ ShapeDrawing class.
public class ShapeDrawing {
    private static final String TAG = ShapeDrawing.class.getSimpleName();

    // The parameters pContours is the output of a call to findContours.
    public static void drawShapeContours(List<MatOfPoint> pContours, Mat pImageOut) {
        Scalar color = new Scalar(0, 255, 0); // BGR green - good against dark background

        for (int i = 0; i < pContours.size(); i++) {
            Imgproc.drawContours(pImageOut, pContours, i, color, 2);
        }
    }

    public static void drawOneContour(List<MatOfPoint> pContours, int pContourIndex,
                                      Mat pImageOut, Scalar pColor) {
        Imgproc.drawContours(pImageOut, pContours, pContourIndex, pColor, 2);
    }

    // Thickness of < 0 means fill with color.
    public static void drawOneRectangle(Rect pRect, Mat pImageOut, int pThickness) {
        Imgproc.rectangle(pImageOut, pRect, new Scalar(0, 255, 0), pThickness); // GREEN
    }

    // Use this to show errors such as no contours or violations of criteria
    // such as size.
    public static void drawX(Mat pImageROI, Scalar pBGRColor, String pOutputFilenamePreamble) {

        double DRAWN_X_SIZE_FACTOR = 0.33;
        int drawnXThickness = 2;
        int roiWidth = pImageROI.cols();
        int roiHeight = pImageROI.rows();

        // Set the Points for the X.
        Point xUpperLeft = new Point(roiWidth * DRAWN_X_SIZE_FACTOR, roiHeight * DRAWN_X_SIZE_FACTOR);
        Point xUpperRight = new Point(roiWidth * (1.0 - DRAWN_X_SIZE_FACTOR), xUpperLeft.y);
        Point xLowerLeft = new Point(xUpperLeft.x, roiHeight * (1.0 - DRAWN_X_SIZE_FACTOR));
        Point xLowerRight = new Point(xUpperRight.x, xLowerLeft.y);

        Imgproc.line(pImageROI, xUpperLeft, xLowerRight, pBGRColor, drawnXThickness);
        Imgproc.line(pImageROI, xLowerLeft, xUpperRight, pBGRColor, drawnXThickness);

        if (RobotLogCommon.isLoggable(RobotLogCommon.CommonLogLevel.d)) {
            DebugImageCommon.writeImage(pOutputFilenamePreamble + "NO_BRECT.png", pImageROI);
            RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "NO_BRECT.png");
        }
    }

}
