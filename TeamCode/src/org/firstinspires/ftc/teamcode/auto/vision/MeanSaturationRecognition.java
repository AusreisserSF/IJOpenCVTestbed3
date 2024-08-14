package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.TimeStamp;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.xml.RecognitionWindowMapping;
import org.firstinspires.ftc.teamcode.auto.xml.VisionParameters;
import org.firstinspires.ftc.teamcode.auto.xml.WatershedParametersFtc;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.ArrayList;

//**TODO If you look at Marlborough's code, they use a very small
// window (40x40 pixels) and all three CenterStage spikes must
// be in view because the the saturation count is relative. But
// the technique may still be useful for getting a grayscale
// image.

public class MeanSaturationRecognition {

    private static final String TAG = MeanSaturationRecognition.class.getSimpleName();

    public enum MeanSaturationRecognitionPath {
        BRIGHT_SPOT, PIXEL_COUNT
        //**TODO MEAN_RANGE ??
    }

    private final RobotConstants.Alliance alliance;
    private final String testCaseDirectory;

    public MeanSaturationRecognition(RobotConstants.Alliance pAlliance, String pTestCaseDirectory) {
        alliance = pAlliance;
        testCaseDirectory = pTestCaseDirectory;
    }

    // Returns the result of image analysis.
    public RobotConstants.RecognitionResults performMeanSaturation(ImageProvider pImageProvider,
                                                                 VisionParameters.ImageParameters pImageParameters,
                                                                 MeanSaturationRecognitionPath pMedianSaturationRecognitionPath,
                                                                 //**TODO TEMP - use watershed parameters.
                                                                 WatershedParametersFtc pMedianSaturationParameters,
                                                                   RecognitionWindowMapping pRecognitionWindowMapping) throws InterruptedException {
        RobotLogCommon.d(TAG, "In MeanSaturationRecognition.performMeanSaturation");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> watershedImage = pImageProvider.getImage();
        if (watershedImage == null)
            return RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR; // don't crash

        // The image is in BGR order (OpenCV imread from a file).
        String fileDate = TimeStamp.getLocalDateTimeStamp(watershedImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(pImageParameters.image_source, testCaseDirectory, fileDate);
        Mat imageROI = ImageUtils.preProcessImage(watershedImage.first, outputFilenamePreamble, pImageParameters);
        RobotLogCommon.d(TAG, "Recognition path " + pMedianSaturationRecognitionPath);

        // Adapt the standard example to our environment.
        switch (pMedianSaturationRecognitionPath) {
            case BRIGHT_SPOT -> {
                return brightSpotPath(imageROI, outputFilenamePreamble, pMedianSaturationParameters.watershedDistanceParameters,
                        pRecognitionWindowMapping);
            }
            default -> throw new AutonomousRobotException(TAG, "Unrecognized recognition path");
        }
    }

    //**TODO The real method is SATURATION_LEVEL with a minimum that indicates the presence of a team prop.
    //**TODO bright spot - need minimum value.
    private RobotConstants.RecognitionResults brightSpotPath(Mat pImageROI, String pOutputFilenamePreamble, WatershedParametersFtc.WatershedDistanceParameters pWatershedDistanceParameters,
                                                             RecognitionWindowMapping pRecognitionWindowMapping) {

        //**TODO Don't need to threshold. WHY NOT?
        VisionParameters.GrayParameters allianceGrayParameters;
        switch (alliance) {
            case RED -> allianceGrayParameters = pWatershedDistanceParameters.redGrayParameters;
            case BLUE -> allianceGrayParameters = pWatershedDistanceParameters.blueGrayParameters;
            default -> throw new AutonomousRobotException(TAG, "distance transform requires an alliance selection");
        }

        // Convert to HSV and split the channels.
        // We're on the HSV path.
        Mat hsvROI = new Mat();
        Imgproc.cvtColor(pImageROI, hsvROI, Imgproc.COLOR_BGR2HSV);

        //**TODO Adjust the median saturation and value of the HSV image
        // then split.

        //**TODO Propagate ...
        // Split the image into its constituent HSV channels
        //ArrayList<Mat> channels = new ArrayList<>();
        //Core.split(hsvROI, channels);
        Mat saturationChannel = new Mat();
        Core.extractChannel(hsvROI, saturationChannel, 1);

        // Write out the S channel as grayscale.
        String satFilename = pOutputFilenamePreamble + "_SAT.png";
        Imgcodecs.imwrite(satFilename, saturationChannel);
        RobotLogCommon.d(TAG, "Writing " + satFilename);

        //**TODO Get the mean saturation of each spike window and see what kind
        // of differentiation we get and what kind of absolute numbers.

        // Define a submat for each spike window, get the mean of each submat
        // and print.
        Pair<Rect, RobotConstants.ObjectLocation> leftWindowData = pRecognitionWindowMapping.recognitionWindows.get(RobotConstants.RecognitionWindow.LEFT);
        Mat leftWindowSubmat = pImageROI.submat(leftWindowData.first);
        Scalar leftWindowMean = Core.mean(leftWindowSubmat);
        RobotLogCommon.d(TAG, "Left spike window saturation channel mean " + leftWindowMean.val[0]);

        Pair<Rect, RobotConstants.ObjectLocation> rightWindowData = pRecognitionWindowMapping.recognitionWindows.get(RobotConstants.RecognitionWindow.RIGHT);
        Mat rightWindowSubmat = pImageROI.submat(rightWindowData.first);
        Scalar rightWindowMean = Core.mean(rightWindowSubmat);
        RobotLogCommon.d(TAG, "Right spike window saturation channel mean " + rightWindowMean.val[0]);

        //Pair<Rect, RobotConstants.ObjectLocation> nposWindowData = pRecognitionWindowMapping.recognitionWindows.get(RobotConstants.RecognitionWindow.WINDOW_NPOS);

        Mat recognitionWindowsOut = pImageROI.clone();
        RecognitionWindowUtils.drawRecognitionWindows(recognitionWindowsOut, pOutputFilenamePreamble,
                pRecognitionWindowMapping.recognitionWindows);

        /*
        //**TODO Overlaps DistanceTransforRecognition.getDistanceTransformImage
        // https://docs.opencv.org/4.x/d3/db4/tutorial_py_watershed.html
        // Follow the standard Python example and threshold the grayscale.
        // Threshold the image: set pixels over the threshold value to white.
        //**TODO Is thresholding really necessary here? [YES, necessary for the distance transform]
        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(saturationChannel, thresholded,
                Math.abs(allianceGrayParameters.threshold_low),    // threshold value
                255,   // white
                Imgproc.THRESH_BINARY); // thresholding type
        RobotLogCommon.v(TAG, "Threshold values: low " + allianceGrayParameters.threshold_low + ", high 255");

        String thrFilename = pOutputFilenamePreamble + "_THR.png";
        Imgcodecs.imwrite(thrFilename, thresholded);
        RobotLogCommon.d(TAG, "Writing " + thrFilename);

        //**TODO is opening really necessary here? [YES, can help with distanceTransform]
        // Follow the standard Python example and perform two morphological openings on the thresholded image.
        Mat opened = new Mat();
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(thresholded, opened, Imgproc.MORPH_OPEN, kernel, new Point(-1, -1), 2);

        String openFilename = pOutputFilenamePreamble + "_OPEN.png";
        Imgcodecs.imwrite(openFilename, opened);
        RobotLogCommon.d(TAG, "Writing " + openFilename);

        // The distance transform identifies regions that are likely to be in
        // the foreground.
        //! [dist]
        // Perform the distance transform algorithm. Imgproc.DIST_L2
        // is a flag for Euclidean distance. Output is 32FC1.
        Mat dist = new Mat();
        Imgproc.distanceTransform(opened, dist, Imgproc.DIST_L2, 3);

        Core.normalize(dist, dist, 0.0, 255.0, Core.NORM_MINMAX);
        Mat dist_8u = new Mat();
        dist.convertTo(dist_8u, CvType.CV_8U);

        // Output the transformed image.
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_DIST.png", dist_8u);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_DIST.png");
        //! [dist]

        Core.MinMaxLocResult brightResult = Core.minMaxLoc(dist_8u);
        RobotLogCommon.d(TAG, "Bright spot location " + brightResult.maxLoc + ", value " + brightResult.maxVal);

        //**TODO Needs its own minimum.
        if (brightResult.maxVal < allianceGrayParameters.threshold_low / 2.0) {
            RobotLogCommon.d(TAG, "Bright spot value was under the threshold");
            return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
        }

        Mat brightSpotOut = pImageROI.clone();
        Imgproc.circle(brightSpotOut, brightResult.maxLoc, 10, new Scalar(0, 255, 0));

        Imgcodecs.imwrite(pOutputFilenamePreamble + "_BRIGHT.png", brightSpotOut);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_BRIGHT.png");

        //**TODO lookThroughWindows depends on the center of the recognized object.
        // Try comparing the mean saturation of window submats to a minimum value.
        return RecognitionWindowUtils.lookThroughWindows(brightResult.maxLoc, brightSpotOut, pOutputFilenamePreamble,
                pRecognitionWindowMapping.recognitionWindows);
         */

        return RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL;
    }

}