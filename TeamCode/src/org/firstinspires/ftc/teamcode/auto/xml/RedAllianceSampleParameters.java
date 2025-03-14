package org.firstinspires.ftc.teamcode.auto.xml;

// Input parameters for testing rectangle filtering.
public class RedAllianceSampleParameters {
    public static final VisionParameters.GrayParameters redGrayParameters =
            new VisionParameters.GrayParameters(150, 200);

    // At the fixed shoulder distance of LIMELIGHT_LEVEL from
    // the camera to a sample in the submersible at the center
    // of the camera's field of view there are 175 pixels at
    // an image resolution 640x480 for the 3.5" width of a
    // sample. This translates to 50 px/in.

    private static final double SAMPLE_WIDTH_IN = 3.5;
    private static final double MIN_SAMPLE_WIDTH_PX = 170.0;
    private static final double MAX_SAMPLE_WIDTH_PX = 270.0;
    private static final double DEFAULT_SAMPLE_WIDTH_PX = 175.0;
    private static final double DEFAULT_PX_PER_IN = DEFAULT_SAMPLE_WIDTH_PX / SAMPLE_WIDTH_IN;

    //!! Note that there is a dependency between the area limits
    // and the default DEFAULT_SAMPLE_WIDTH_PX value above.
    private static final double MIN_SAMPLE_AREA = 10000.0;
    private static final double MAX_SAMPLE_AREA = 21000.0;

    private static final double MIN_SAMPLE_ASPECT_RATIO = 1.6;
    private static final double MAX_SAMPLE_ASPECT_RATIO = 3.0;

    // This value should be in inches and should be at least 1/2 the width of a sample,
    // i.e. 1.75 in + the height of the center pickup zone .7 + a margin .25 = 2.7" - in
    // case of a T configuration.
    //**TODO This value could be computed from the CENTER pickup zone.
    // <max_distance_from_target_sample_center>200</max_distance_from_target_sample_center>
    // Do this in SampleRecognition.java. But for now just plug in a value here:
    private static final double MAX_DISTANCE_FROM_TARGET_SAMPLE_CENTER = 2.7;

    public static final SampleCriteria sampleCriteria =
            new SampleCriteria(MIN_SAMPLE_WIDTH_PX, MAX_SAMPLE_WIDTH_PX,
                    MAX_DISTANCE_FROM_TARGET_SAMPLE_CENTER,
                    MIN_SAMPLE_AREA, MAX_SAMPLE_AREA,
                    MIN_SAMPLE_ASPECT_RATIO, MAX_SAMPLE_ASPECT_RATIO);

    public static class SampleCriteria {
        public final double min_sample_width_px;
        public final double max_sample_width_px;
        public final double max_distance_from_target_sample_center;
        public final double min_sample_area;
        public final double max_sample_area;
        public final double min_sample_aspect_ratio;
        public final double max_sample_aspect_ratio;

        public SampleCriteria(double pMinSampleWidthPx, double pMaxSampleWidthPx,
                              double pMinSampleArea, double pMaxSampleArea,
                              double pMinSampleAspectRatio, double pMaxSampleAspectRatio,
                              double pMaxDistanceFromTargetSampleCenter) {
            min_sample_width_px = pMinSampleWidthPx;
                    max_sample_width_px = pMaxSampleWidthPx;
            min_sample_area = pMinSampleArea;
            max_sample_area = pMaxSampleArea;
            min_sample_aspect_ratio = pMinSampleAspectRatio;
            max_sample_aspect_ratio = pMaxSampleAspectRatio;
            max_distance_from_target_sample_center = pMaxDistanceFromTargetSampleCenter;
        }

    }

}