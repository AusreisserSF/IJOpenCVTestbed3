package org.firstinspires.ftc.teamcode.auto.xml;

// Input parameters for testing rectangle filtering.
public class RedAllianceSampleParameters {
    public static final VisionParameters.GrayParameters redGrayParameters =
            new VisionParameters.GrayParameters(150, 200);
    private static final double max_distance_from_target_sample_center = 200.0;
    private static final double min_sample_area = 10000.0;
    private static final double max_sample_area = 21000.0;
    private static final double MIN_SAMPLE_ASPECT_RATIO = 1.6;
    private static final double MAX_SAMPLE_ASPECT_RATIO = 3.0;

    public static final SampleCriteria sampleCriteria =
            new SampleCriteria(max_distance_from_target_sample_center,
                    min_sample_area, max_sample_area,
                    MIN_SAMPLE_ASPECT_RATIO, MAX_SAMPLE_ASPECT_RATIO);

    public static class SampleCriteria {
        public final double max_distance_from_target_sample_center;
        public final double min_sample_area;
        public final double max_sample_area;
        public final double min_sample_aspect_ratio;
        public final double max_sample_aspect_ratio;

        public SampleCriteria(double pMaxDistanceFromTargetSampleCenter,
                              double pMinSampleArea, double pMaxSampleArea,
                              double pMinSampleAspectRatio, double pMaxSampleAspectRatio) {
            max_distance_from_target_sample_center = pMaxDistanceFromTargetSampleCenter;
            min_sample_area = pMinSampleArea;
            max_sample_area = pMaxSampleArea;
            min_sample_aspect_ratio = pMinSampleAspectRatio;
            max_sample_aspect_ratio = pMaxSampleAspectRatio;
        }

    }

}