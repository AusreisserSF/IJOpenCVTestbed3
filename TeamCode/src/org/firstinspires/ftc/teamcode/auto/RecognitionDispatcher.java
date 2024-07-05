package org.firstinspires.ftc.teamcode.auto;

import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;
import javafx.scene.text.Text;
import javafx.stage.Stage;
import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.WorkingDirectory;
import org.firstinspires.ftc.ftcdevcommon.xml.RobotXMLElement;
import org.firstinspires.ftc.ftcdevcommon.xml.XPathAccess;
import org.firstinspires.ftc.teamcode.auto.vision.*;
import org.firstinspires.ftc.teamcode.auto.xml.GoldCubeParametersXML;
import org.firstinspires.ftc.teamcode.auto.xml.RobotActionXML;
import org.firstinspires.ftc.teamcode.auto.xml.WatershedParametersFtcXML;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.ByteArrayInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.List;
import java.util.Map;

public class RecognitionDispatcher extends Application {

    private static final String TAG = RecognitionDispatcher.class.getSimpleName();

    public static final double FIELD_WIDTH = 400;
    public static final double FIELD_HEIGHT = 400;

    public static final double NOTIFICATION_TEXT_POSITION_Y = (FIELD_HEIGHT / 2) + 20;
    public static final double NOTIFICATION_TEXT_POSITION_X = 10;
    public static final Font NOTIFICATION_TEXT_FONT = Font.font("Comic Sans MS", FontWeight.BOLD, 16);

    private Stage stage;
    private Pane field;

    // Load OpenCV.
    private static final boolean openCVInitialized;

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME); // IntelliJ only
        openCVInitialized = true; // IntelliJ only
    }

// The directory structure looks like this:
// Files
//     test_case_1
//     test_case_2
//     test_case_3

    // See https://stackoverflow.com/questions/21729362/trying-to-call-a-javafx-application-from-java-nosuchmethodexception
    //!! Default constructor is required.
    public RecognitionDispatcher() {
        RobotLogCommon.i(TAG, "Starting image recognition");

        if (!openCVInitialized)
            throw new AutonomousRobotException(TAG, "Failure in OpenCV initialization");
    }

    @Override
    public void start(Stage pStage) throws Exception {
        // Process the command line parameters.
        Parameters parameters = getParameters();
        Map<String, String> namedParameters = parameters.getNamed();

        // Get the name of the test case directory as the first argument on the command line.
        String testCase = parameters.getUnnamed().getFirst();
        if (testCase == null) {
            System.out.println(TAG + " Missing test case name");
            return;
        }

        String fullTestCaseDir = WorkingDirectory.getWorkingDirectory() + "/" + testCase + "/";

        // Create a test log that looks like this example from the c++ project OpenCVTestbed3:
        // TestLog_2024-06-06_0817-43.759_run.txt
        // However, since all Java projects use the native Java logger with file versioning,
        // the file name of the log will come out as equivalent:
        // TestLog_2024-06-06_0817:43.759.txt.0
        RobotLogCommon.initialize(RobotLogCommon.LogIdentifier.TEST_LOG, fullTestCaseDir);

        // Check the contents of a frequently used optional command line argument.
        RobotConstants.Alliance alliance = RobotConstants.Alliance.NONE;
        String allianceParameter = namedParameters.get("alliance"); // optional
        if (allianceParameter != null)
            alliance = RobotConstants.Alliance.valueOf(allianceParameter);

        RobotLogCommon.c(TAG, "Alliance " + alliance);

        // Each test case directory has its own RobotAction.xml, in which there
        // is a single OpMode TEST. Under this OpMode the <actions> element must
        // contain a single child element, whose name is that of the action
        // (test case).
        RobotActionXML robotActionXML = new RobotActionXML(fullTestCaseDir + RobotConstants.ACTION_FILENAME);
        RobotActionXML.RobotActionData actionData = robotActionXML.getOpModeData("TEST");
        if (actionData.actionElements.size() != 1)
            throw new AutonomousRobotException(TAG, "TEST OpMode must contain a single action");

        RobotLogCommon.setMostDetailedLogLevel(actionData.logLevel);
        List<RobotXMLElement> actionElements = actionData.actionElements;

        // Set up XPath access to the current action.
        RobotXMLElement actionElement = actionElements.getFirst();
        XPathAccess actionXPath = new XPathAccess(actionElement);

        String actionName = actionElement.getRobotXMLElementName().toUpperCase();
        RobotLogCommon.d(TAG, "Executing action " + actionName);

        // Initialize the JavaFX display.
        stage = pStage;
        field = new Pane();

        String imageFilename;
        switch (actionName) {
            // Reference implementation for the standard gold cube.
            case "FIND_GOLD_CUBE" -> {
                // Read the parameters for gold cube recognition from the xml file.
                GoldCubeParametersXML goldCubeParametersXML = new GoldCubeParametersXML(fullTestCaseDir);
                GoldCubeParameters goldCubeParameters = goldCubeParametersXML.getGoldCubeParameters();

                // Get the <image_parameters> for the gold cube from the RobotAction XML file.
                VisionParameters.ImageParameters goldCubeImageParameters =
                        robotActionXML.getImageParametersFromXPath(actionElement, "image_parameters");

                // Make sure that this tester is reading the image from a file.
                if (!(goldCubeImageParameters.image_source.endsWith(".png") ||
                        goldCubeImageParameters.image_source.endsWith(".jpg")))
                    throw new AutonomousRobotException(TAG, "Invalid image file name");

                imageFilename = goldCubeImageParameters.image_source;
                ImageProvider fileImage = new FileImage(fullTestCaseDir + goldCubeImageParameters.image_source);

                // Perform image recognition.
                // Get the recognition path from the XML file.
                String recognitionPathString = actionXPath.getRequiredText("gold_cube_recognition/recognition_path");
                GoldCubeRecognition.GoldCubeRecognitionPath goldCubeRecognitionPath =
                        GoldCubeRecognition.GoldCubeRecognitionPath.valueOf(recognitionPathString.toUpperCase());

                RobotLogCommon.d(TAG, "Recognition path " + goldCubeRecognitionPath);

                // Perform image recognition.
                GoldCubeRecognition goldCubeRecognition = new GoldCubeRecognition(fullTestCaseDir, alliance);
                RobotConstants.RecognitionResults goldCubeReturn =
                        goldCubeRecognition.recognizeGoldCubeWebcam(fileImage, goldCubeImageParameters, goldCubeParameters, goldCubeRecognitionPath);

                displayResults(fullTestCaseDir + goldCubeImageParameters.image_source,
                        buildResultsOnlyDisplayText(imageFilename, goldCubeReturn),
                        "Test gold cube recognition");
            }

            // Standard OpenCV Watershed example from --
            // https://docs.opencv.org/4.x/d2/dbd/tutorial_distance_transform.html
            case "WATERSHED_STD" -> {
                // Get the <image_parameters> for the playing cards from the RobotAction XML file.
                VisionParameters.ImageParameters watershedImageParameters =
                        robotActionXML.getImageParametersFromXPath(actionElement, "image_parameters");

                // Make sure that this tester is reading the image from a file.
                if (!(watershedImageParameters.image_source.endsWith(".png") ||
                        watershedImageParameters.image_source.endsWith(".jpg")))
                    throw new AutonomousRobotException(TAG, "Invalid image file name");

                imageFilename = watershedImageParameters.image_source;
                ImageProvider fileImage = new FileImage(fullTestCaseDir + watershedImageParameters.image_source);

                // Perform image recognition.
                // Get the recognition path from the XML file.
                String recognitionPathString = actionXPath.getRequiredText("watershed_recognition/recognition_path");
                WatershedRecognitionStd.WatershedRecognitionPath watershedRecognitionPath =
                        WatershedRecognitionStd.WatershedRecognitionPath.valueOf(recognitionPathString.toUpperCase());

                RobotLogCommon.d(TAG, "Recognition path " + watershedRecognitionPath);

                // Perform image recognition.
                WatershedRecognitionStd watershedRecognitionStd = new WatershedRecognitionStd(fullTestCaseDir);
                RobotConstants.RecognitionResults watershedStdReturn =
                        watershedRecognitionStd.performWatershedStd(fileImage, watershedImageParameters, watershedRecognitionPath);

                displayResults(fullTestCaseDir + imageFilename,
                        buildResultsOnlyDisplayText(imageFilename, watershedStdReturn),
                        "Test standard OpenCV Watershed");
            }

            case "WATERSHED_FTC" -> {
                WatershedParametersFtcXML watershedParametersFtcXML = new WatershedParametersFtcXML(fullTestCaseDir);
                WatershedParametersFtc watershedParametersFtc = watershedParametersFtcXML.getWatershedParameters();

                // Get the <image_parameters> for the playing cards from the RobotAction XML file.
                VisionParameters.ImageParameters watershedImageParameters =
                        robotActionXML.getImageParametersFromXPath(actionElement, "image_parameters");

                // Make sure that this tester is reading the image from a file.
                if (!(watershedImageParameters.image_source.endsWith(".png") ||
                        watershedImageParameters.image_source.endsWith(".jpg")))
                    throw new AutonomousRobotException(TAG, "Invalid image file name");

                imageFilename = watershedImageParameters.image_source;
                ImageProvider fileImage = new FileImage(fullTestCaseDir + watershedImageParameters.image_source);

                // Perform image recognition.
                // Get the recognition path from the XML file.
                String recognitionPathString = actionXPath.getRequiredText("watershed_recognition/recognition_path");
                WatershedRecognitionFtc.WatershedRecognitionPath watershedRecognitionPath =
                        WatershedRecognitionFtc.WatershedRecognitionPath.valueOf(recognitionPathString.toUpperCase());

                RobotLogCommon.d(TAG, "Recognition path " + watershedRecognitionPath);

                // Perform image recognition.
                WatershedRecognitionFtc watershedRecognitionFtc = new WatershedRecognitionFtc(alliance, fullTestCaseDir);
                RobotConstants.RecognitionResults watershedFtcReturn =
                        watershedRecognitionFtc.performWatershedFtc(fileImage, watershedImageParameters,
                                watershedRecognitionPath, watershedParametersFtc);

                displayResults(fullTestCaseDir + imageFilename,
                        buildResultsOnlyDisplayText(imageFilename, watershedFtcReturn),
                        "Test standard OpenCV Watershed");
            }

            case "DISTANCE" -> {
                DistanceParametersXML distanceParametersXML = new DistanceParametersXML(fullTestCaseDir);
                DistanceParameters distanceParameters = distanceParametersXML.getDistanceParameters();

                // Get the <image_parameters> for the playing cards from the RobotAction XML file.
                VisionParameters.ImageParameters watershedImageParameters =
                        robotActionXML.getImageParametersFromXPath(actionElement, "image_parameters");

                // Make sure that this tester is reading the image from a file.
                if (!(watershedImageParameters.image_source.endsWith(".png") ||
                        watershedImageParameters.image_source.endsWith(".jpg")))
                    throw new AutonomousRobotException(TAG, "Invalid image file name");

                imageFilename = watershedImageParameters.image_source;
                ImageProvider fileImage = new FileImage(fullTestCaseDir + watershedImageParameters.image_source);

                // Perform image recognition.
                // Get the recognition path from the XML file.
                String recognitionPathString = actionXPath.getRequiredText("watershed_recognition/recognition_path");
                DistanceTransformRecognition.DistanceTransformRecognitionPath distanceRecognitionPath =
                        DistanceTransformRecognition.DistanceTransformRecognitionPath.valueOf(recognitionPathString.toUpperCase());

                RobotLogCommon.d(TAG, "Recognition path " + distanceRecognitionPath);

                // Perform image recognition.
                DistanceTransformRecognition distanceTransformRecognition = new DistanceTransformRecognition(alliance, fullTestCaseDir);
                RobotConstants.RecognitionResults distanceReturn =
                        distanceTransformRecognition.performDistanceTransform(fileImage, watershedImageParameters,
                                distanceRecognitionPath, distanceParameters);

                displayResults(fullTestCaseDir + imageFilename,
                        buildResultsOnlyDisplayText(imageFilename, distanceReturn),
                        "Test standard OpenCV Watershed");
            }

            default -> throw new AutonomousRobotException(TAG, "Unrecognized image recognition action");
        }

        RobotLogCommon.closeLog();
    }

    private String buildResultsOnlyDisplayText(String pImageFilename, RobotConstants.RecognitionResults pRecognitionReturn) {
        return "Image: " +
                pImageFilename +
                '\n' +
                pRecognitionReturn +
                '\n';
    }

    private void displayResults(Mat pImageMat, String pResultText, String pTitle) {
        // From https://stackoverflow.com/questions/27755171/display-opencv-mat-with-javafx
        MatOfByte byteMat = new MatOfByte();
        Imgcodecs.imencode(".png", pImageMat, byteMat);
        Image image = new Image(new ByteArrayInputStream(byteMat.toArray()));
        displayResults(image, pResultText, pTitle);
    }

    private void displayResults(String pImageFile, String pResultText, String pTitle) throws FileNotFoundException {
        InputStream stream = new FileInputStream(pImageFile);
        Image image = new Image(stream);
        displayResults(image, pResultText, pTitle);
    }

    // Display the image in the Pane.
    private void displayResults(Image pImage, String pResultText, String pTitle) {
        ImageView imageView = new ImageView();
        imageView.setImage(pImage);
        imageView.setX(0);
        imageView.setY(0);
        imageView.setFitWidth(FIELD_WIDTH / 2);
        imageView.setFitHeight(FIELD_HEIGHT / 2);
        imageView.setPreserveRatio(true);
        field.getChildren().add(imageView);

        // Write text to field.
        Text displayText = new Text(pResultText);
        displayText.setFont(NOTIFICATION_TEXT_FONT);
        displayText.setFill(Color.CYAN);
        displayText.setX(NOTIFICATION_TEXT_POSITION_X);
        displayText.setY(NOTIFICATION_TEXT_POSITION_Y);
        field.getChildren().add(displayText);

        Scene scene = new Scene(field, FIELD_WIDTH, FIELD_HEIGHT, Color.GRAY);
        stage.setTitle(pTitle);
        stage.setScene(scene);
        stage.show();
    }

}