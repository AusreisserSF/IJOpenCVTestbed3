package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;

import org.opencv.core.Rect;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathConstants;
import javax.xml.xpath.XPathExpressionException;
import javax.xml.xpath.XPathFactory;
import java.io.File;
import java.io.IOException;
import java.util.EnumMap;

//**TODO Revise comment ...
// The purpose of this class is to collect information from the
// FIND_TEAM_PROP element of each of the Autonomous OpModes in
// RobotConfig.xml associated with a starting position in the
// competition, e.g. BLUE_A2, RED_F4. The collected information
// can be used to crop webcam images and show spike mark windows
// in the Driver Station camera stream. Before a match the driver
// can then adjust the camera alignment.
public class RecognitionWindowMappingXML {

    public static final String TAG = RecognitionWindowMappingXML.class.getSimpleName();

    private final Document document;
    private final XPath xpath;

    /*
    // IntelliJ only
    private static final String JAXP_SCHEMA_LANGUAGE =
            "http://java.sun.com/xml/jaxp/properties/schemaLanguage";
    private static final String W3C_XML_SCHEMA =
            "http://www.w3.org/2001/XMLSchema";
     */

    public RecognitionWindowMappingXML(String pRobotActionFilename) throws ParserConfigurationException, SAXException, IOException {

    /*
    // IntelliJ only
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        dbFactory.setNamespaceAware(true);
        dbFactory.setValidating(true);
        dbFactory.setAttribute(JAXP_SCHEMA_LANGUAGE, W3C_XML_SCHEMA);

        //## ONLY works with a validating parser (DTD or schema),
        // which the IntelliJ parser is.
        dbFactory.setIgnoringElementContentWhitespace(true);
    // End IntelliJ only
    */

        // Android or IntelliJ
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        //## ONLY works with a validating parser (DTD or schema),
        // which the Android Studio parser is not.
        // dbFactory.setIgnoringElementContentWhitespace(true);
        //PY 8/17/2019 Android throws UnsupportedOperationException dbFactory.setXIncludeAware(true);
        // End Android or IntelliJ

        DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
        document = dBuilder.parse(new File(pRobotActionFilename));

        XPathFactory xpathFactory = XPathFactory.newInstance();
        xpath = xpathFactory.newXPath();
    }

    // Collect data (resolution, ROI, recognition window boundaries)
    // for one Autonomous OpMode from RobotAction.xml. May return null
    // if the OpMode does not contain the requested action.
    public RecognitionWindowMapping collectRecognitionWindowMapping(RobotConstants.OpMode pOpMode, String pAction) throws XPathExpressionException {
        RobotLogCommon.c(TAG, "Collecting recognition window data for Autonomous OpMode " + pOpMode + " and action " + pAction);
        return getRecognitionWindowMapping(pOpMode, pAction);
    }

    // Collect spike window data (resolution, ROI, spike windows, etc.)
    // for all Autonomous competition OpModes from RobotAction.xml.
    public EnumMap<RobotConstants.OpMode, RecognitionWindowMapping> collectRecognitionWindowMapping(String pAction) throws XPathExpressionException {
        EnumMap<RobotConstants.OpMode, RecognitionWindowMapping> recognitionWindowMapping =
                new EnumMap<>(RobotConstants.OpMode.class);

        // Get all OpModes but only process those with an OpModeType
        // of COMPETITION or AUTO_TEST.
        RecognitionWindowMapping recognitionWindowDataOneOpMode;
        RobotConstants.OpMode[] allOpModes =
                RobotConstants.OpMode.values();
        for (RobotConstants.OpMode oneOpMode : allOpModes) {
            if (oneOpMode.getOpModeType() == RobotConstants.OpMode.OpModeType.COMPETITION ||
                    oneOpMode.getOpModeType() == RobotConstants.OpMode.OpModeType.AUTO_TEST) {
                RobotLogCommon.c(TAG, "Collecting recognition window data for Autonomous OpMode " + oneOpMode + " and action " + pAction);
                recognitionWindowDataOneOpMode = getRecognitionWindowMapping(oneOpMode, pAction);
                if (recognitionWindowDataOneOpMode != null)
                    recognitionWindowMapping.put(oneOpMode, recognitionWindowDataOneOpMode);
            }
        }

        return recognitionWindowMapping;
    }

    // Find the requested opMode in the RobotAction.xml file.
    // Package and return all data associated with the
    // requested action under the OpMode.
    private RecognitionWindowMapping getRecognitionWindowMapping(RobotConstants.OpMode pOpMode,
                                                                 String pAction) throws XPathExpressionException {
        EnumMap<RobotConstants.RecognitionWindow, Pair<Rect, RobotConstants.ObjectLocation>> recognitionWindows =
                new EnumMap<>(RobotConstants.RecognitionWindow.class);

        // Use XPath to locate the desired OpMode and its child action element.
        String actionPath = "/RobotAction/OpMode[@id=" + "'" + pOpMode + "']" + "/actions/" + pAction;
        Node action_node = (Node) xpath.evaluate(actionPath, document, XPathConstants.NODE);
        if (action_node == null) {
            RobotLogCommon.d(TAG, "No path to " + pOpMode + "/" + pAction);
            return null;
        }

        RobotLogCommon.c(TAG, "Extracting data from RobotAction.xml for " + actionPath);

        // The next element in the XML is required: <image_parameters>
        Node image_node = action_node.getFirstChild();
        image_node = XMLUtils.getNextElement(image_node);
        if ((image_node == null) || !image_node.getNodeName().equals("image_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'image_parameters' not found");

        VisionParameters.ImageParameters imageParameters = ImageXML.parseImageParameters(image_node);

        // Point to the recognition element, e.g. <distance_recognition>.
        Node recognition_node = image_node.getNextSibling();
        recognition_node = XMLUtils.getNextElement(recognition_node);
        if ((recognition_node == null) || !recognition_node.getNodeName().endsWith("recognition"))
            throw new AutonomousRobotException(TAG, "Recognition element not found");

        // Drop down to the <recognition_path> element.
        Node path_node = recognition_node.getFirstChild();
        path_node = XMLUtils.getNextElement(path_node);
        if ((path_node == null) || !path_node.getNodeName().equals("recognition_path") || path_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'recognition_path' not found");

        // Point to the <recognition_window> element.
        Node window_node = path_node.getNextSibling();
        window_node = XMLUtils.getNextElement(window_node);
        if ((window_node == null) || !window_node.getNodeName().equals("recognition_window") || window_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'recognition_window' not found");

        // Drop down the <left> element.
        Node left_node = window_node.getFirstChild();
        left_node = XMLUtils.getNextElement(left_node);
        if ((left_node == null) || !left_node.getNodeName().equals("left"))
            throw new AutonomousRobotException(TAG, "Element 'recognition_window/left' not found");

        // Drop down and parse the children of the <left> element.
        Node left_width_node = left_node.getFirstChild();
        left_width_node = XMLUtils.getNextElement(left_width_node);
        if ((left_width_node == null) || !left_width_node.getNodeName().equals("width") || left_width_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'recognition_window/left/width' not found");

        String leftWidthText = left_width_node.getTextContent();
        int leftWidth;
        try {
            leftWidth = Integer.parseInt(leftWidthText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'recognition_window/left_/width'");
        }

        // Parse the <object_location> element.
        Node left_object_node = left_width_node.getNextSibling();
        left_object_node = XMLUtils.getNextElement(left_object_node);
        if ((left_object_node == null) || !left_object_node.getNodeName().equals("object_location") || left_object_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'recognition_window/left/object_location' not found");

        String leftObjectLocationText = left_object_node.getTextContent().toUpperCase();
        RobotConstants.ObjectLocation leftObjectLocation =
                RobotConstants.ObjectLocation.valueOf(leftObjectLocationText);

        recognitionWindows.put(RobotConstants.RecognitionWindow.LEFT, Pair.create(new Rect(0, 0, leftWidth, imageParameters.image_roi.height), leftObjectLocation));

        // Parse the <right> element.
        Node right_node = left_node.getNextSibling();
        right_node = XMLUtils.getNextElement(right_node);
        if ((right_node == null) || !right_node.getNodeName().equals("right"))
            throw new AutonomousRobotException(TAG, "Element 'recognition_window/right' not found");

        // Parse the <object_location> element.
        Node right_object_node = right_node.getFirstChild();
        right_object_node = XMLUtils.getNextElement(right_object_node);
        if ((right_object_node == null) || !right_object_node.getNodeName().equals("object_location") || right_object_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'recognition_window/right/object_location' not found");

        String rightObjectLocationText = right_object_node.getTextContent().toUpperCase();
        RobotConstants.ObjectLocation rightObjectLocation =
                RobotConstants.ObjectLocation.valueOf(rightObjectLocationText);

        // Note: the right window starts 1 pixel past the left element. The height of the right
        // window is the same as that of the left window.
        recognitionWindows.put(RobotConstants.RecognitionWindow.RIGHT, Pair.create(new Rect(leftWidth, 0, imageParameters.image_roi.width - leftWidth, imageParameters.image_roi.height), rightObjectLocation));

        // Parse the <window_npos> element.
        Node npos_node = right_node.getNextSibling();
        npos_node = XMLUtils.getNextElement(npos_node);
        if ((npos_node == null) || !npos_node.getNodeName().equals("window_npos"))
            throw new AutonomousRobotException(TAG, "Element 'recognition_window/window_npos' not found");

        // Drop down and parse the <object_location> element.
        Node npos_location_node = npos_node.getFirstChild();
        npos_location_node = XMLUtils.getNextElement(npos_location_node);
        if ((npos_location_node == null) || !npos_location_node.getNodeName().equals("object_location") || npos_location_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'recognition_window/window_npos/object_location' not found");

        String nposLocationText = npos_location_node.getTextContent().toUpperCase();
        RobotConstants.ObjectLocation nposLocation =
                RobotConstants.ObjectLocation.valueOf(nposLocationText);

        recognitionWindows.put(RobotConstants.RecognitionWindow.WINDOW_NPOS, Pair.create(new Rect(0, 0, 0, 0), nposLocation));

        return new RecognitionWindowMapping(imageParameters, recognitionWindows);
    }

}