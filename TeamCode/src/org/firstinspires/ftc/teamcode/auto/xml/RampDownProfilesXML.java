package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.platform.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.xml.XMLUtils;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.*;
import java.io.File;
import java.io.IOException;
import java.util.EnumMap;

public class RampDownProfilesXML {

    public static final String TAG = RampDownProfilesXML.class.getSimpleName();

    private static final String RAMPDOWN_FILE_NAME = "RampDownProfiles.xml";

    private final EnumMap<RampDownProfiles.MovementKey, RampDownProfiles.RampDownProfile> xmlRampDownProfiles = new EnumMap<>(RampDownProfiles.MovementKey.class);
    private final RampDownProfiles rampDownProfiles;

    public RampDownProfilesXML(String pXMLDir) {
        Node rampdown_profiles_node;
        try {
            String xmlFilePath = pXMLDir + RAMPDOWN_FILE_NAME;
            RobotLogCommon.c(TAG, "Parsing " + RAMPDOWN_FILE_NAME);

            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            dbFactory.setIgnoringComments(true);

            // ONLY works with a validating parser (DTD or schema)
            // dbFactory.setIgnoringElementContentWhitespace(true);
            // Not supported in Android Studio dbFactory.setXIncludeAware(true);

            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            Document document = dBuilder.parse(new File(xmlFilePath));
            XPathFactory xpathFactory = XPathFactory.newInstance();
            XPath xpath = xpathFactory.newXPath();

            // Point to the first node.
            XPathExpression expr = xpath.compile("//ramp_down_profiles");
            rampdown_profiles_node = (Node) expr.evaluate(document, XPathConstants.NODE);
            if (rampdown_profiles_node == null)
                throw new AutonomousRobotException(TAG, "Element '//rampdown_profiles' not found");

        } catch (ParserConfigurationException pex) {
            throw new AutonomousRobotException(TAG, "DOM parser Exception " + pex.getMessage());
        } catch (SAXException sx) {
            throw new AutonomousRobotException(TAG, "SAX Exception " + sx.getMessage());
        } catch (IOException iex) {
            throw new AutonomousRobotException(TAG, "IOException " + iex.getMessage());
        } catch (XPathExpressionException xex) {
            throw new AutonomousRobotException(TAG, "XPath Exception " + xex.getMessage());
        }

        // Now iterate through the <profile> elements.
        NodeList profiles = rampdown_profiles_node.getChildNodes();
        Node profile_node;
        for (int i = 0; i < profiles.getLength(); i++) {
            profile_node = profiles.item(i);
            if (profile_node.getNodeType() != Node.ELEMENT_NODE)
                continue;

            parseProfile(profile_node);
        }

        rampDownProfiles = new RampDownProfiles(xmlRampDownProfiles);
    }

    public RampDownProfiles getRampDownProfiles() {
        return rampDownProfiles;
    }

    // Parse the children of a <profile> element.
    // Create an instance of the RampProfile class and store
    // it in an EnumMap; the movement_key is the identifier
    // of the profile.
    private void parseProfile(Node pProfileNode) {
        // Point to <movement_key>.
        Node key_node = pProfileNode.getFirstChild();
        key_node = XMLUtils.getNextElement(key_node);
        if ((key_node == null) || !key_node.getNodeName().equals("movement_key") || key_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Missing required movement_key element");

        String keyString = key_node.getTextContent().trim().toUpperCase();
        RampDownProfiles.MovementKey movementKey = RampDownProfiles.MovementKey.valueOf(keyString);

        // Point to <direction>.
        Node direction_node = key_node.getNextSibling();
        direction_node = XMLUtils.getNextElement(direction_node);
        if ((direction_node == null) || !direction_node.getNodeName().equals("direction") || direction_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Missing required direction element");

        String directionString = direction_node.getTextContent().trim().toUpperCase();
        RampDownProfiles.RampDownProfile.RampDownDirection direction = RampDownProfiles.RampDownProfile.RampDownDirection.valueOf(directionString);

        // Point to <initial_velocity>.
        Node initial_velocity_node = direction_node.getNextSibling();
        initial_velocity_node = XMLUtils.getNextElement(initial_velocity_node);
        if ((initial_velocity_node == null) || !initial_velocity_node.getNodeName().equals("initial_velocity") || initial_velocity_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Missing required initial_velocity element");

        String ivText = initial_velocity_node.getTextContent();
        double initialVelocity;
        try {
            initialVelocity = Double.parseDouble(ivText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'initial_velocity'");
        }

        // Point to <final_velocity>.
        Node final_velocity_node = initial_velocity_node.getNextSibling();
        final_velocity_node = XMLUtils.getNextElement(final_velocity_node);
        if ((final_velocity_node == null) || !final_velocity_node.getNodeName().equals("final_velocity") || final_velocity_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Missing required final_velocity element");

        String fvText = final_velocity_node.getTextContent();
        double finalVelocity;
        try {
            finalVelocity = Double.parseDouble(fvText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'final_velocity'");
        }

        // Point to <rampdown_factor>.
        // This actually is the fraction of the click count at which the rampdown starts.
        Node factor_node = final_velocity_node.getNextSibling();
        factor_node = XMLUtils.getNextElement(factor_node);
        if ((factor_node == null) || !factor_node.getNodeName().equals("rampdown_factor") || factor_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Missing required rampdown_factor element");

        String factorText = factor_node.getTextContent();
        double rampdownFactor;
        try {
            rampdownFactor = Double.parseDouble(factorText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'rampdown_factor'");
        }

        RampDownProfiles.RampDownProfile oneProfile =
                new RampDownProfiles.RampDownProfile(direction, initialVelocity, finalVelocity, rampdownFactor);
        xmlRampDownProfiles.put(movementKey, oneProfile);
    }
}
        