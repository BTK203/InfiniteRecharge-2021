package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;

/** if our code is colonial America, this class is Rhode Island */
public class Util {

	/**
	 * Trims int within a range
	 * @param input int before truncation
	 * @param lowLimit lowest allowable int
	 * @param highLimit highest allowable int
	 * @return truncated int
	 */
	public static int truncateInt(int input, int lowLimit, int highLimit) {
		if (input < lowLimit) { input = lowLimit; }
		if (input > highLimit) { input = highLimit; }
		return input;
	}

	/**
	 * Converts celsius temperature to fahrenheit temperature
	 * @param celsius temperature in celsius degrees
	 * @return temperature in fahrenheit degrees
	 */
	public static double celsiusToFahrenheit(double celsius) {
		return (celsius * (9/5)) + 32;
	}

	/**
	 *	Kind of self explanatory, but with some spice
	 *	Use this mainly as a get method to retrieve values the user types into the smart dash
	 *		(the 'Set' part is only in case the value doesn't exist, backup is a default to use and set if it isn't there)
	 */
	static Preferences pref = Preferences.getInstance();
	public static double getAndSetDouble(String key, double backup) {
		if(!pref.containsKey(key)) pref.putDouble(key, backup);
		return pref.getDouble(key, backup);
	}

	/**
	 * Gets boolean value from Prefs, or sets it to backup if it doesn't exist
	 * @param key    The name of the bool to grab
	 * @param backup The backup value to use if the bool doesn't exist
	 * @return       The value of the boolean "Key"
	 */
	public static boolean getAndSetBoolean(String key, boolean backup) {
		if(!pref.containsKey(key)) pref.putBoolean(key, backup);
		return pref.getBoolean(key, backup);
	}

	/**
     * Really stupid but needed to round a double to n places
     * @param value  original value
     * @param places how many values after decimal point
     * @return       rounded value
     */
    public static double roundTo(double value, int places) {
        double val = value;
        val *= Math.pow(10, places);
        val = Math.round(val);
        val /= Math.pow(10, places);
        return val;
	}
	
	/**
	 * Checks if a controller exists.
	 * @param index The index of the controller (same you would use to define the Joystick)
	 * @return      True if the controller exists, false otherwise.
	 */
	public static boolean controllerExists(int index) {
		return DriverStation.getInstance().getJoystickName(index) != "";
	}

	public static double closestToZero(double[] set) {
		double least = Double.MAX_VALUE;
		for(int i=0; i<set.length; i++) {
			if(Math.abs(set[i]) < Math.abs(least)) {
				least = set[i];
			}
		}

		return least;
	}

	/**
	 * Gets the angle that the robot needs to turn through to acheive a heading.
	 * @param angle   The angle of the robot
	 * @param heading The desired heading to be acheived.
	 * @return        The angle that the robot needs to turn to have its desired heading.
	 */
	public static double getAngleToHeading(double angle, double heading) {
		double angle1 = heading - angle;         //angle to heading without crossing 0
		double angle2 = angle1 - 360;
		double angle3 = angle1 + 360;

		return closestToZero(new double[] {angle1, angle2, angle3});
	}

	/**
	 * Tests the equality of two values, then prints and returns the result.
	 * Print string will be displayed on RioLog as an error reading: "Assertion [assertionID] SUCCEEDED/FAILED."
	 * @param assertionName The informational name of the assertion. Will be used in the printout.
	 * @param item1       The first item to test.
	 * @param item2       The second item to test.
	 * @return            True if item1 equals item2. False otherwise.
	 */
	public static boolean assertEquals(String assertionName, Object item1, Object item2) {
		boolean success = item1.equals(item2);
		String message = "Assertion " + assertionName + " " + (success ? "SUCCEEDED" : "FAILED") + "." + " Item 1: " + item1.toString() + " | Item 2: " + item2.toString();
		DriverStation.reportError(message, false);
		return success;
	}
}