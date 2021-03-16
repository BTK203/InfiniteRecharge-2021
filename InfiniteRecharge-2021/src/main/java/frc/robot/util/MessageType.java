package frc.robot.util;

public enum MessageType {
    UNKNOWN("UNKNOWN", -1),      //MESSAGE FORMATS BY TYPE:
    POSITION("Pos", 0),          //Pos:[point]
    PATH("Path", 1),             //Path-[dir]:[path]
    DIRECTORY_REQUEST("Dir", 2), //Dir:[dir]
    LOAD("Load", 3),             //Load:[path]
    SAVE("Save", 4);             //save-[path]:[file]

    private final String code;
    private final int index;

    /**
     * Creates a MessageType.
     * @param code The code that represents the MessageType 
     * @param index The index of the MessageType.
     * @return A MessageType representing the code and index.
     */
    MessageType(String code, int index) {
        this.code = code;
        this.index = index;
    }

    /**
     * Gets the index of the MessageType.
     * @return Index of the MessageType. -1 is unknown.
     */
    public int getIndex() {
        return index;
    }

    /**
     * Gets the string code that represents the MessageType.
     * @return String code to use. Should be sent as the subject of all messages to and from robot.
     */
    public String getCode() {
        return code;
    }

    /**
     * Returns a MessageType based on the contents of the code.
     * @param code A MessageType code.
     * @return A MessageType corresponding to the value of the code.
     */
    public static MessageType fromString(String code) {
        for(MessageType type : MessageType.values()) {
            if(code.startsWith(type.getCode())) {
                return type;
            }
        }

        return UNKNOWN;
    }
}
