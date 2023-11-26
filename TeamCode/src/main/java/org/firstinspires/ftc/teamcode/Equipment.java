public class Equipment {

    //////////////////////////////
    // PROPERTIES
    //////////////////////////////
    private Robot robot;

    ///////////////////////////////
    // CONSTRUCTOR(S)
    ///////////////////////////////
    public Equipment() {

    }

    public Equipment(Robot robot) {
        this.robot = robot;
    }

    ////////////////////////////////
    // METHODS
    ////////////////////////////////

    public void setRobot(Robot robot) {
        this.robot = robot;
    }

    public Robot getRobot(){
        return this.robot;
    }
}
