public class Motor extends Equipment{

    //////////////////////////////
    // PROPERTIES
    //////////////////////////////
    private Robot robot;

    ///////////////////////////////
    // CONSTRUCTOR(S)
    ///////////////////////////////
    Motor(){
        super();
    }

    ////////////////////////////////
    // METHODS
    ////////////////////////////////

    public void doXDrive(int s){
        System.out.println("...driving in X " + this + " on " + getRobot());
    }

    public void doYDrive(int s){
        System.out.println("...driving in Y:" + this + " on " + getRobot() );
    }

}