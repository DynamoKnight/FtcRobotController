public class Robot extends Thing{

    //////////////////////////////
    // PROPERTIES
    //////////////////////////////
    private Stage map;
    private Camera camera;
    private Motor motor;

    ///////////////////////////////
    // CONSTRUCTOR(S)
    ///////////////////////////////
    public Robot(){
        super();
    }

    public Robot(Stage map, Camera camera, Motor motor){
        super();
        this.map = map;
        this.camera = camera;
        this.motor = motor;
        super.setPos(map.getHome());
        System.out.println("Robot is "+this);
        camera.setRobot(this);
        motor.setRobot(this);

    }

    public Robot(int x, int y){
        super(x,y);
    }
    ////////////////////////////////
    // METHODS
    ////////////////////////////////
    public void drive(Distance distance){
        for(int x = 0; x < distance.getX(); x++){
            this.motor.doXDrive(distance.getX());
            this.addX();
            System.out.println("new X: " + this.getX());
        }
        for(int y = 0; y < distance.getY(); y++){
            this.motor.doYDrive(distance.getY());
            this.addY();
            System.out.println("new Y: " +this.getY());
        }
    }

    public Stage getMap(){
        return this.map;
    }

    public Camera getCamera(){
        return this.camera;
    }
}