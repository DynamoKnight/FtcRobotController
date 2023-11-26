public class Thing extends Position {
    //////////////////////////////
    // PROPERTIES
    //////////////////////////////

    ///////////////////////////////
    // CONSTRUCTOR(S)
    ///////////////////////////////
    public Thing() {
        super();
    }
    public Thing(Position p){
        super(p.getX(),p.getY());
    }
    public Thing(int x, int y) {
        super(x, y);
        System.out.println(this);
    }

    ////////////////////////////////
    // METHODS
    ////////////////////////////////
    public Distance getDistance(Thing target) {
        Distance d = new Distance(target.getX() - getX(), target.getY() - getY());
        return d;
    }

}