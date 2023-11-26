public class Stage{
    //////////////////////////////
    // PROPERTIES
    //////////////////////////////

    private Thing backdropRed;
    private Thing backdropBlue;
    private Position artHashLeft;
    private Position artHashCenter;
    private Position artHashRight;
    private Position home;

    ///////////////////////////////
    // CONSTRUCTOR(S)
    ///////////////////////////////
    public Stage(){
        home = new Position(10,12);
        artHashLeft = new Position(23,7);
    }

    Position getHome(){
        return home;
    }

    Position getArtHashLeft(){
        return artHashLeft;
    }




}