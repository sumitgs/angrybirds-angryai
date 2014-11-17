/*****************************************************************************
** ANGRYBIRDS AI AGENT FRAMEWORK
        ** Copyright (c) 2014, XiaoYu (Gary) Ge, Stephen Gould, Jochen Renz
        **  Sahan Abeyasinghe,Jim Keys,  Andrew Wang, Peng Zhang
        ** All rights reserved.
        **This work is licensed under the terms of the GNU Affero General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
        **To view a copy of this license, visit http://www.gnu.org/licenses/
        *****************************************************************************/
        package ab.demo;

        import java.awt.Point;
        import java.awt.Rectangle;
        import java.awt.image.BufferedImage;
        import java.util.ArrayList;
        import java.util.LinkedHashMap;
        import java.util.List;
        import java.util.Map;
        import java.util.Random;
        import java.util.LinkedList;
        import java.util.Queue;
        import java.util.*;

        import ab.demo.other.ActionRobot;
        import ab.demo.other.Shot;
        import ab.planner.TrajectoryPlanner;
        import ab.utils.StateUtil;
        import ab.vision.ABObject;
        import ab.vision.ABShape;
        import ab.vision.ABType;
        import ab.vision.GameStateExtractor.GameState;
        import ab.vision.Vision;
        import ab.utils.ABUtil;

public class NaiveAgent implements Runnable {

    private ActionRobot aRobot;
    private Random randomGenerator;
    public int currentLevel = 1;
    public static int time_limit = 12;
    private Map<Integer,Integer> scores = new LinkedHashMap<Integer,Integer>();
    TrajectoryPlanner tp;
    //private boolean firstShot;
    private boolean firstshot = true;
    private boolean rolling = false;
    private Point prevTarget;
    private ABObject previousTarget = null;
    // a standalone implementation of the Naive Agent
    public NaiveAgent() {

        aRobot = new ActionRobot();
        tp = new TrajectoryPlanner();

        prevTarget = null;
        firstshot = true;
        randomGenerator = new Random();
        // --- go to the Poached Eggs episode level selection page ---
        ActionRobot.GoFromMainMenuToLevelSelection();

    }

    // information about the node in the list
    class blockInfo {
        int id;
        int weight;
        int parentWeight;
        ABType type;

        public blockInfo(int id, int weight, int parentWeight, ABType type) {
            this.id = id;
            this.weight = weight;
            this.parentWeight = parentWeight;
            this.type = type;
        }

    }

    private int getWeight (ABObject block) {
        if(block.type == ABType.Pig)
            return block.area * 10;
        else if(block.type == ABType.Stone)
            return block.area * 10;
        else if(block.type == ABType.Wood)
            return block.area * 2;
        else if(block.type == ABType.Hill)
            return 9999;
        else
            return block.area * 1;
    }
    // convert all blocks into linked list structure
    private List<blockInfo> convertToList(List<ABObject> blocks) {
        List<blockInfo> blk = new LinkedList<blockInfo>();

        for(ABObject o : blocks) {
            int id = o.id;
            int weight = getWeight(o);
            int parentWeight = getParentWeight(o, blocks);
            ABType type = o.type;

            blockInfo blkInf = new blockInfo(id, weight, parentWeight, type);

            blk.add(blkInf);
        }

        return blk;

    }

    private int getObstacleWeight(ABObject block, Vision vision, List<ABObject> blocks, Point releasePoint ){
        int weight0 = 0;

        Rectangle sling =  vision.findSlingshotMBR();

        Point _tpt = block.getCenter();

        weight0 = 0;
        ArrayList<ABObject> releaseZero = new ArrayList<ABObject>();

        List<Point> points = tp.predictTrajectory(sling, releasePoint);
        for (Point point : points) {
            if (point.x < 840 && point.y < 480 && point.y > 100 && point.x > 400)
                for (ABObject ab : blocks) {
                    if (
                            ((ab.contains(point) && !ab.contains(_tpt)) || Math.abs(vision.getMBRVision()._scene[point.y][point.x] - 72) < 10)
                                    && point.x < _tpt.x
                            ) {


                        boolean enter = true;
                        if (releaseZero.isEmpty()) {
                            releaseZero.add(ab);
                        } else {
                            for (ABObject obj : releaseZero) {
                                if (obj.id == ab.id) {
                                    enter = false;
                                }
                            }
                        }
                        if (enter) {
                            releaseZero.add(ab);
                            weight0 = weight0 + getWeight(ab);
                        }
                    }


                }

        }
        System.out.println("WEIGHT IS: " + weight0);
        System.out.print("id: ");
        for(ABObject a: releaseZero) {
            System.out.print(" " + a.id + " ");
        }

        return weight0;

    }

    private ABObject getFirstObstacle(ABObject block, Vision vision, List<ABObject> blocks, Point releasePoint ){

        Rectangle sling =  vision.findSlingshotMBR();

        Point _tpt = block.getCenter();

        List<Point> points = tp.predictTrajectory(sling, releasePoint);
        for (Point point : points) {
            if (point.x < 840 && point.y < 480 && point.y > 100 && point.x > 400)
                for (ABObject ab : blocks) {
                    if (
                            ((ab.contains(point) && !ab.contains(_tpt)) || Math.abs(vision.getMBRVision()._scene[point.y][point.x] - 72) < 10)
                                    && point.x < _tpt.x
                            ) {
                        System.out.println("id: " + ab.id + " type: " + ab.type + " shape: " + ab.shape);
                        return ab;
                    }
                }
        }
        return null;
    }

    private int numberOfPigsAbove(ABObject block, List<ABObject> blocks) {
        int i = 0;
        List<ABObject> closed = new LinkedList<ABObject>();
        Queue<ABObject> q = new LinkedList<ABObject>();

        q.offer(block);

        while(!q.isEmpty()) {

            ABObject obj = q.remove();

            List<ABObject> result = new LinkedList<ABObject>();

            for(ABObject o: blocks)
            {
                boolean add = true;
                if(ABUtil.isSupport(o,obj)) {
                    for(ABObject o2: closed){
                        if(o.id == o2.id){
                            add = false;
                        }
                    }
                    if(add){
                        result.add(o);
                    }

                }

            }

            if (!result.isEmpty()) {
                for (ABObject o : result) {
                    if (o.type == ABType.Pig)
                        i = i +1;
                    q.add(o);
                    closed.add(o);
                }
            }
        }
        return i;
    }

    // print list
    private void printList(List<ABObject> blocks) {
        System.out.println("**********printing the list**********");
        List<blockInfo> list = convertToList(blocks);

        for (blockInfo b : list) {
            System.out.println("id: " + b.id + " weight: " + b.weight + " parentWeight: " + b.parentWeight + " type: "+ b.type);
        }


    }

    private boolean isSupport(ABObject o2, ABObject o1)
    {
        int gap = 5;
        if(o2.x == o1.x && o2.y == o1.y && o2.width == o1.width && o2.height == o1.height)
            return false;

        int ex_o1 = o1.x + o1.width;
        int ex_o2 = o2.x + o2.width;

        int ey_o2 = o2.y + o2.height;
        if(
                (Math.abs(ey_o2 - o1.y) < gap)
                        &&
                        !( o2.x - ex_o1  > gap || o1.x - ex_o2 > gap )
                )
            return true;

        return false;
    }

    // to get list of object supported by o2
    private List<ABObject> getSupported(ABObject o2, List<ABObject> obj)
    {
        List<ABObject> result = new LinkedList<ABObject>();
        //Loop through the potential supporters
        for(ABObject o1: obj)
        {
            if(isSupport(o1,o2))
                result.add(o1);
        }
        return result;
    }

    // get sum of weight of all the nodes which are supported by block o1
    private int getParentWeight(ABObject o1, List<ABObject> blocks) {
        int weight = 0;
        List<ABObject> closed = new LinkedList<ABObject>();
        Queue<ABObject> q = new LinkedList<ABObject>();

        q.offer(o1);

        while(!q.isEmpty()) {

            ABObject obj = q.remove();

            List<ABObject> result = new LinkedList<ABObject>();

            for(ABObject o: blocks)
            {
                boolean add = true;
                if(ABUtil.isSupport(o,obj)) {
                    for(ABObject o2: closed){
                        if(o.id == o2.id){
                            add = false;
                        }
                    }
                    if(add){
                        result.add(o);
                    }

                }

            }

            if (!result.isEmpty()) {
                for (ABObject o : result) {
                    weight = weight + getWeight(o);
                    q.add(o);
                    closed.add(o);
                }
            }
        }
        return weight;
    }


    // run the client
    public void run() {

        aRobot.loadLevel(currentLevel);
        while (true) {
            GameState state = solve();
            if (state == GameState.WON) {
                try {
                    Thread.sleep(3000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                int score = StateUtil.getScore(ActionRobot.proxy);
                if(!scores.containsKey(currentLevel))
                    scores.put(currentLevel, score);
                else
                {
                    if(scores.get(currentLevel) < score)
                        scores.put(currentLevel, score);
                }
                int totalScore = 0;
                for(Integer key: scores.keySet()){

                    totalScore += scores.get(key);
                    System.out.println(" Level " + key
                            + " Score: " + scores.get(key) + " ");
                }
                System.out.println("Total Score: " + totalScore);
                aRobot.loadLevel(++currentLevel);

                // make a new trajectory planner whenever a new level is entered
                tp = new TrajectoryPlanner();

                // first shot on this level, try high shot first
                firstshot = true;
                rolling = false;
            } else if (state == GameState.LOST) {
                System.out.println("Restart");
                aRobot.restartLevel();
            } else if (state == GameState.LEVEL_SELECTION) {
                System.out
                        .println("Unexpected level selection page, go to the last current level : "
                                + currentLevel);
                aRobot.loadLevel(currentLevel);
            } else if (state == GameState.MAIN_MENU) {
                System.out
                        .println("Unexpected main menu page, go to the last current level : "
                                + currentLevel);
                ActionRobot.GoFromMainMenuToLevelSelection();
                aRobot.loadLevel(currentLevel);
            } else if (state == GameState.EPISODE_MENU) {
                System.out
                        .println("Unexpected episode menu page, go to the last current level : "
                                + currentLevel);
                ActionRobot.GoFromMainMenuToLevelSelection();
                aRobot.loadLevel(currentLevel);
            }

        }

    }



    private double distance(Point p1, Point p2) {
        return Math
                .sqrt((double) ((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y)
                        * (p1.y - p2.y)));
    }

    public GameState solve()
    {

        // capture Image
        BufferedImage screenshot = ActionRobot.doScreenShot();

        // process image
        Vision vision = new Vision(screenshot);

        // find the slingshot
        Rectangle sling = vision.findSlingshotMBR();
        List<ABObject> pigs = vision.findPigsRealShape();

        List<ABObject> blocks = vision.findBlocksRealShape();

        // confirm the slingshot
        while (sling == null && aRobot.getState() == GameState.PLAYING) {
            System.out
                    .println("No slingshot detected. Please remove pop up or zoom out");
            ActionRobot.fullyZoomOut();
            screenshot = ActionRobot.doScreenShot();
            vision = new Vision(screenshot);
            sling = vision.findSlingshotMBR();
        }
        // get all the pigs

        for (ABObject p : pigs) {
            blocks.add(p);
        }

        List<ABObject> hills = vision.findHills();
        for (ABObject h : hills) {
            blocks.add(h);
        }
        ABObject t = null;

        System.out.println("\n\n\n\n");



        GameState state = aRobot.getState();

        // if there is a sling, then play, otherwise just skip.
        if (sling != null) {

            /*for (ABObject p : blocks) {
                System.out.println( "area: " + p.area + " angle: " +  p.angle + " shape: " + p.shape);
            }*/

            /*printList(blocks);*/

            System.out.println("printlist is running");

            if (!pigs.isEmpty()) {

                Point releasePoint = null;
                Shot shot = new Shot();

                int dx,dy;
                {
                    // random pick up a pig
                    ABObject pig = pigs.get(randomGenerator.nextInt(pigs.size()));

                    t = getTarget(blocks, pigs, vision);
                    previousTarget = t;
                    Point _tpt = null;
                    System.out.println("target: " + t.type + " shape: " + t.shape);
                    //if(t.type == ABType.Pig || (t.type == ABType.Stone && (t.shape == ABShape.Circle || t.shape == ABShape.Poly)  )) {
                    if(t.type == ABType.Pig || t.shape == ABShape.Circle) {
                        System.out.println("I AM IN THE PIG AND ROUND STONE SECTION");
                        System.out.println();
                        System.out.println("original target point: ");
                        System.out.println();
                        _tpt = t.getCenter();
                    }
                    else {
                        System.out.println("I AM IN THE WIDE BLOCK SELECTION CENTER");
                        System.out.println();
                        System.out.println("original target point: ");
                        System.out.println();
                        ABObject temp = t;
                        System.out.println("Actual Location: " + t.getLocation());
                        temp.setLocation((int) t.getX(), (int) t.getY() + t.height/2 + t.height/3);
                        System.out.println("setted Location: " + temp.getLocation());
                        System.out.println("Height: " + t.height);

                        _tpt = temp.getLocation();
                    }
                    // if the target is very close to before, randomly choose a
                    // point near it
                    if (prevTarget != null && distance(prevTarget, _tpt) < 10) {
                        System.out.println("randomly choosing the point");
                        double _angle = randomGenerator.nextDouble() * Math.PI * 2;
                        _tpt.x = _tpt.x + (int) (Math.cos(_angle) * 10);
                        _tpt.y = _tpt.y + (int) (Math.sin(_angle) * 10);
                        System.out.println("Randomly changing to " + _tpt);
                    }

                    prevTarget = new Point(_tpt.x, _tpt.y);

                    // estimate the trajectory
                    ArrayList<Point> pts = tp.estimateLaunchPoint(sling, _tpt);


                                                                                            /* for the round stone */


                    if(t.shape == ABShape.Circle || t.shape == ABShape.Poly) {
                        int i=0;
                        int j=0;
                        boolean fromAbove = false;
                        System.out.println("i am here 1");

                        for(ABObject p : pigs) {
                            if(p.getX() >= t.getX()) {
                                i = i+1;
                            }
                            else {
                                j = j+1;
                            }
                        }

                        System.out.println(" type: " + t.type + " shape:  " + t.shape);

                        int enter1 = getObstacleWeight(t, vision, blocks, pts.get(1));
                        System.out.println("enter1: " + enter1);

                        int enter2 = getObstacleWeight(t, vision, blocks, pts.get(0));
                        System.out.println("enter2: " + enter2);

                        if(i > 0 && j > 0 && pts.size() > 1 && enter1 == 0) {

                            System.out.println("round stone selecting the upper trajectory: ");

                            ABObject temp = t;
                            System.out.println("Actual Location: " + t.getLocation());
                            temp.setLocation((int) t.getX() - 3, (int) t.getY());
                            System.out.println("setted Location: " + temp.getLocation());
                            System.out.println("Height: " + t.height);

                            _tpt = temp.getLocation();

                            pts = tp.estimateLaunchPoint(sling, _tpt);

                            releasePoint = pts.get(1);

                        }
                        else if ( enter2 == 0) {
                            System.out.println("round stone selecting lower trajectory");

                            ABObject temp = t;
                            System.out.println("Actual Location: " + t.getLocation());
                            temp.setLocation((int) t.getX(), (int) t.getY() - t.height/3);
                            System.out.println("set Location: " + temp.getLocation());
                            System.out.println("Height: " + t.height);

                            _tpt = temp.getLocation();

                            pts = tp.estimateLaunchPoint(sling, _tpt);

                            releasePoint = pts.get(0);

                        }
                        else {
                            System.out.println("executing the support for round stone");

                            List<ABObject> blks = ABUtil.getSupporters(t, blocks);

                            System.out.println("size of the supporting blocks: " + blks.size());

                            for(ABObject b : blks) {
                                _tpt = b.getLocation();

                                pts = tp.estimateLaunchPoint(sling, _tpt);

                                releasePoint = pts.get(0);

                                int reachable = getObstacleWeight(b, vision, blocks, releasePoint);
                                System.out.println("round stone support: " + reachable);
                                if(reachable == 0) {
                                    t = b;
                                    _tpt = t.getLocation();
                                    pts = tp.estimateLaunchPoint(sling, _tpt);
                                    releasePoint = pts.get(0);
                                    break;
                                }
                            }

                        }
                    }

                                                                                        /* for the pig */


                    else if(t.type == ABType.Pig) {



                       /* List<ABObject> blocks1 = blocks;

                        for(ABObject hill : hills) {
                            blocks1.add(hill);
                        }*/

                        boolean reachable = false;
                        if(!pts.isEmpty()) {

                            releasePoint = pts.get(0);
                            int weight = getObstacleWeight(t, vision, blocks, releasePoint);

                            if(weight ==  0){
                                System.out.println("pig select lower trajectory");
                                reachable = true;
                                releasePoint = pts.get(0);
                            }
                        }
                        if(!reachable && pts.size() > 1) {

                            releasePoint = pts.get(1);
                            int weight = getObstacleWeight(t, vision, blocks, releasePoint);

                            if(weight ==  0){
                                System.out.println("pig select lower trajectory");
                                reachable = true;
                                releasePoint = pts.get(1);
                            }
                        }

                        if(!reachable) {
                            int i=Integer.MAX_VALUE;
                            int j = Integer.MAX_VALUE;
                            if(!pts.isEmpty()){
                                releasePoint = pts.get(0);
                                i = getObstacleWeight(t, vision, blocks,releasePoint);
                                ABObject first = getFirstObstacle(t, vision, blocks, releasePoint);
                                System.out.println("first object: " + first.id + " type: " + first.type + " shape: " + first.shape + " location: " + first.getCenter() + " angle: " + first.angle);

                                if(pts.size()>1){
                                    releasePoint = pts.get(1);
                                    j = getObstacleWeight(t, vision, blocks,releasePoint);

                                    ABObject second = getFirstObstacle(t, vision, blocks, releasePoint);
                                    System.out.println("second object: " + second.id + " type: " + second.type + " shape: " + second.shape + " location: " + second.getCenter() + " angle: " + second.angle);
                                }
                            }

                            if(i <= j) {
                                System.out.println("pig lower minimum obstacle and applying the support structure");

                                releasePoint = pts.get(0);

                                ABObject support = getFirstObstacle(t, vision, blocks, releasePoint);

                                if(support.height / support.width > 2) {
                                    System.out.println("support is selected");

                                    _tpt = support.getCenter();

                                    pts = tp.estimateLaunchPoint(sling, _tpt);

                                    releasePoint = pts.get(0);

                                }


                            }
                            else {
                                System.out.println("pig upper minimum obstacle");

                                releasePoint = pts.get(1);
                            }
                        }

                    }


                                                                                                /* for the wide block */

                    else {
                        System.out.println("for block selecting the lower trajectory");

                        releasePoint = pts.get(0);
                    }


                    // Get the reference point
                    Point refPoint = tp.getReferencePoint(sling);


                    //Calculate the tapping time according the bird type
                    if (releasePoint != null) {
                        double releaseAngle = tp.getReleaseAngle(sling,
                                releasePoint);
                        System.out.println("previous: " + previousTarget.id + " target: " + t.id);
                        System.out.println("previous Target: " + previousTarget.getCenter() + " target: " + t.getCenter());
                        System.out.println("Release Point so this is going to be new release point: " + releasePoint);
                        System.out.println("Release Angle and this is going to be new release angle: "
                                + Math.toDegrees(releaseAngle));
                        int tapInterval = 0;
                        switch (aRobot.getBirdTypeOnSling())
                        {

                            case RedBird:
                                tapInterval = 0; break;               // start of trajectory
                            case YellowBird:
                                tapInterval = 80 + randomGenerator.nextInt(10);break; // 65-90% of the way
                            case WhiteBird:
                                tapInterval =  70 + randomGenerator.nextInt(20);break; // 70-90% of the way
                            case BlackBird:
                                tapInterval =  70 + randomGenerator.nextInt(20);break; // 70-90% of the way
                            case BlueBird:
                                tapInterval = 75 + randomGenerator.nextInt(10);break;  // 65-80% of the way
                            default:
                                tapInterval =  60;
                        }

                        System.out.println("Tap interval: " + tapInterval);
                        int tapTime = tp.getTapTime(sling, releasePoint, _tpt, tapInterval);
                        dx = (int)releasePoint.getX() - refPoint.x;
                        dy = (int)releasePoint.getY() - refPoint.y;
                        shot = new Shot(refPoint.x, refPoint.y, dx, dy, 0, tapTime);

                        //  System.out.println("reahcability of shot" + ABUtil.isReachable(vision, pig.getCenter(), shot));
                    }
                    else
                    {
                        System.err.println("No Release Point Found");
                        return state;
                    }
                }

                // check whether the slingshot is changed. the change of the slingshot indicates a change in the scale.
                {
                    ActionRobot.fullyZoomOut();
                    screenshot = ActionRobot.doScreenShot();
                    vision = new Vision(screenshot);
                    Rectangle _sling = vision.findSlingshotMBR();
                    if(_sling != null)
                    {
                        double scale_diff = Math.pow((sling.width - _sling.width),2) +  Math.pow((sling.height - _sling.height),2);
                        if(scale_diff < 25)
                        {
                            if(dx < 0)
                            {
                                aRobot.cshoot(shot);
                                state = aRobot.getState();
                                if ( state == GameState.PLAYING )
                                {
                                    screenshot = ActionRobot.doScreenShot();
                                    vision = new Vision(screenshot);
                                    List<Point> traj = vision.findTrajPoints();
                                    tp.adjustTrajectory(traj, sling, releasePoint);
                                    firstshot = false;
                                }
                            }
                        }
                        else {
                            System.out.println("previous: " + previousTarget.id + " target: " + t.id);
                            System.out.println("Scale is changed, can not execute the shot, will re-segement the image");
                        }
                    }
                    else {
                        System.out.println("no sling detected, can not execute the shot, will re-segement the image");
                    }
                }

            }

        }
        return state;
    }

    private ABObject findBird(Vision vision) {
        List<ABObject> birds = vision.findBirdsMBR();

        ABObject activeBird = null;

        for (ABObject r : birds) {
            if ((activeBird == null) || (activeBird.y > r.y)) {
                activeBird = r;
            }
        }
        return activeBird;
    }
    private ABObject getTarget(List<ABObject> blocks, List<ABObject> pigs, Vision vision) {
        System.out.println("whether it is first shot: " + firstshot);
        int h=0;
        Point releasePoint = null;
        int dx, dy;

        Rectangle sling = vision.findSlingshotMBR();
        ABObject target = null;
        ABObject posTarget = null;
        if(firstshot) {
            System.out.println("enter into round stone");
            if(findBird(vision).type != ABType.YellowBird) {
                for (ABObject o : blocks) {
                    if(o.type == ABType.Stone || o.type == ABType.Wood) {
                        if (o.shape == ABShape.Circle) {
                            if (o.area >= h) {
                                rolling = true;
                                posTarget = o;
                                h = o.area;
                            }
                        }
                    }
                }

                /*if(posTarget != null) {
                    ArrayList<Point> pts = tp.estimateLaunchPoint(sling, posTarget.getCenter());
                    if (pts.size() > 0) {
                        releasePoint = pts.get(0);

                        if (getObstacleWeight(posTarget, vision, blocks, releasePoint) == 0) {
                            System.out.println("lower trajectory has zero weight");
                            target = posTarget;
                            return target;
                        }

                        if (pts.size() > 1) {
                            releasePoint = pts.get(1);

                            if (getObstacleWeight(posTarget, vision, blocks, releasePoint) == 0) {
                                System.out.println("upper trajectory has zero weight");
                                target = posTarget;
                                return target;
                            }

                        }
                    }
                }*/

                if (posTarget != null) {
                    System.out.println("selecting the round stone");

                    firstshot = false;
                    target = posTarget;
                    return target;
                }
            }

            System.out.println("rolling stone: " + rolling);

            System.out.println("enter hill: ");

            int counter = 0;
            for(ABObject pig : pigs) {
                int tapInterval = 0;
                Point refPoint = tp.getReferencePoint(sling);
                ArrayList<Point> pts = tp.estimateLaunchPoint(sling, pig.getCenter());
                releasePoint = pts.get(0);
                int tapTime = tp.getTapTime(sling, releasePoint, pig.getCenter(), tapInterval);
                dx = (int) releasePoint.getX() - refPoint.x;
                dy = (int) releasePoint.getY() - refPoint.y;
                Shot shot = new Shot(refPoint.x, refPoint.y, dx, dy, 0, tapTime);

                if (ABUtil.isReachable(vision, pig.getCenter(), shot))
                {
                    counter = counter + 1;
                }
                else {
                    counter = 100;
                    break;
                }

                System.out.println("i: " + counter);
                System.out.println("total pigs: " + pigs.size());

                if(counter == pigs.size()) {
                    System.out.println("slect hill: ");
                    target = pig;
                    return target;
                }


            }

            System.out.println("leaving hill");


            System.out.println("enter into wide blok mechanism");

            List<ABObject> w_blocks = new LinkedList<ABObject>();
            for(ABObject o :blocks ){
                if(o.type != ABType.Stone  && o.type != ABType.Hill) {
                    double wide = (double) o.height / o.width;
                    if (wide < 0.2) {
                        System.out.println("first ratio: " + wide);
                        w_blocks.add(o);
                    }
                }
            }

            System.out.println("fisrt w_blocks: " + w_blocks.size());

            List<ABObject> wide_blocks = w_blocks;

            System.out.println("fisrt wide_blocks: " + wide_blocks.size());

            //List<Integer> heuristic = gen_heuristic(wide_blocks, blocks);

            List<_target> targets = new LinkedList<_target>();

            for(int i = 0; i < wide_blocks.size(); i++){
                _target temp = new _target();
                temp.o = wide_blocks.get(i);
                temp.weight = getParentWeight(wide_blocks.get(i), blocks) / getWeight(wide_blocks.get(i));
                double d = Double.MAX_VALUE;
                for (ABObject p : pigs) {
                    if(distance(wide_blocks.get(i).getCenter() , p.getCenter()) < d) {
                        d = distance(wide_blocks.get(i).getCenter(), p.getCenter());
                    }
                }
                temp.distance = d;
                temp.pigs = numberOfPigsAbove(wide_blocks.get(i), blocks);
                targets.add(temp);
            }

            if(targets.size() > 1) {
                Collections.sort(targets, new weightComparator());

                Collections.sort(targets, new distanceComparator());

                Collections.sort(targets, new pigComparator());
            }


            for(_target t : targets) {
                System.out.println(" pigs: " + t.pigs + " distance: " + t.distance + " weight: " + t.weight + " ratio: "  + (double) t.o.height / t.o.width);
            }

            if(!targets.isEmpty()) {
                for (_target t : targets) {
                    if(t.pigs > 0 || rolling) {
                        ABObject o = t.o;

                        int tapInterval = 0;
                        Point refPoint = tp.getReferencePoint(sling);
                        ArrayList<Point> pts = tp.estimateLaunchPoint(sling, o.getLocation());
                        releasePoint = pts.get(0);
                        int tapTime = tp.getTapTime(sling, releasePoint, o.getLocation(), tapInterval);
                        dx = (int) releasePoint.getX() - refPoint.x;
                        dy = (int) releasePoint.getY() - refPoint.y;
                        Shot shot = new Shot(refPoint.x, refPoint.y, dx, dy, 0, tapTime);


                        if (ABUtil.isReachable(vision, o.getLocation(), shot) == true)
                        {
                            System.out.println("fisrt pigs: " + t.pigs + " disance: " + t.distance + " weight: " + t.weight);
                            target = o;
                            break;
                        }
                    }
                }
            }

            if(target != null) {
                System.out.println("selecting the fist shot wide block");
                firstshot = false;
                return target;
            }
            System.out.println("leaving first shot wide block");


        }

        else {
            for(ABObject o : pigs) {
                System.out.println("enter second shot reachable pig");

                Point _tpt = o.getCenter();

                ArrayList<Point> pts = tp.estimateLaunchPoint(sling, _tpt);
                if(!pts.isEmpty()){
                    releasePoint = pts.get(0);
                    int weight = getObstacleWeight(o, vision, blocks, releasePoint);

                    if(weight ==  0){
                        System.out.println("selected second shot reachable pig");

                        target = o;
                        return target;
                    }
                    if(pts.size()>1){
                        releasePoint = pts.get(1);

                        int weight1 = getObstacleWeight(o, vision, blocks, releasePoint);

                        if(weight1 == 0){
                            System.out.println("selected second shot reachable pig");

                            target = o;
                            return target;
                        }
                    }
                }
            }

            System.out.println("enter second shor wide block");


            List<ABObject> w_blocks = new LinkedList<ABObject>();
            for(ABObject o :blocks ){
                if(o.type != ABType.Stone  && o.type != ABType.Hill) {
                    double wide = (double) o.height / o.width;
                    if (wide < 0.2) {
                        System.out.println("SECOND ratio: " + wide);
                        w_blocks.add(o);
                    }
                }
            }

            System.out.println("second w_blocks: " + w_blocks.size());

            List<ABObject> wide_blocks = w_blocks;

            System.out.println("second wide_blocks: " + wide_blocks.size());

            //List<Integer> heuristic = gen_heuristic(wide_blocks, blocks);

            List<_target> targets = new LinkedList<_target>();

            for(int i = 0; i < wide_blocks.size(); i++){
                _target temp = new _target();
                temp.o = wide_blocks.get(i);
                temp.weight = getParentWeight(wide_blocks.get(i), blocks) / getWeight(wide_blocks.get(i));
                double d = Double.MAX_VALUE;
                for (ABObject p : pigs) {
                    if(distance(wide_blocks.get(i).getCenter() , p.getCenter()) < d) {
                        d = distance(wide_blocks.get(i).getCenter(), p.getCenter());
                    }
                }
                temp.distance = d;
                temp.pigs = numberOfPigsAbove(wide_blocks.get(i), blocks);
                targets.add(temp);
            }

            if(targets.size() > 1) {
                Collections.sort(targets, new weightComparator());

                Collections.sort(targets, new distanceComparator());

                Collections.sort(targets, new pigComparator());
            }


            for(_target t : targets) {
                System.out.println(" pigs: " + t.pigs + " distance: " + t.distance + " weight: " + t.weight);
            }

            if(!targets.isEmpty()) {
                for (_target t : targets) {
                    if(t.pigs > 0|| rolling) {
                        ABObject o = t.o;
                        int tapInterval = 0;
                        Point refPoint = tp.getReferencePoint(sling);

                        ArrayList<Point> pts = tp.estimateLaunchPoint(sling, o.getLocation());

                        if(pts.size() > 0) {
                            releasePoint = pts.get(0);

                            int tapTime = tp.getTapTime(sling, releasePoint, o.getLocation(), tapInterval);

                            dx = (int) releasePoint.getX() - refPoint.x;
                            dy = (int) releasePoint.getY() - refPoint.y;

                            Shot shot = new Shot(refPoint.x, refPoint.y, dx, dy, 0, tapTime);


                            if (ABUtil.isReachable(vision, o.getLocation(), shot) == true) {
                                System.out.println("11");
                                System.out.println("second pigs: " + t.pigs + " disance: " + t.distance + " weight: " + t.weight);
                                target = o;
                                break;
                            }
                        }
                    }
                }
            }

            if(target != null) {
                System.out.println("selected second shor wide block");

                return target;
            }

            System.out.println("leaving second shor wide block");




            System.out.println("select second shor random pig");
            int total = 0;
            int xz = Integer.MIN_VALUE;

            for(ABObject p : pigs){
                total = total + p.area;
            }

            ABObject pig = pigs.get(randomGenerator.nextInt(pigs.size()));

            if(pig.area * pigs.size() != total) {
                for (ABObject p : pigs) {
                    if (p.area > xz) {
                        xz = p.area;
                        target = p;
                    }
                }
                System.out.println("select biggets: " );
            }
            else {
                double xy = Double.MAX_VALUE;
                for (ABObject p : pigs) {
                    if (p.getX() < xy) {
                        xy = p.getX();
                        target = p;
                    }
                }
                System.out.println("select nearest: " );

            }

            return target;

        }
        System.out.println("null return pig");
        double xy = Double.MAX_VALUE;
        for(ABObject p: pigs) {
            if (p.getX() < xy )
            {
                xy = p.getX();
                target = p;
            }
        }

        return target;
    }


    class _target{
        ABObject o;
        double weight;
        double distance;
        int pigs;
    }

    class pigComparator implements Comparator<_target> {
        @Override
        public int compare(_target t1, _target t2) {
            double pig1 = t1.pigs;
            double pig2 = t2.pigs;

            if (pig1 > pig2) {
                return -1;
            } else if (pig1 < pig2) {
                return 1;
            } else {
                return 0;
            }
        }
    }

    class distanceComparator implements Comparator<_target> {
        @Override
        public int compare(_target t1, _target t2) {
            double distance1 = t1.distance;
            double distance2 = t2.distance;

            if (distance1 > distance2) {
                return 1;
            } else if (distance1 < distance2) {
                return -1;
            } else {
                return 0;
            }
        }
    }

    class weightComparator implements Comparator<_target> {
        @Override
        public int compare(_target t1, _target t2) {
            double weight1 = t1.weight;
            double weight2 = t2.weight;

            if (weight1 > weight2) {
                return -1;
            } else if (weight1 < weight2) {
                return 1;
            } else {
                return 0;
            }
        }
    }

    public static void main(String args[]) {

        NaiveAgent na = new NaiveAgent();
        if (args.length > 0)
            na.currentLevel = Integer.parseInt(args[0]);
        na.run();

    }
}
