/*****************************************************************************
 ** ANGRYBIRDS AI AGENT FRAMEWORK
 ** Copyright (c) 2014, XiaoYu (Gary) Ge, Stephen Gould, Jochen Renz
 **  Sahan Abeyasinghe,Jim Keys,  Andrew Wang, Peng Zhang
 ** All rights reserved.
 **This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License. 
 **To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/3.0/ 
 *or send a letter to Creative Commons, 444 Castro Street, Suite 900, Mountain View, California, 94041, USA.
 *****************************************************************************/
package ab.demo;

import java.awt.Point;
import java.awt.Rectangle;
import java.awt.image.BufferedImage;
import java.util.*;

import ab.demo.other.ActionRobot;
import ab.demo.other.Shot;
import ab.planner.TrajectoryPlanner;
import ab.utils.ABUtil;
import ab.utils.StateUtil;
import ab.vision.*;
import ab.vision.GameStateExtractor.GameState;

public class NaiveAgent implements Runnable {

    private ActionRobot aRobot;
    private Random randomGenerator;
    public int currentLevel = 20;
    public static int time_limit = 12;
    private Map<Integer,Integer> scores = new LinkedHashMap<Integer,Integer>();
    TrajectoryPlanner tp;
    private boolean firstShot;
    private Point prevTarget;
    // a standalone implementation of the Naive Agent
    public NaiveAgent() {

        aRobot = new ActionRobot();
        tp = new TrajectoryPlanner();
        prevTarget = null;
        firstShot = true;
        randomGenerator = new Random();
        // --- go to the Poached Eggs episode level selection page ---
        ActionRobot.GoFromMainMenuToLevelSelection();

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
                firstShot = true;
            } else if (state == GameState.LOST) {
                firstShot = true;
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




        // confirm the slingshot
        while (sling == null && aRobot.getState() == GameState.PLAYING) {
            System.out
                    .println("No slingshot detected. Please remove pop up or zoom out");
            ActionRobot.fullyZoomOut();
            screenshot = ActionRobot.doScreenShot();
            vision = new Vision(screenshot);
            sling = vision.findSlingshotMBR();
        }
        System.out.println("\nPrinting Hills\n");
        List<ABObject> hills = vision.findHills();
        for(ABObject o: hills){
            System.out.println("id: "+o.id+" type: "+o.type+" height: "+o.height+" width: "+o.width+" shape: "+o.shape);
        }



        // get all the pigs
        List<ABObject> pigs = vision.findPigsMBR();
        System.out.println("---------Printing Pigs---------\n\n\n");
        for(ABObject o: pigs){
            System.out.println(o.getLocation());
        }
        System.out.println("\n\n\n");
        List<ABObject> blocks = vision.findBlocksMBR();
        System.out.println("Printing MBR blocks info\n\n\n");
        for(ABObject o: blocks){
            System.out.println("id :"+o.id+" height: "+o.height+" width: "+o.width+" angle: "+o.angle+ " type: "+o.type+" shape: "+o.shape);
        }

        //printList(blocks);
        GameState state = aRobot.getState();
        boolean reachability = false;//edited
        boolean reachability0 = false;
        boolean reachability1 = false;
        boolean targets_empty = true;
        // if there is a sling, then play, otherwise just skip.
        if (sling != null) {

            if (!pigs.isEmpty()) {
                //edited
                Point _tpt = new Point();
                if(!firstShot) {
                    for (ABObject o : pigs) {
                        Point releasePoint = null;
                        Shot shot = new Shot();
                        int dx, dy;
                        _tpt = o.getCenter();
                        ArrayList<Point> pts = tp.estimateLaunchPoint(sling, _tpt);
                        if (!pts.isEmpty()) {
                            releasePoint = pts.get(0);
                            Point refPoint = tp.getReferencePoint(sling);
                            dx = (int) releasePoint.getX() - refPoint.x;
                            dy = (int) releasePoint.getY() - refPoint.y;
                            shot = new Shot(refPoint.x, refPoint.y, dx, dy, 0, 0);
                            if (isReachable(vision, _tpt, shot)) {
                                System.out.println("\nSelected zero reachable Target type: " + o.type + " id: " + o.id + "\n");
                                reachability = true;
                                reachability0 = true;
                                break;
                            }
                            if (pts.size() > 1) {
                                releasePoint = pts.get(1);
                                refPoint = tp.getReferencePoint(sling);
                                dx = (int) releasePoint.getX() - refPoint.x;
                                dy = (int) releasePoint.getY() - refPoint.y;
                                shot = new Shot(refPoint.x, refPoint.y, dx, dy, 0, 0);
                                if (isReachable(vision, _tpt, shot)) {
                                    System.out.println("\nSelected one reachable Target type: " + o.type + " id: " + o.id + "\n");
                                    reachability = true;
                                    reachability1 = true;
                                    break;
                                }
                            }
                        }
                    }
                }
                //edited
                Shot shot = new Shot();
                Point releasePoint = null;
                int dx,dy;
                {
                    ABObject target_block = new ABObject();
                    if(!reachability){
                        //edited
                        //List<ABObject> blocks = vision.findBlocksMBR();
                        List<ABObject> wide_blocks = new LinkedList<ABObject>();
                        for(ABObject o :blocks){
                            float wide = o.height/o.width;
                            if(wide<0.25){
                                wide_blocks.add(o);
                            }
                        }
                        List<Integer> heuristic = gen_heuristic(wide_blocks, blocks, sling, vision);
                        List<target> targets = new LinkedList<target>();
                        if(!heuristic.isEmpty()){
                            for(int i = 0; i < wide_blocks.size(); i++){
                                target temp = new target();
                                temp.o = wide_blocks.get(i);
                                temp.h = heuristic.get(i);
                                targets.add(temp);
                            }

                        }
                        if(!targets.isEmpty()) {
                            Collections.sort(targets, new Comparator<target>() {

                                @Override
                                public int compare(target t1, target t2) {

                                    return ((Integer) (t1.h)).compareTo((Integer) (t2.h));
                                }
                            });
                        }
                            System.out.println("Printing targets\n\n\n");
                            for(target t: targets){
                                System.out.println("Id: "+t.o.id+" "+"type: "+t.o.type+"heuristic: "+t.h);
                            }
                            if(targets.get(0).h < -20) {
                                target_block = targets.get(0).o;
                                System.out.println("\nSelected target_block type: "+target_block.type+" id: "+target_block.id+"\n");
                                Point temp = target_block.getLocation();
                                _tpt.x = temp.x;
                                _tpt.y = temp.y + (target_block.height);
                                for(ABObject p: pigs){
                                    if(p.x >= _tpt.x){
                                        targets_empty = false;
                                        break;
                                    }
                                }
                            }
                        if(aRobot.getBirdTypeOnSling() == ABType.YellowBird && !firstShot){
                            targets_empty = true;
                        }
                        if(targets_empty){
                            //randomly pick a pig
                            int min_weight = Integer.MAX_VALUE;
                            for(ABObject pig: pigs){
                                ArrayList<Point> Points = tp.estimateLaunchPoint(sling, pig.getCenter());
                                if(Points.size()>0){
                                    int temp = getObstacleWeight(pig, vision, blocks, Points.get(0));
                                    if(min_weight >= temp){
                                        min_weight = temp;
                                        target_block = pig;
                                        reachability0 = true;
                                        reachability1 = false;
                                    }
                                }
                                if(Points.size()>1){
                                    int temp = getObstacleWeight(pig, vision, blocks, Points.get(1));
                                    if(min_weight >= temp){
                                        min_weight = temp;
                                        target_block = pig;
                                        reachability1 = true;
                                        reachability0 = false;
                                    }
                                }
                            }
                            System.out.println("\nSelected random target_block type: "+target_block.type+" id: "+target_block.id+"\n");
                            _tpt = target_block.getCenter();
                        }
                        //edited
                    }
                    if(firstShot){
                        List<ABObject> realBlocks = vision.findBlocksRealShape();
                        for(ABObject obj: realBlocks){
                            if((obj.shape == ABShape.Circle || obj.shape == ABShape.Poly) && (obj.type == ABType.Stone || obj.type == ABType.Wood)){
                                if(ABUtil.getSupporters(obj, blocks).size() == 0) {
                                    target_block = obj;
                                    _tpt = target_block.getCenter();
                                    System.out.println("\nRound target id: " + obj.id + " type: " + obj.type + " shape:" + obj.shape);
                                    break;
                                }
                            }
                        }
                    }


                    // if the target is very close to before, randomly choose a
                    // point near it
                    if (prevTarget != null && distance(prevTarget, _tpt) < 10) {
                        double _angle = randomGenerator.nextDouble() * Math.PI * 2;
                        _tpt.x = _tpt.x + (int) (Math.cos(_angle) * 10);
                        _tpt.y = _tpt.y + (int) (Math.sin(_angle) * 10);
                        System.out.println("Randomly changing to " + _tpt);
                    }

                    prevTarget = new Point(_tpt.x, _tpt.y);

                    // estimate the trajectory
                    ArrayList<Point> pts = tp.estimateLaunchPoint(sling, _tpt);
                    if(pts.size()>=1) {
                        if (reachability) {
                            if (reachability0) {
                                releasePoint = pts.get(0);
                            }
                            if (reachability1 && pts.size()>1) {
                                releasePoint = pts.get(1);
                            }
                        }
                        else if(targets_empty && !reachability){
                            if (reachability0) {
                                releasePoint = pts.get(0);
                            }
                            if (reachability1 && pts.size()>1) {
                                releasePoint = pts.get(1);
                            }
                        }
                        else if(!targets_empty && !reachability){
                            releasePoint = pts.get(0);
                        }
                    }

                    // do a high shot when entering a level to find an accurate velocity
                    /*if (firstShot && pts.size() > 1)
                    {
                        releasePoint = pts.get(0);
                    }
                    else if (pts.size() == 1)
                        releasePoint = pts.get(0);
                    else if (pts.size() == 2)
                    {
                        // randomly choose between the trajectories, with a 1 in
                        // 6 chance of choosing the high one
                        if (randomGenerator.nextInt(6) == 0)
                            releasePoint = pts.get(0);
                        else
                            releasePoint = pts.get(0);
                    }*/
                    else
                    if(pts.isEmpty())
                    {
                        System.out.println("No release point found for the target");
                        System.out.println("Try a shot with 45 degree");
                        releasePoint = tp.findReleasePoint(sling, Math.PI/4);
                    }


                    // Get the reference point
                    Point refPoint = tp.getReferencePoint(sling);


                    //Calculate the tapping time according the bird type
                    if (releasePoint != null) {
                        double releaseAngle = tp.getReleaseAngle(sling,
                                releasePoint);
                        System.out.println("Release Point: " + releasePoint);
                        System.out.println("Release Angle: "
                                + Math.toDegrees(releaseAngle));
                        int tapInterval = 0;
                        switch (aRobot.getBirdTypeOnSling())
                        {

                            case RedBird:
                                tapInterval = 0; break;               // start of trajectory
                            case YellowBird:
                                tapInterval = 75 + randomGenerator.nextInt(10);break; // 65-90% of the way
                            case WhiteBird:
                                tapInterval =  70 + randomGenerator.nextInt(20);break; // 70-90% of the way
                            case BlackBird:
                                tapInterval =  70 + randomGenerator.nextInt(20);break; // 70-90% of the way
                            case BlueBird:
                                tapInterval =  65 + randomGenerator.nextInt(15);break; // 65-85% of the way
                            default:
                                tapInterval =  60;
                        }

                        int tapTime = tp.getTapTime(sling, releasePoint, _tpt, tapInterval);
                        dx = (int)releasePoint.getX() - refPoint.x;
                        dy = (int)releasePoint.getY() - refPoint.y;
                        shot = new Shot(refPoint.x, refPoint.y, dx, dy, 0, tapTime);
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
                                    firstShot = false;
                                }
                            }
                        }
                        else
                            System.out.println("Scale is changed, can not execute the shot, will re-segement the image");
                    }
                    else
                        System.out.println("no sling detected, can not execute the shot, will re-segement the image");
                }

            }

        }
        return state;
    }

    List<Integer> gen_heuristic(List<ABObject> wide_blocks, List<ABObject> blocks, Rectangle sling, Vision vision ){
        if(!wide_blocks.isEmpty() && !blocks.isEmpty()){
            List<Integer> heuristic = new LinkedList<Integer>();
            int h = 0;
            for(ABObject o: wide_blocks ){
                h -= o.width;
                h -= (getParentWeight(o, blocks)*4)/100;
                if(o.type == ABType.Stone){
                    h +=(o.width + o.height);
                }
                TrajectoryPlanner tp1 = new TrajectoryPlanner();
                ArrayList<Point> Points = tp.estimateLaunchPoint(sling, o.getLocation());
                if(!Points.isEmpty()){
                    h += (getObstacleWeight(o, vision, blocks, Points.get(0))*7)/100;
                }
                else{
                    h = 9999;
                }
                int supporter_width = 0;

                List<ABObject> supporters = ABUtil.getSupporters(o, blocks);
                List<ABObject> supported = getSupported(o, blocks);
                if(!supporters.isEmpty()){
                    for(ABObject o1: supporters){
                        h += o1.width;
                        supporter_width += o1.width;
                    }
                    h += (supporter_width/supporters.size())/2;
                }
                else{
                    h = 9999;
                }
                if(!supported.isEmpty()){
                    for(ABObject o1: supported){
                        h -= o1.width;
                    }
                }
                heuristic.add(h);
                h = 0;
            }
            return heuristic;
        }
        return null;
    }

    List<ABObject> getSupported(ABObject o, List<ABObject> blocks){
        if(!blocks.isEmpty()){
            List<ABObject> supported = new LinkedList<ABObject>();
            for(ABObject o2: blocks){
                if(ABUtil.isSupport(o2, o)){
                    supported.add(o2);
                }
            }
            return supported;
        }
        return null;
    }

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
                    if (o.type == ABType.Pig)
                        weight = weight + o.width * o.height * 10;
                    else if (o.type == ABType.Stone)
                        weight = weight + o.width * 1;
                    else if (o.type == ABType.Ice)
                        weight = weight + o.width * 1;
                    else
                        weight = weight +  o.width *1;
                    q.add(o);
                    closed.add(o);
                }
            }
        }
        return weight;
    }

    private int getWeight (ABObject block) {
        if(block.type == ABType.Pig)
            return block.width * 10 + block.height;
        else if(block.type == ABType.Stone)
            return block.area;
        else if(block.type == ABType.Wood)
            return block.width * 1 + block.height;
        else if(block.type == ABType.Hill)
            return 9999;
        else
            return (block.width + block.height);
    }

    private int getObstacleWeight(ABObject block, Vision vision, List<ABObject> blocks, Point releasePoint ){
        int weight = 0;
        List<ABObject> hills = vision.findHills();
        for(ABObject hill: hills){
            blocks.add(hill);
        }

        Rectangle sling =  vision.findSlingshotMBR();

        Point _tpt = block.getCenter();
        ArrayList<ABObject> release = new ArrayList<ABObject>();

        List<Point> points = tp.predictTrajectory(sling, releasePoint);
        for (Point point : points) {
            if (point.x < 840 && point.y < 480 && point.y > 100 && point.x > 400)
                for (ABObject ab : blocks) {
                    if (
                            ((ab.contains(point) && !ab.contains(_tpt)) || Math.abs(vision.getMBRVision()._scene[point.y][point.x] - 72) < 10)
                                    && point.x < _tpt.x
                            ) {


                        boolean enter = true;
                        if (release.isEmpty()) {
                            release.add(ab);
                        } else {
                            for (ABObject obj : release) {
                                if (obj.id == ab.id) {
                                    enter = false;
                                }
                            }
                        }
                        if (enter) {
                            release.add(ab);
                            weight = weight + getWeight(ab);
                        }
                    }


                }

        }
        System.out.println("WEIGHT IS: " + weight);
        System.out.print("id: ");
        for(ABObject a: release) {
            System.out.print(" " + a.id + " ");
        }

        return weight;

    }

    //Return true if the target can be hit by releasing the bird at the specified release point
    public static boolean isReachable(Vision vision, Point target, Shot shot)
    {
        TrajectoryPlanner tp1 = new TrajectoryPlanner();
        //test whether the trajectory can pass the target without considering obstructions
        Point releasePoint = new Point(shot.getX() + shot.getDx(), shot.getY() + shot.getDy());
        int traY = tp1.getYCoordinate(vision.findSlingshotMBR(), releasePoint, target.x);
        if (Math.abs(traY - target.y) > 100)
        {
            //System.out.println(Math.abs(traY - target.y));
            return false;
        }
        boolean result = true;
        List<ABObject> obstacles = vision.findBlocksMBR();
        List<ABObject> hills = vision.findHills();
        for(ABObject hill: hills){
            obstacles.add(hill);
        }
        List<Point> points = tp1.predictTrajectory(vision.findSlingshotMBR(), releasePoint);
        for(Point point: points)
        {
            if(point.x < 840 && point.y < 480 && point.y > 100 && point.x > 400)
                for(ABObject ab: obstacles)
                {
                    if(
                            ((ab.contains(point) && !ab.contains(target))||Math.abs(vision.getMBRVision()._scene[point.y][point.x] - 72 ) < 10)
                                    && point.x < target.x
                            )
                        return false;
                }

        }
        return result;
    }

    public static void main(String args[]) {

        NaiveAgent na = new NaiveAgent();
        if (args.length > 0)
            na.currentLevel = Integer.parseInt(args[0]);
        na.run();

    }
}

class target{
    ABObject o;
    int h;
}

