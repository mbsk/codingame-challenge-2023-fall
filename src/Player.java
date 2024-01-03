import java.util.*;
import java.util.List;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

/**
 * Score points by scanning valuable fish faster than your opponent.
 **/
class Player {

    final static int KILL_DISTANCE = 500;
    final static int MAX_MOVE = 600;
    final static int DEPTH_SURFACE = 490;
    final static int LIGHT_0_RADIUS = 800;
    final static int LIGHT_1_RADIUS = 2000;
    final static int LIGHT_MONSTERS_EXTRA = 300;
    final static Point ORIGIN = new Point(0,0);

    final static int DIVE_LIGHT_ON_EVERY = 2;

    static Map<Integer, Creature> creatures = new HashMap<>();
    static Map<Integer, Drone> drones = new HashMap<>();
    static CommonOperationPicture cop = new CommonOperationPicture();

    static int turn = 0;
    static int myScore = 0;
    static int foeScore = 0;

    enum Depth {
        L0(-10,0), L1(0,2500),L2(1,5000),L3(2,7500), MONSTER(-1, 2500, 10000);
        final int type;
        final int start;
        int end;
        int middle;

        static final Map<Integer, Depth> BY_TYPE = new HashMap<>();

        static {
            for (Depth e: values()) {
                BY_TYPE.put(e.type, e);
            }
        }

        Depth(int type, int start) {
            this.type = type;
            this.start = start;
            this.end = start+2500;
            this.middle = start+1250;
        }

        Depth(int type, int start, int end) {
            this(type, start);
            this.end = 10000;
            this.middle = (this.end-this.start)/2;
        }

    }

    static class Point{
        long x;
        long y;

        public Point(long x, long y) {
            this.x = x;
            this.y = y;
        }

        double distanceTo(Point to) {
            return Math.sqrt(Math.pow(this.x-to.x,2)+Math.pow(this.y-to.y,2));
        }

        Point add(Point to) {
            return new Point(this.x + to.x, this.y + to.y);
        }

        Vector vectorTo(Point to) {
            return Vector.build(this, to);
        }

    }

    static class Vector {
        double x = 0;
        double y = 0;

        private Vector() {}

        static Vector build(Point from, Point to) {
            Vector v = new Vector();
            v.x = to.x - from.x;
            v.y = to.y - from.y;
            return v;
        }

        public void reset() {
            x = 0;
            y = 0;
        }

        public Vector rotate(double angle) {
            Vector v = new Vector();
            v.x = this.x * Math.cos(angle) - this.y * Math.sin(angle);
            v.y = this.x * Math.sin(angle) + this.y * Math.cos(angle);
            return v;
        }

        public Point normalizedDirection(int n) {
            double length = Math.sqrt(Math.pow(this.x,2)+Math.pow(this.y,2));
            long dx = Math.round(n * x/length);
            long dy = Math.round(n * y/length);
            return new Point(dx, dy);
        }

        public Vector add(Vector u) {
            Vector v = new Vector();
            v.x = this.x + u.x;
            v.y = this.y + u.y;
            return v;
        }

        public double size() {
            return Math.sqrt(Math.pow(this.x,2)+Math.pow(this.y,2));
        }

        public double angleTo(Vector v) {
            double num = this.x*v.x + this.y*v.y;
            double den = (Math.sqrt(Math.pow(v.x, 2) + Math.pow(v.y, 2)) * (Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.y, 2))) );
            return Math.abs(Math.acos(num / den) % Math.PI);
        }

    }

    static class CommonOperationPicture {
        Map<Integer, RadarPlot> radar = new HashMap<>();
        long[] xPositions;
        long[] yPositions;

        public void update() {
            Map<Integer, RadarPlot> _radar = new HashMap<>();

            // get only my drones
            List<Drone> myDrones = drones.values().stream().filter(d->d.mine).toList();

            // order by x
            List<Drone> hOrderedDrones = myDrones.stream().sorted(Comparator.comparing(d -> d.pos.x)).toList();
            long[] _xPositions = new long[hOrderedDrones.size()];
            for (int i = 0; i < hOrderedDrones.size(); i++) {
                final int order = i; // needed a variable "effectively" final to be used whithin streams
                hOrderedDrones.get(i).radars.entrySet().stream()
                        .filter(e->"BL".equals(e.getKey()) || "TL".equals(e.getKey()))
                        .map(Map.Entry::getValue)
                        .forEach(j->this.updatePlot(j, order, true, _radar));
                hOrderedDrones.get(i).radars.entrySet().stream()
                        .filter(e->"BR".equals(e.getKey()) || "TR".equals(e.getKey()))
                        .map(Map.Entry::getValue)
                        .forEach(j->this.updatePlot(j, order+1, true, _radar));
                _xPositions[i] = hOrderedDrones.get(i).pos.x;
            }


            // order by y
            List<Drone> vOrderedDrones = myDrones.stream().sorted(Comparator.comparing(d -> d.pos.y)).toList();
            long[] _yPositions = new long[vOrderedDrones.size()];
            for (int i = 0; i < vOrderedDrones.size(); i++) {
                final int order = i; // needed a variable "effectively" final to be used whithin streams
                vOrderedDrones.get(i).radars.entrySet().stream()
                        .filter(e->"TL".equals(e.getKey()) || "TR".equals(e.getKey()))
                        .map(Map.Entry::getValue)
                        .forEach(j->this.updatePlot(j, order, false, _radar));
                vOrderedDrones.get(i).radars.entrySet().stream()
                        .filter(e->"BL".equals(e.getKey()) || "BR".equals(e.getKey()))
                        .map(Map.Entry::getValue)
                        .forEach(j->this.updatePlot(j, order+1, false, _radar));
                _yPositions[i] = vOrderedDrones.get(i).pos.y;
            }

            // calibrate col and line (make them 0 index)
            int oversize = IntStream.range(0, myDrones.size()).sum();
            _radar.values().forEach(p->{
                p.col-=oversize;
                p.line-=oversize;
            });

            //_radar.values().forEach(p->System.err.println("COP id="+p.id+" (c,l)="+p.col+","+p.line+" x="+creatures.get(p.id).pos.x+" y="+creatures.get(p.id).pos.y));

            // update creature approximate position (for none visible)
            // center of boundingbox + be more precise if change radar zone horizontally or vertically
            _radar.values().forEach(p-> {
                Creature c = creatures.get(p.id);
                p.lineWasVisible--;
                p.colWasVisible--;
                c.wasVisible--;
                //System.err.println("COP UPDATE id="+p.id+" col="+p.col+" lin="+p.line+" wasC="+p.colWasVisible+" wasL="+p.lineWasVisible);
                if(!c.visible && c.wasVisible<=0) {
                    Depth depth = Depth.BY_TYPE.get(c.type);

                    if(p.colWasVisible<=0) {
                        long xmin = p.col==0?0:_xPositions[p.col-1];
                        long xmax = p.col==myDrones.size()?10000:_xPositions[p.col];
                        c.pos.x = Math.round((float) (xmax + xmin) /2);
                    }
                    if(p.lineWasVisible<=0) {
                        long ymin = p.line==0?0:_yPositions[p.line-1];
                        long ymax = p.line==myDrones.size()?10000:_yPositions[p.line];
                        ymin = Math.max(ymin,depth.start);
                        ymax = Math.min(ymax,depth.end);
                        c.pos.y = Math.round((float) (ymax + ymin) /2);
                    }
                    RadarPlot previousPlot = radar.get(p.id);
                    if(previousPlot!=null && previousPlot.col!=p.col) {
                        // plot switched zone horizontally : we can be more precise for x
                        int i = p.col<previousPlot.col?p.col:p.col-1;
                        p.colWasVisible = 4;
                        c.pos.x = Math.round((float) (_xPositions[i] + xPositions[i]) /2);
                    }
                    if(previousPlot!=null && previousPlot.line!=p.line) {
                        // plot switched zone vertically : we can be more precise for y
                        int i = p.line<previousPlot.line?p.line:p.line-1;
                        p.lineWasVisible = 4;
                        c.pos.y = Math.round((float) (Math.min(Math.max(_yPositions[i], depth.start), depth.end) + Math.min(Math.max(yPositions[i], depth.start), depth.end)) /2);
                    }
                }
            });

            this.radar = _radar;
            this.xPositions = _xPositions;
            this.yPositions = _yPositions;

            radar.values().stream().sorted(Comparator.comparing(p->p.id)).forEach(p->System.err.println("COP id="+p.id+" (c,l)="+p.col+","+p.line+" x="+creatures.get(p.id).pos.x+" y="+creatures.get(p.id).pos.y
                    +" vis="+creatures.get(p.id).visible+" was="+creatures.get(p.id).wasVisible + " (wasC, wasL)="+p.colWasVisible+","+p.lineWasVisible));
        }

        private void updatePlot(Set<Integer> indexes, int order, boolean isCol, Map<Integer, RadarPlot> _radar) {
            indexes.stream()
                    .map(creatures::get)
                    .forEach(c-> {
                        _radar.putIfAbsent(c.id, new RadarPlot());
                        RadarPlot plot = _radar.get(c.id);
                        plot.id = c.id;
                        RadarPlot previousPlot = radar.get(c.id);
                        if(previousPlot!=null) {
                            plot.colWasVisible = previousPlot.colWasVisible;
                            plot.lineWasVisible = previousPlot.lineWasVisible;
                        }
                        if(isCol) plot.col+=order;
                        else plot.line+=order;
                    });
        }

        public List<Creature> getCreatures() {
            return radar.values().stream().map(p->creatures.get(p.id)).toList();
        }

        public Point getTargetPosition(Drone drone, Creature target) {
            RadarPlot plot = radar.get(target.id);
            Depth depth = Depth.BY_TYPE.get(target.type);
            long xmin=0,xmax=0,ymin=0,ymax=0;
            if(plot.colWasVisible>0 || target.visible || target.wasVisible>0) {
                xmin = target.pos.x;
                xmax = xmin;
            } else {
                xmin = plot.col==0?0:xPositions[plot.col-1];
                xmax = plot.col==xPositions.length?10000:xPositions[plot.col];
            }
            if(plot.lineWasVisible>0 || target.visible || target.wasVisible>0) {
                ymin = target.pos.y;
                ymax = ymin;
            } else {
                ymin = plot.line==0?0:yPositions[plot.line-1];
                ymax = plot.line==yPositions.length?10000:yPositions[plot.line];
                ymin = Math.max(ymin,depth.start);
                ymax = Math.min(ymax,depth.end);
            }
            /*Point p1 = new Point(xmin,ymin);
            Point p2 = new Point(xmin,ymax);
            Point p3 = new Point(xmax,ymin);
            Point p4 = new Point(xmax,ymax);
            return Stream.of(p1,p2,p3,p4).max(Comparator.comparing(p -> drone.pos.distanceTo(p))).orElse(null);*/
            Point p = new Point((xmin+xmax)/2, (ymin+ymax)/2); // return middle of the cell
            System.err.println("GET TARGET POSITION d="+ drone.id+" c="+target.id+" wasL="+plot.lineWasVisible+" wasC="+plot.colWasVisible+" p.x="+p.x+" p.y="+p.y+" ["+xmin+","+xmax+"] ["+ymin+","+ymax+"]");
            return p;
        }
    }

    static class RadarPlot {
        int id;
        int col;
        int line;
        int colWasVisible=0;
        int lineWasVisible=0;
    }

    static abstract class Strategy {
        Creature target = null;
        String msg = "";

        public abstract boolean isFinished(Drone drone);

        public abstract Point getTarget(Drone drone);

        public abstract boolean isLightOn(Drone drone);

    }

    static class Dive extends Strategy {

        /**
         * 1) Dive to closest type3
         * 2) as soon as 2 type3 are scanned => both robots surface and need to scan every type1 and type2
         * 3) the closest to surface scan type2 and typein priority only
         * 4) the deepest to surface scan
         */

        enum DiveStep {RACE_TO_TYPE3, SURFACE, RACE_TO_SURFACE}

        DiveStep step = null;

        public Point getTarget(Drone drone) {
            final List<Creature> notScanned = cop.getCreatures().stream()
                    .filter(c->c.type!=-1)
                    .filter(c->!c.myScan)
                    .collect(Collectors.toList()); // collect to modifiable collection

            // ignore creatures currently scanned by our drones
            drones.values().stream().filter(d->d.mine).forEach(d->
                    notScanned.removeAll(d.currentScan.stream().map(i->creatures.get(i)).toList()));

            // if our current target was scanned, change it
            if(target!=null && !notScanned.contains(target)) target = null;

            if(creatures.values().stream().filter(c->c.foeScan).count() >= 6 && myScore == 0) {
                step=DiveStep.RACE_TO_SURFACE;
                target = null;
            } else if (canWin()) {
                step = DiveStep.SURFACE;
                target = null;
            } /*else if (step==DiveStep.RACE_TO_TYPE3) {
                Drone bestScoreDrone = drones.values().stream().max(Comparator.comparing(Drone::getCurrentScanScore)).orElse(null);
                step=(bestScoreDrone!=null && bestScoreDrone.id==drone.id)?DiveStep.RACE_TO_SURFACE:DiveStep.SURFACE;
                target = null;
            } */
            else {
                if(step!=DiveStep.RACE_TO_TYPE3) {
                    step=DiveStep.RACE_TO_TYPE3;
                    target = null;
                }

            }

            if(target == null) {
                if(step==DiveStep.SURFACE) {
                    // 2x type3 were scanned
                    // go to surface and scan all L2+L1 remaining creatures
                    target = getPossibleTarget(drone, notScanned, Depth.L2, new ArrayList<>());
                } else if (step==DiveStep.RACE_TO_SURFACE) {
                    target = null;
                } else{
                    target = getPossibleTarget(drone, notScanned, Depth.L3, new ArrayList<>());
                }
            }

            // if all drone's scan were scored while going to surface, choose another target
            if(target==null && drone.currentScan.stream().map(i->creatures.get(i)).filter(c->!c.myScan).count()==0) {
                target = getPossibleTarget(drone, notScanned, Depth.L3, new ArrayList<>());
            }

            if(target == null) {
                msg = "SURFACE";
                System.err.println("#### DIVE getTarget=SURFACE");
                return new Point(drone.pos.x, DEPTH_SURFACE); // go to surface
            } else {
                msg = "DIVE "+target.id;
                System.err.println("#### DIVE getTarget="+target.id);
                return cop.getTargetPosition(drone, target);
            }

            // TODO
            //- if target is close (800) to another robot's target => choose another one

        }

        public static Creature getPossibleTarget(Drone drone, List<Creature> notScanned, Depth maxDepth, List<Creature> exclude) {
            List<Drone> myOtherDrones = drones.values().stream()
                    .filter(d-> drone.id!=d.id) // other
                    .filter(d->d.mine) // mine
                    .toList();

            Creature target = null;
            Depth depth = maxDepth;
            boolean run = true;
            while(run && depth!=Depth.L0) {
                // target notScanned creatures by priority L3->L1
                final Depth dd = depth;
                Creature possibleTarget = notScanned.stream()
                        .filter(c -> c.type == dd.type)
                        .filter(c -> !exclude.contains(c))
                        .min(Comparator.comparing(c -> drone.pos.distanceTo(c.pos))).orElse(null);

                if (possibleTarget == null) {
                    if (depth == Depth.L3) depth = Depth.L2;
                    else if (depth == Depth.L2) depth = Depth.L1;
                    else depth = Depth.L0;
                } else {
                    //List<Drone> droneWithSameTarget = myOtherDrones.stream().filter(d -> d.peekNextMission().target == possibleTarget).toList();
                    boolean droneWithSameTargetCell = myOtherDrones.stream().anyMatch(d -> {
                        if(d.peekNextMission().target == null || possibleTarget == null) return false;
                        RadarPlot p1 = cop.radar.get(d.peekNextMission().target.id);
                        RadarPlot p2 = cop.radar.get(possibleTarget.id);
                        if(p1 == null || p2 == null) return false;
                        //System.err.println("!!!!!!!!!! "+p1.id+","+p1.col+","+p1.line+"  "+p2.id+","+p2.col+","+p2.line);
                        return p1.col==p2.col && p1.line== p2.line;
                    });
                    if (!droneWithSameTargetCell) {
                        target = possibleTarget;
                        run = false;
                    } else {
                        // do we need to switch target if our possible target is of another drone ?
                        exclude.add(possibleTarget);
                    }
                }
            }
            return target;
        }

        public boolean canWin() {

            List<Drone> myDrones = drones.values().stream().filter(d->d.mine).toList();

            Set<Integer> notScored = creatures.values().stream().filter(c->c.type!=-1 && !c.myScan && !c.foeScan ).map(c->c.id).collect(Collectors.toSet());
            Set<Integer> remaining = creatures.values().stream().filter(c->c.type!=-1 && !c.myScan).map(c->c.id).collect(Collectors.toSet());

            Set<Integer> possibleScoredAsFirst = new HashSet<>();
            // all our scan which are not scored
            myDrones.forEach(d->possibleScoredAsFirst.addAll(d.currentScan.stream().filter(notScored::contains).toList()));
            // add all not scored scans from L1 and L2
            notScored.stream().map(creatures::get).filter(c->c.type<Depth.L3.type).map(c->c.id).forEach(possibleScoredAsFirst::add);

            /*
            calculate our potential score if we score all our current scan and above creatures as first
            it's the sum of
            + current score
            + all current scan if scored as first
            + L2/L1 creatures not scanned if scored as first
            + all remaining if scored as 2nd
             */

            int myPossibleScore = myScore // currentScore
                    + possibleScoredAsFirst.stream().map(creatures::get).mapToInt(c->(c.type+1)*2).sum() // all current scanned and if scored as first
                    + remaining.stream().filter(i->!possibleScoredAsFirst.contains(i)).map(creatures::get).mapToInt(c->(c.type+1)).sum(); // all remaining creatures
            // check type combo
            for(int type=0;type<=2;type++) {
                int i = type;
                if(creatures.values().stream().anyMatch(c->c.type==i && !c.myScan)) {
                    // a combo is possible
                    boolean isComboAsFirstPossible = creatures.values().stream().anyMatch(c->c.type==i && !c.foeScan); // if foe didn't scored all type
                    Predicate<Creature> isScoredAsFirst = c->possibleScoredAsFirst.contains(c.id) || c.myScan;
                    myPossibleScore+=(isComboAsFirstPossible && creatures.values().stream().filter(c->c.type==i).allMatch(isScoredAsFirst))?8:4;
                }
            }
            // check color combo
            for(int color=0;color<=3;color++) {
                int i = color;
                if(creatures.values().stream().anyMatch(c->c.color==i && !c.myScan)) {
                    // a combo is possible
                    boolean isComboAsFirstPossible = creatures.values().stream().anyMatch(c->c.color==i && !c.foeScan); // if foe didn't scored all color
                    Predicate<Creature> isScoredAsFirst = c->possibleScoredAsFirst.contains(c.id) || c.myScan;
                    myPossibleScore+=(isComboAsFirstPossible && creatures.values().stream().filter(c->c.color==i).allMatch(isScoredAsFirst))?6:3;
                }
            }

            Set<Integer> foePossibleScoredAtFirst = notScored.stream().filter(i->!possibleScoredAsFirst.contains(i)).collect(Collectors.toSet());
            remaining = creatures.values().stream().filter(c->c.type!=-1 && !c.foeScan).map(c->c.id).collect(Collectors.toSet());

            int foePossibleScore = foeScore // currentScore
                    + foePossibleScoredAtFirst.stream().map(creatures::get).mapToInt(c->(c.type+1)*2).sum() // all current scanned and if scored as first
                    + remaining.stream().filter(i->!foePossibleScoredAtFirst.contains(i)).map(creatures::get).mapToInt(c->(c.type+1)).sum(); // all remaining creatures
            // check foe type combo
            for(int type=0;type<=2;type++) {
                int i = type;
                if(creatures.values().stream().anyMatch(c->c.type==i && !c.foeScan)) {
                    // a combo is possible
                    boolean isComboAsFirstPossible = creatures.values().stream().anyMatch(c->c.type==i && !c.myScan); // if we didn't scored all type
                    Predicate<Creature> isScoredAsFirst = c->foePossibleScoredAtFirst.contains(c.id) || c.foeScan;
                    foePossibleScore+=(isComboAsFirstPossible && creatures.values().stream().filter(c->c.type==i).anyMatch(isScoredAsFirst))?8:4;
                }
            }
            // check foe color combo
            for(int color=0;color<=3;color++) {
                int i = color;
                if(creatures.values().stream().anyMatch(c->c.color==i && !c.foeScan)) {
                    boolean isComboAsFirstPossible = creatures.values().stream().anyMatch(c->c.color==i && !c.myScan); // if I didn't scored all color
                    Predicate<Creature> isScoredAsFirst = c->foePossibleScoredAtFirst.contains(c.id) || c.foeScan;
                    foePossibleScore+=(isComboAsFirstPossible && creatures.values().stream().filter(c->c.color==i).anyMatch(isScoredAsFirst))?6:3;
                }
            }
            System.err.println("CAN WIN my="+myPossibleScore+" foe="+foePossibleScore);

            return myPossibleScore > foePossibleScore;
        }

        public boolean isFinished(Drone drone) {
            return target == null && drone.pos.y<500;
        }

        @Override
        public boolean isLightOn(Drone drone) {
            final List<Creature> notScanned = cop.getCreatures().stream()
                    .filter(c->c.type!=-1)
                    .filter(c->!c.myScan)
                    .filter(c->!drone.currentScan.contains(c.id)) // ignore creatures currently scanned by our drones
                    .toList(); // collect to modifiable collection

            return (myScore==0 && drone.pos.y>2000 && turn%DIVE_LIGHT_ON_EVERY==0) || (step!=DiveStep.RACE_TO_TYPE3 && notScanned.stream().anyMatch(c->drone.pos.distanceTo(c.pos)<3000));
        }
    }

    static class Avoid extends Strategy {

        Strategy next;
        boolean finished = false;
        int startTurn;

        public Avoid(Strategy next) {
            this.next = next;
            target = next.target;
            msg = "AVOID";
            startTurn = turn;
        }


        @Override
        public Point getTarget(Drone drone) {
            if(target == null) next.target = null;

            System.err.println("AVOID getTarget droneId="+drone.id);

            // select monsters type going towards us
            List<Creature> monsters = creatures.values().stream()
                    .filter(m -> m.type == -1 && (m.visible || m.wasVisible>0 || cop.radar.get(m.id).lineWasVisible>0 || cop.radar.get(m.id).colWasVisible>0))
                    .collect(Collectors.toList()); // collect to a modifiable List

            // select all monsters inside our light and goig towards us
            List<Creature> absoluteDanger = monsters.stream()
                    .filter(m -> drone.pos.distanceTo(m.pos) <= 650 ) // inside LIGHT 0 ZONE
                    .filter(m -> drone.pos.distanceTo(m.pos) >= drone.pos.distanceTo(m.pos.add(new Point(m.vx, m.vy)))) // going towards me
                    .collect(Collectors.toList());
            absoluteDanger.forEach(m -> System.err.println("AVOID getTarget ABSOLUTEDANGER id="+m.id+" m.x="+m.pos.x+" m.y="+m.pos.y+" dist="+drone.pos.distanceTo(m.pos)+" m.vis="+m.visible+" m.was="+m.wasVisible+" vx="+m.vx+" vy="+m.vy));

            // select monsters who are under our light but not closer to 800
            Predicate<Creature> visibleAndGoingCloseToMe = m -> m.visible && drone.pos.distanceTo(m.pos) >= drone.pos.distanceTo(m.pos.add(new Point(m.vx, m.vy))) && drone.pos.distanceTo(m.pos.add(new Point(m.vx, m.vy))) < LIGHT_1_RADIUS;
            Predicate<Creature> wasVisibleAndCloseToMe = m -> !m.visible && drone.pos.distanceTo(m.pos)<2000;
            List<Creature> potentialDanger = monsters.stream()
                    .filter(m -> !absoluteDanger.contains(m))
                    .filter(visibleAndGoingCloseToMe.or(wasVisibleAndCloseToMe))
                    .collect(Collectors.toList());
            potentialDanger.forEach(m -> System.err.println("AVOID getTarget POTENTIAL id="+m.id+" m.x="+m.pos.x+" m.y="+m.pos.y+" dist="+drone.pos.distanceTo(m.pos)+" m.vis="+m.visible+" m.was="+m.wasVisible+" vx="+m.vx+" vy="+m.vy));

            // select all monsters being globaly visible which could a potential danger in next turn
            List<Creature> otherPotential = monsters.stream()
                    .filter(m -> !absoluteDanger.contains(m))
                    .filter(m -> !potentialDanger.contains(m))
                    .collect(Collectors.toList());
            // add creature from radar
            cop.radar.values().stream()
                    .map(p->creatures.get(p.id))
                    .filter(m -> m.type == -1)
                    .filter(m -> !monsters.contains(m))
                    .filter(m -> drone.pos.distanceTo(m.pos)<2000)
                    .forEach(otherPotential::add);

            otherPotential.forEach(m -> System.err.println("AVOID getTarget OTHER id="+m.id+" m.x="+m.pos.x+" m.y="+m.pos.y+" dist="+drone.pos.distanceTo(m.pos)+" m.vis="+m.visible+" m.was="+m.wasVisible+" vx="+m.vx+" vy="+m.vy));

            Set<Integer> additionnal = new HashSet<>();

            Point t = new Point(5000,5000);
            boolean run = true;
            while (run) {
                finished = false;
                Vector avoidVector = new Vector();

                System.err.println("AVOID getTarget WHILE droneId="+drone.id);

                if(!absoluteDanger.isEmpty() || !potentialDanger.isEmpty()) {
                    Set<Integer> list = new HashSet<>();
                    if (!absoluteDanger.isEmpty()) {
                        System.err.println("AVOID getTarget step 1");
                        list.addAll(absoluteDanger.stream().map(m -> m.id).toList());
                    } else {
                        System.err.println("AVOID getTarget step 2");
                    }
                    list.addAll(potentialDanger.stream().map(m -> m.id).toList());
                    list.addAll(additionnal);

                    Vector vec = new Vector();
                    list.stream()
                            .map(i -> creatures.get(i))
                            .forEach(m -> {
                                Point nextPos = m.pos.add(new Point(m.vx, m.vy));
                                double dist = drone.pos.distanceTo(nextPos);
                                Vector v = nextPos.vectorTo(drone.pos);
                                if(dist<1) {
                                    dist = 1;
                                    v = m.pos.vectorTo(drone.pos);
                                }
                                vec.x += v.x / (dist * ((m.visible || m.wasVisible>0)?1:dist)); // prioritize monsters which are/was really visible (not based only on radar)
                                vec.y += v.y / (dist * ((m.visible || m.wasVisible>0)?1:dist));
                                //System.err.println("AVOID getTarget step : monster.id=" + m.id + " dist=" + drone.pos.distanceTo(m.pos) + " nextDist=" + dist + " m.vis="+m.visible+" m.was="+m.wasVisible+" vec.x=" + vec.normalizedDirection(100).x + " vec.y=" + vec.normalizedDirection(100).y);
                                System.err.println("AVOID getTarget step : c.id="+m.id+" v.x="+v.x+" v.y="+v.y);
                            });

                    avoidVector = vec;
                    if (absoluteDanger.isEmpty()) {
                        double size = 72;
                        List<Point> points = IntStream.range(0,(int)size).mapToObj(a->drone.pos.add(vec.rotate(2d*(double)a*Math.PI/size).normalizedDirection(MAX_MOVE))).toList();

                        Point nextTarget = next.getTarget(drone);
                        //nextTarget.x = drone.pos.y<nextTarget.y-1000?drone.pos.x:nextTarget.x;
                        Point avoidPoint = points.stream()
                                // filter out points which are out of map
                                .filter(p-> p.x>=0 && p.x<=10000 && p.y>=0 && p.y<=10000)
                                // filter out points which are in the LIGHT_0_RADIUS of current position of the monster
                                .filter(p-> list.stream().map(creatures::get).noneMatch(m->p.distanceTo(m.pos) <= (m.visible?LIGHT_0_RADIUS+50:LIGHT_0_RADIUS*2)))
                                // filter out points which are in the LIGHT_0_RADIUS of next position of the monster
                                .filter(p-> list.stream().map(creatures::get).noneMatch(m->p.distanceTo(m.pos.add(new Point(m.vx, m.vy))) <= (m.visible?LIGHT_0_RADIUS+50:LIGHT_0_RADIUS*2)))
                                // get the one which minimize the distance to target
                                .min(Comparator.comparing(p->p.distanceTo(nextTarget))).orElse(null);

                        if(avoidPoint!=null) {
                            // check if we can go directly to destination
                            avoidVector = drone.pos.vectorTo(avoidPoint);
                            double a = avoidVector.angleTo(drone.pos.vectorTo(nextTarget));
                            System.err.println("AVOID getTarget step : avoid.x=" + avoidVector.x + " avoid.y=" + avoidVector.y + " a=" + a);

                        } else {
                            System.err.println("AVOID getTarget step : COULDN'T find any escape vector => going directly to target");
                        }
                    }


                } else {
                    finished = true;
                }

                if (avoidVector.size() == 0) {
                    avoidVector = drone.pos.vectorTo(next.getTarget(drone));
                }
                Point tmp = drone.pos.add(avoidVector.normalizedDirection(MAX_MOVE));
                t = tmp;

                // check whether there is a potential kill
                List<Creature> killHits = otherPotential.stream()
                        .filter(m -> !additionnal.contains(m.id))
                        .filter(m -> tmp.distanceTo(m.pos.add(new Point(m.vx, m.vy))) < (m.visible?LIGHT_0_RADIUS:KILL_DISTANCE*2))
                        .toList();
                killHits.forEach(m -> System.err.println("AVOID getTarget KILL HITS id=" + m.id));
                if (killHits.isEmpty()) {
                    run = false;
                } else {
                    additionnal.addAll(killHits.stream().map(m -> m.id).toList());
                    potentialDanger.addAll(killHits.stream().filter(m->!potentialDanger.contains(m)).toList());
                }

            }
            Vector v = drone.pos.vectorTo(t);
            if(drone.pos.x>300 && t.x < 300) {
                System.err.println("AVOID getTarget OUT_OF_MAP x<300 target.x="+t.x);
                v.x/=2;
                v.y*=2;
                t = drone.pos.add(v.normalizedDirection(MAX_MOVE));
            }
            if(drone.pos.x<9700 && t.x > 9700) {
                System.err.println("AVOID getTarget OUT_OF_MAP x>9700 target.x="+t.x);
                v.x/=2;
                v.y*=2;
                t = drone.pos.add(v.normalizedDirection(MAX_MOVE));
            }
            if(drone.pos.y<9700 && t.y > 9700) {
                System.err.println("AVOID getTarget OUT_OF_MAP y>9700 next.y="+t.y);
                v.x*=2;
                v.y/=2;
                t = drone.pos.add(v.normalizedDirection(MAX_MOVE));
            }
            if(finished) msg = next.msg;
            return t;
        }

        @Override
        public boolean isLightOn(Drone drone) {
            // light if nearby our target (because if scanned, it can change the next one)
            // or a nearby monsters just disappeared (wasVisible=1)
            boolean light = creatures.values().stream().anyMatch(m -> m.type == -1 && !m.visible && m.wasVisible==1 && drone.pos.distanceTo(m.pos)<2000)
                    || drone.pos.distanceTo(next.getTarget(drone))<LIGHT_1_RADIUS+500;
            if(light) drone.disableLightTurnCounter = 0;
            return light;
        }

        @Override
        public boolean isFinished(Drone drone) {
            return finished;
        }
    }

    static class Creature {
        int id;
        int color;
        int type;
        Point pos = new Point(0,0);
        int vx;
        int vy;
        boolean myScan = false;
        boolean foeScan = false;
        boolean visible = false;
        int wasVisible = 0;

    }

    static class Drone {
        int id;
        boolean mine = false;
        Point pos = new Point(0,0);
        int emergency;
        int battery;
        Set<Integer> currentScan = new HashSet<>();
        Map<String, Set<Integer>> radars = new HashMap<>();

        final Deque<Strategy> strategy = new ArrayDeque<>();
        int initPosition = -1;
        int disableLightTurnCounter = 0;
        boolean firstToScan = false;

        boolean light = false;

        public Drone() {
            radars.put("BL", new HashSet<>());
            radars.put("BR", new HashSet<>());
            radars.put("TL", new HashSet<>());
            radars.put("TR", new HashSet<>());
        }

        private Strategy peekNextMission() {
            if(strategy.isEmpty()) {
                if(initPosition==0 || initPosition==drones.size()-1) {
                    strategy.add(new Dive());
                } else {
                    strategy.add(new Dive());
                }
            }
            return strategy.peek();
        }

        void resetRadar() {
            radars.values().forEach(Set::clear);
        }

        public void process() {
            if(emergency==1) {
                strategy.clear();
                System.out.println("WAIT 0 EMERGENCY");
                return;
            }
            disableLightTurnCounter--;
            if(initPosition == -1) {
                List<Drone> hSortedDrones= drones.values().stream().sorted(Comparator.comparing(d -> d.pos.x)).toList();
                initPosition = hSortedDrones.indexOf(this);
            }

            Strategy next = peekNextMission();
            if(next.isFinished(this)) {
                strategy.poll();
                next = peekNextMission();
            }
            System.err.println("\n======================================================================");
            System.err.println("DRONE.process droneId="+this.id+" next="+next.getClass().getSimpleName());

            // handle monsters
            List<Creature> monsters = creatures.values().stream()
                    .filter(c -> c.visible || c.wasVisible>0) // keep only visible
                    .filter(c -> c.type == -1) // keep only monsters
                    .filter(c -> this.pos.distanceTo(c.pos) <= LIGHT_MONSTERS_EXTRA+LIGHT_1_RADIUS)
                    .toList();
            //monsters.forEach(m -> System.err.println("MONSTERS drone="+this.id+" monster="+m.id));
            // monster avoidance management
            if(!monsters.isEmpty() && !(next instanceof Avoid)) {
                next = new Avoid(next);
                strategy.push(next);
            }

            // get next waypoint
            Point target = next.getTarget(this);

            // light management
            light = disableLightTurnCounter<=0 && battery>=5 && next.isLightOn(this);
            if(light) this.disableLightTurnCounter=2;

            System.out.println("MOVE "+target.x+" "+target.y+" "+(light?1:0)+" B="+this.battery+" "+next.msg);
        }

    }

    public static void main(String args[]) {
        Scanner in = new Scanner(System.in);
        int creatureCount = in.nextInt();
        int byColor=0;
        int byType=0;
        for (int i = 0; i < creatureCount; i++) {
            Creature creature = new Creature();
            creature.id = in.nextInt();
            creature.color = in.nextInt();
            creature.type = in.nextInt();
            creatures.put(creature.id, creature);
        }

        // game loop
        while (true) {
            turn++;
            myScore = in.nextInt();
            foeScore = in.nextInt();
            int myScanCount = in.nextInt();
            for (int i = 0; i < myScanCount; i++) {
                int creatureId = in.nextInt();
                creatures.get(creatureId).myScan = true;
            }
            int foeScanCount = in.nextInt();
            for (int i = 0; i < foeScanCount; i++) {
                int creatureId = in.nextInt();
                creatures.get(creatureId).foeScan = true;
            }
            int myDroneCount = in.nextInt();
            for (int i = 0; i < myDroneCount; i++) {
                int droneId = in.nextInt();
                drones.putIfAbsent(droneId, new Drone());
                Drone drone = drones.get(droneId);
                drone.mine = true;
                drone.id = droneId;
                drone.pos.x = in.nextInt();
                drone.pos.y = in.nextInt();
                drone.emergency = in.nextInt();
                drone.battery = in.nextInt();
                drone.currentScan.clear();
            }
            int foeDroneCount = in.nextInt();
            for (int i = 0; i < foeDroneCount; i++) {
                int droneId = in.nextInt();
                drones.putIfAbsent(droneId, new Drone());
                Drone drone = drones.get(droneId);
                drone.id = droneId;
                drone.pos.x = in.nextInt();
                drone.pos.y = in.nextInt();
                drone.emergency = in.nextInt();
                drone.battery = in.nextInt();
            }
            int droneScanCount = in.nextInt();
            for (int i = 0; i < droneScanCount; i++) {
                int droneId = in.nextInt();
                int creatureId = in.nextInt();
                Drone drone = drones.get(droneId);
                //System.err.println("SCAN droneId="+droneId+" creatureId="+creatureId);
                if(drone.emergency==0) drone.currentScan.add(creatureId);
            }
            creatures.values().forEach(creature -> {
                creature.wasVisible = creature.visible?3:creature.wasVisible;
                creature.visible=false;
            });
            int visibleCreatureCount = in.nextInt();
            for (int i = 0; i < visibleCreatureCount; i++) {
                int creatureId = in.nextInt();
                Creature creature = creatures.get(creatureId);
                creature.visible = true;
                creature.pos.x = in.nextInt();
                creature.pos.y = in.nextInt();
                creature.vx = in.nextInt();
                creature.vy = in.nextInt();
            }
            creatures.values().stream().filter(c->!c.visible && c.wasVisible>0).forEach(c -> {
                c.pos.x += c.vx;
                c.pos.y += c.vy;
            });

            int radarBlipCount = in.nextInt();
            drones.values().forEach(Drone::resetRadar);
            for (int i = 0; i < radarBlipCount; i++) {
                int droneId = in.nextInt();
                int creatureId = in.nextInt();
                String radar = in.next();
                drones.get(droneId).radars.get(radar).add(creatureId);
                //System.err.println("RADAR droneId="+droneId+" creatureId="+creatureId+" radar="+radar);
            }
            cop.update();
            drones.values().stream().filter(c->c.mine).forEach(Drone::process);
        }
    }
}