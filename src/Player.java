import java.util.*;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import java.util.stream.Stream;

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

    final static int DIVE_LIGHT_ON_EVERY = 3;

    static Map<Integer, Creature> creatures = new HashMap<>();
    static Map<Integer, Drone> drones = new HashMap<>();
    static CommonOperationPicture cop = new CommonOperationPicture();

    static int turn = 0;

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

        Point copy() {
            return new Point(this.x, this.y);
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

        Vector copy() {
            Vector copy = new Vector();
            copy.x = this.x;
            copy.y = this.y;
            return copy;
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
                if(!c.visible && !c.wasVisible) {
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
                        p.colWasVisible = 2;
                        c.pos.x = Math.round((float) (_xPositions[i] + xPositions[i]) /2);
                    }
                    if(previousPlot!=null && previousPlot.line!=p.line) {
                        // plot switched zone vertically : we can be more precise for y
                        int i = p.line<previousPlot.line?p.line:p.line-1;
                        p.lineWasVisible = 2;
                        c.pos.y = Math.round((float) (_yPositions[i] + yPositions[i]) /2);
                    }
                }
            });

            this.radar = _radar;
            this.xPositions = _xPositions;
            this.yPositions = _yPositions;

            radar.values().forEach(p->System.err.println("COP id="+p.id+" (c,l)="+p.col+","+p.line+" x="+creatures.get(p.id).pos.x+" y="+creatures.get(p.id).pos.y));
        }

        private void updatePlot(Set<Integer> indexes, int order, boolean isCol, Map<Integer, RadarPlot> _radar) {
            indexes.stream()
                    .map(i->creatures.get(i))
                    .forEach(c-> {
                        _radar.putIfAbsent(c.id, new RadarPlot());
                        RadarPlot plot = _radar.get(c.id);
                        plot.id = c.id;
                        if(isCol) plot.col+=order;
                        else plot.line+=order;
                    });
        }

        public List<Creature> getCreatures() {
            return radar.values().stream().map(p->creatures.get(p.id)).toList();
        }

        public Point getCollidingTarget(Drone drone, Creature target) {
            RadarPlot plot = radar.get(target.id);
            Depth depth = Depth.BY_TYPE.get(target.type);
            long xmin=0,xmax=0,ymin=0,ymax=0;
            if(plot.colWasVisible>0 || target.visible || target.wasVisible) {
                xmin = target.pos.x;
                xmax = xmin;
            } else {
                xmin = plot.col==0?0:xPositions[plot.col-1];
                xmax = plot.col==xPositions.length?10000:xPositions[plot.col];
            }
            if(plot.lineWasVisible>0 || target.visible || target.wasVisible) {
                ymin = target.pos.y;
                ymax = ymin;
            } else {
                ymin = plot.line==0?0:yPositions[plot.line-1];
                ymax = plot.line==yPositions.length?10000:yPositions[plot.line];
                ymin = Math.max(ymin,depth.start);
                ymax = Math.min(ymax,depth.end);
            }
            Point p1 = new Point(xmin,ymin);
            Point p2 = new Point(xmin,ymax);
            Point p3 = new Point(xmax,ymin);
            Point p4 = new Point(xmax,ymax);
            return Stream.of(p1,p2,p3,p4).max(Comparator.comparing(p -> drone.pos.distanceTo(p))).orElse(null);
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

    /**
     * 1) Dive to closest type3
     * 2) as soon as 2 type3 are scanned => both robots surface and need to scan every type1 and type2
     * 3) the closest to surface scan type2 and typein priority only
     * 4) the deepest to surface scan
     */
    static class Dive extends Strategy {

        enum DiveStep {RACE_TO_TYPE3, RACE_SURFACE, COMPLETE}

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

            long remainingType3 = notScanned.stream().filter(c->c.type==Depth.L3.type).count();
            if(remainingType3>2) {
                if(step!=DiveStep.RACE_TO_TYPE3) {
                    step=DiveStep.RACE_TO_TYPE3;
                    target = null;
                }
                // find a type3
                if(target==null) target = getPossibleTarget(drone, notScanned, Depth.L3, new ArrayList<>());
            } else if(step==DiveStep.RACE_TO_TYPE3 || step==DiveStep.RACE_SURFACE) {
                if(step!=DiveStep.RACE_SURFACE) {
                    step=DiveStep.RACE_SURFACE;
                    target = null;
                }
                // 2x type3 were scanned
                // go to surface and scan all L2+L1 remaining creatures
                if(target==null) target = getPossibleTarget(drone, notScanned, Depth.L2, new ArrayList<>());
            } else{
                if(target==null) target = getPossibleTarget(drone, notScanned, Depth.L3, new ArrayList<>());
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
                return cop.getCollidingTarget(drone, target);
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
                    List<Drone> droneWithSameTarget = myOtherDrones.stream().filter(d -> d.peekNextMission().target == possibleTarget).toList();
                    if (droneWithSameTarget.isEmpty()) {
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

        public boolean isFinished(Drone drone) {
            return target == null && drone.pos.y<500;
        }

        @Override
        public boolean isLightOn(Drone drone) {
            final List<Creature> notScanned = cop.getCreatures().stream()
                    .filter(c->c.type!=-1)
                    .filter(c->!c.myScan)
                    .collect(Collectors.toList()); // collect to modifiable collection
            // ignore creatures currently scanned by our drones
            drones.values().stream().filter(d->d.mine).forEach(d->
                    notScanned.removeAll(d.currentScan.stream().map(i->creatures.get(i)).toList()));

            return (drone.pos.y>2000 && turn%DIVE_LIGHT_ON_EVERY==0) || (step!=DiveStep.RACE_TO_TYPE3 && notScanned.stream().anyMatch(c->drone.pos.distanceTo(c.pos)<3000));
        }
    }

    static class Avoid extends Strategy {

        Strategy next;
        boolean finished = false;

        public Avoid(Strategy next) {
            this.next = next;
            target = next.target;
            msg = "AVOID";
        }


        @Override
        public Point getTarget(Drone drone) {
            if(target == null) next.target = null;

            System.err.println("AVOID getTarget droneId="+drone.id);

            // select monsters type going towards us
            List<Creature> monsters = creatures.values().stream()
                    .filter(m -> m.type == -1 && (m.visible || m.wasVisible || cop.radar.get(m.id).lineWasVisible>0 || cop.radar.get(m.id).colWasVisible>0))
                    .collect(Collectors.toList()); // collect to a modifiable List

            // select all monsters inside 800u
            List<Creature> absoluteDanger = monsters.stream()
                    .filter(m -> drone.pos.distanceTo(m.pos) <= 800 ) // inside LIGHT 0 ZONE
                    .filter(m -> drone.pos.distanceTo(m.pos) >= drone.pos.distanceTo(m.pos.add(new Point(m.vx, m.vy)))) // going towards me
                    .collect(Collectors.toList());
            absoluteDanger.forEach(m -> System.err.println("AVOID getTarget ABSOLUTEDANGER id="+m.id));

            // select monsters who are under our light but not closer to 800
            List<Creature> potentialDanger = monsters.stream()
                    .filter(m -> !absoluteDanger.contains(m))
                    .filter(m -> drone.pos.distanceTo(m.pos) <= LIGHT_MONSTERS_EXTRA+LIGHT_1_RADIUS ) // inside LIGHT 0 ZONE
                    .filter(m -> drone.pos.distanceTo(m.pos) >= drone.pos.distanceTo(m.pos.add(new Point(m.vx, m.vy)))) // going towards me
                    .collect(Collectors.toList());
            potentialDanger.forEach(m -> System.err.println("AVOID getTarget POTENTIAL id="+m.id));

            // select all monsters being globaly visible which could a potential danger in next turn
            List<Creature> otherPotential = monsters.stream()
                    .filter(m -> !absoluteDanger.contains(m))
                    .filter(m -> !potentialDanger.contains(m))
                    .collect(Collectors.toList());
            otherPotential.forEach(m -> System.err.println("AVOID getTarget OTHER id="+m.id));

            Set<Integer> additionnal = new HashSet<>();

            Point t = new Point(5000,5000);
            boolean run = true;
            while (run) {
                finished = false;
                Vector avoidVector = new Vector();

                if (!absoluteDanger.isEmpty()) {
                    // flew in opposite direction
                    Set<Integer> list = new HashSet<>();
                    list.addAll(absoluteDanger.stream().map(m -> m.id).toList());
                    list.addAll(additionnal);
                    Vector vec = new Vector();
                    list.stream()
                            .map(i -> creatures.get(i))
                            .forEach(m -> {
                                Point nextPos = m.pos.add(new Point(m.vx, m.vy));
                                double dist = drone.pos.distanceTo(nextPos);
                                Vector v = nextPos.vectorTo(drone.pos);
                                vec.x += v.x / (dist * dist);
                                vec.y += v.y / (dist * dist);
                                System.err.println("AVOID getTarget step 1 : monster.id=" + m.id + " dist=" + dist + " vec.x=" + vec.x + " vec.y=" + vec.y);
                            });
                    avoidVector = vec;
                } else if (!potentialDanger.isEmpty()) {
                    // flew by 90° minimum or direct to target
                    Set<Integer> list = new HashSet<>();
                    list.addAll(potentialDanger.stream().map(m -> m.id).toList());
                    list.addAll(additionnal);
                    System.err.println("AVOID getTarget step 2 : monsters.size=" + list.size());
                    Vector vec = new Vector();
                    list.stream()
                            .map(i -> creatures.get(i))
                            .forEach(m -> {
                                Point nextPos = m.pos.add(new Point(m.vx, m.vy));
                                double dist = drone.pos.distanceTo(nextPos);
                                Vector v = nextPos.vectorTo(drone.pos);
                                vec.x += v.x / (dist * dist);
                                vec.y += v.y / (dist * dist);
                                System.err.println("AVOID getTarget step 2 : drone.id=" + drone.id + " d.x="+drone.pos.x+" d.y="+drone.pos.y);
                                System.err.println("AVOID getTarget step 2 : monster.id=" + m.id + " m.x="+m.pos.x+" m.y="+m.pos.y+" dist=" + dist + " vec.x=" + vec.x + " vec.y=" + vec.y);
                            });

                    Point nextTarget = next.getTarget(drone);
                    avoidVector = drone.pos.vectorTo(nextTarget);
                    double a = vec.angleTo(avoidVector);
                    System.err.println("AVOID getTarget step 2 : avoid.x=" + avoidVector.x + " avoid.y=" + avoidVector.y + " a=" + a);
                    if (Math.abs(a) < Math.PI / 2) {
                        finished = true;
                        System.err.println("AVOID getTarget step 2 : direct route to target");
                    } else {
                        // turn by PI/2
                        // choose the +PI/2 or -PI/2 which result to be closer to target and not off the map
                        Point p1 = drone.pos.add(vec.rotate(Math.PI / 2d).normalizedDirection(MAX_MOVE));
                        Point p2 = drone.pos.add(vec.rotate(-Math.PI / 2d).normalizedDirection(MAX_MOVE));
                        if (p1.x < 0 || p1.x > 10000 || p1.y < 0 || p1.y > 10000) {
                            System.err.println("AVOID getTarget step 2 rotation 1 p.x=" + p2.x + " p.y=" + p2.y);
                            avoidVector = drone.pos.vectorTo(p2);
                        } else if (p2.x < 0 || p2.x > 10000 || p2.y < 0 || p2.y > 10000) {
                            System.err.println("AVOID getTarget step 2 rotation 2 p.x=" + p1.x + " p.y=" + p1.y);
                            avoidVector = drone.pos.vectorTo(p1);
                        } else if (p1.distanceTo(nextTarget) < p2.distanceTo(nextTarget)) {
                            System.err.println("AVOID getTarget step 2 rotation 3 p.x=" + p1.x + " p.y=" + p1.y);
                            avoidVector = drone.pos.vectorTo(p1);
                        } else {
                            System.err.println("AVOID getTarget step 2 rotation 4 p.x=" + p2.x + " p.y=" + p2.y);
                            avoidVector = drone.pos.vectorTo(p2);
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
                        .filter(m -> tmp.distanceTo(m.pos.add(new Point(m.vx, m.vy))) < KILL_DISTANCE)
                        .toList();
                killHits.forEach(m -> System.err.println("AVOID getTarget KILL HITS id=" + m.id));
                if (killHits.isEmpty()) {
                    run = false;
                } else {
                    additionnal.addAll(killHits.stream().map(m -> m.id).toList());
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
            // TODO
            //- return next.getTarget() if finished true
            //msg = "AVOID next=("+next.
            if(finished) msg = next.msg;
            return t;
        }

        @Override
        public boolean isLightOn(Drone drone) {
            boolean light = drone.pos.distanceTo(next.getTarget(drone))<3000; // only light if nearby our target (because if scanned, it can change the next one)
            drone.disableLightTurnCounter = light?0:2;
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
        boolean wasVisible = false;

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
            System.err.println("DRONE.process droneId="+this.id+" next="+next.getClass().getSimpleName());

            // handle monsters
            List<Creature> monsters = creatures.values().stream()
                    .filter(c -> c.visible || c.wasVisible) // keep only visible
                    .filter(c -> c.type == -1) // keep only monsters
                    .filter(c -> this.pos.distanceTo(c.pos) <= LIGHT_MONSTERS_EXTRA+LIGHT_1_RADIUS)
                    .toList();
            monsters.forEach(m -> System.err.println("MONSTERS drone="+this.id+" monster="+m.id));
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

            System.out.println("MOVE "+target.x+" "+target.y+" "+(light?1:0)+" "+next.msg);
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
            int myScore = in.nextInt();
            int foeScore = in.nextInt();
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
                creature.wasVisible = creature.visible;
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
            // update creatures which became not visible on last turn based on their last velocity vector
            creatures.values().stream()
                    .filter(c -> !c.visible)
                    .filter(c -> c.wasVisible)
                    .forEach(c -> c.pos.add(new Point(c.vx,c.vy)));

                /*creatures.values().forEach( c -> {
                    System.err.println("CREATURE id="+c.id+" x="+c.pos.x+" y="+c.pos.y);
                });*/

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