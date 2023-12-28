    import java.util.*;
    import java.util.List;
    import java.util.stream.Collectors;

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

        static int creatureCount;
        static Map<Integer, Creature> creatures = new HashMap<>();
        static Map<Integer, Drone> drones = new HashMap<>();

        static int turn = 0;

        enum Depth {
            L0(-10,0), L1(0,2500),L2(1,5000),L3(2,7500);
            public final int type;
            public final int start;
            public final int middle;
            Depth(int type, int start) {
                this.type = type;
                this.start = start;
                this.middle = start+1250;
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

        static abstract class Strategy {
            protected final Point target;

            public Strategy(Point target) {
                this.target = target;
            }

            public boolean isFinished(Drone drone) {
                return drone.pos.distanceTo(target) < 5;
            }

            public Point getTarget(Drone drone) {
                return target;
            }

            public abstract boolean isLightOn(Drone drone);

        }

        static class Dive extends Strategy {

            private Depth depth;
            private final int startTurn;

            public Dive(Depth depth) {
                super(new Point(-1, depth.middle));
                this.depth = depth;
                this.startTurn = turn;
            }

            public Point getTarget(Drone drone) {

                List<Creature> notScanned = new ArrayList<>();

                boolean run = true;
                while(run && depth!=Depth.L0) {
                    notScanned = drone.getCreaturesByRadar().stream()
                            .filter(c->c.type!=-1)
                            .filter(c->!c.myScan)
                            .filter(c->depth.type==c.type)
                            .toList();
                    run = notScanned.size() == 0;
                    if(run) {
                        if(depth == Depth.L3) depth=Depth.L2;
                        else if(depth == Depth.L2) depth=Depth.L1;
                        else depth = Depth.L0;
                    }
                }

                //notScanned.forEach(c -> System.err.println("DIVE NOT SCANNED c.id="+c.id+" score="+c.getScore()));

                long leftCount = notScanned.stream()
                        .filter(c->drone.radars.get("BL").contains(c.id)||drone.radars.get("TL").contains(c.id))
                        .mapToInt(c->c.foeScan?1:2)// prioritize those which were not scanned by the other
                        .sum();

                long rightCount = notScanned.stream()
                        .filter(c->drone.radars.get("BR").contains(c.id)||drone.radars.get("TR").contains(c.id))
                        .mapToInt(c->c.foeScan?1:2)// prioritize those which were not scanned by the other
                        .sum();

                int deviation = 0;
                if(rightCount>leftCount) deviation = 1;
                else if(rightCount<leftCount) deviation = -1;
                else {
                    deviation = drone.pos.x<5000?-1:1;
                }

                long x = 5000;
                if(drone.pos.y < depth.start) {
                    x = drone.pos.x<=5000?2500:7500;
                } else if(target.x == -1) {
                    target.x = deviation>0?9000:1000;
                    x = target.x;
                } else {
                    if(leftCount==0 && target.x <= 5000) target.x = 9000;
                    else if(rightCount==0 && target.x >= 5000) target.x = 1000;
                    x = target.x;
                }

                System.err.println("DIVE R="+rightCount+ " L="+leftCount+" target.x="+target.x);

                return new Point(x, depth.middle);
            }

            public boolean isFinished(Drone drone) {

                long remainingScanCount = drone.getCreaturesByRadar().stream()
                        .filter(c->c.type!=-1)
                        .filter(c->depth.type==c.type)
                        .filter(c->!c.myScan)
                        .filter(c -> drones.values().stream().filter(d->d.mine).noneMatch(d->d.currentScan.contains(c.id)))
                        .map(c->c.id)
                        .distinct()
                        .count();

                long scannedCount = drone.currentScan.stream().filter(i -> creatures.get(i).type == depth.type).count();
                long score = drones.values().stream().filter(d->d.mine).mapToInt(d->d.currentScan.stream().map(i->creatures.get(i)).mapToInt(Creature::getScore).sum()).sum();

                System.err.println("DIVE isFINISHED score="+score+" allCurrentScan="+drone.currentScan.size()+ " type="+depth.type+" scanCount="+scannedCount+" remaining="+remainingScanCount);

                return depth == null
                        || score > 20
                        || (drone.currentScan.size()>0 && turn-startTurn > 32)
                        || (scannedCount>0 && remainingScanCount <= 1);
            }

            @Override
            public boolean isLightOn(Drone drone) {
                System.err.println("DIVE isLightOn y="+drone.pos.y+" turn="+turn);
                return drone.pos.y>2000 && turn%DIVE_LIGHT_ON_EVERY==0;
            }
        }

        static class Surface extends Strategy {

            public Surface() {
                super(new Point(0, DEPTH_SURFACE));
            }

            @Override
            public Point getTarget(Drone drone) {
                return new Point(drone.pos.x, target.y);
            }

            @Override
            public boolean isFinished(Drone drone) {
                return drone.pos.y <= 500;
            }

            @Override
            public boolean isLightOn(Drone drone) {
                return turn%DIVE_LIGHT_ON_EVERY==0 && drone.pos.y <= Depth.L2.middle;
            }
        }

        static class Avoid extends Strategy {

            private Strategy next;
            private boolean finished = false;

            public Avoid(Strategy next) {
                super(next.target);
                this.next = next;
            }


            @Override
            public Point getTarget(Drone drone) {
                System.err.println("AVOID getTarget droneId="+drone.id);

                // select monsters type going towards us
                List<Creature> monsters = creatures.values().stream()
                        .filter(m -> m.type == -1 && (m.visible || m.wasVisible))
                        .collect(Collectors.toList()); // collect to a modifiable List

                // add "fake" monsters to avoid borders
                List<BorderAvoid> borders = BorderAvoid.getBorderAvoid(drone);
                //monsters.addAll(borders);
                borders.forEach(b->creatures.put(b.id,b));

                // select all monsters inside 800u
                List<Creature> absoluteDanger = monsters.stream()
                        .filter(m -> drone.pos.distanceTo(m.pos) <= 800 ) // inside LIGHT 0 ZONE
                        .filter(m -> drone.pos.distanceTo(m.pos) >= drone.pos.distanceTo(m.pos.add(new Point(m.vx, m.vy)))) // going towards me
                        .toList();
                absoluteDanger.forEach(m -> System.err.println("AVOID getTarget ABSOLUTEDANGER id="+m.id));

                // select monsters who are under our light but not closer to 800
                List<Creature> potentialDanger = monsters.stream()
                        .filter(m -> !absoluteDanger.contains(m))
                        .filter(m -> drone.pos.distanceTo(m.pos) <= LIGHT_MONSTERS_EXTRA+LIGHT_1_RADIUS ) // inside LIGHT 0 ZONE
                        .filter(m -> drone.pos.distanceTo(m.pos) >= drone.pos.distanceTo(m.pos.add(new Point(m.vx, m.vy)))) // going towards me
                        .toList();
                potentialDanger.forEach(m -> System.err.println("AVOID getTarget POTENTIAL id="+m.id));

                // select all monsters being globaly visible which could a potential danger in next turn
                List<Creature> otherPotential = monsters.stream()
                        .filter(m -> !absoluteDanger.contains(m))
                        .filter(m -> !potentialDanger.contains(m))
                        .toList();
                otherPotential.forEach(m -> System.err.println("AVOID getTarget OTHER id="+m.id));

                Set<Integer> additionnal = new HashSet<>();

                Point t = new Point(5000,5000);
                boolean run = true;
                while (run) {
                    finished = false;
                    Vector avoidVector = new Vector();

                    if(!absoluteDanger.isEmpty()) {
                        // flew in opposite direction
                        Set<Integer> list = new HashSet<>();
                        list.addAll(absoluteDanger.stream().map(m->m.id).toList());
                        list.addAll(additionnal);
                        Vector vec = new Vector();
                        list.stream()
                                .map(i -> creatures.get(i))
                                .forEach(m -> {
                                    Point nextPos = m.pos.add(new Point(m.vx, m.vy));
                                    double dist = drone.pos.distanceTo(nextPos);
                                    Vector v = nextPos.vectorTo(drone.pos);
                                    vec.x += v.x / (dist*dist);
                                    vec.y += v.y / (dist*dist);
                                    System.err.println("AVOID getTarget step 1 : monster.id="+m.id+" dist="+dist+" vec.x="+vec.x+" vec.y="+vec.y);
                                });
                        avoidVector = vec;
                    } else if(!potentialDanger.isEmpty()) {
                        // flew by 90° minimum or direct to target
                        Set<Integer> list = new HashSet<>();
                        list.addAll(potentialDanger.stream().map(m->m.id).toList());
                        list.addAll(additionnal);
                        System.err.println("AVOID getTarget step 2 : monsters.size="+list.size());
                        Vector vec = new Vector();
                        list.stream()
                                .map(i->creatures.get(i))
                                .forEach(m -> {
                                    Point nextPos = m.pos.add(new Point(m.vx, m.vy));
                                    double dist = drone.pos.distanceTo(nextPos);
                                    Vector v = nextPos.vectorTo(drone.pos);
                                    vec.x += v.x / (dist*dist);
                                    vec.y += v.y / (dist*dist);
                                    System.err.println("AVOID getTarget step 2 : monster.id="+m.id+" dist="+dist+" vec.x="+vec.x+" vec.y="+vec.y);
                                });

                        avoidVector = drone.pos.vectorTo(next.getTarget(drone));
                        double a = vec.angleTo(avoidVector);
                        System.err.println("AVOID getTarget step 2 : avoid.x="+avoidVector.x+" avoid.y="+avoidVector.y+" a="+a);
                        if(Math.abs(a)<Math.PI/2) {
                            finished = true;
                            System.err.println("AVOID getTarget step 2 : direct route to target");
                        } else {
                            // turn by PI/2
                            // choose the +PI/2 or -PI/2 which result to be closer to target and not off the map
                            Point p1 = vec.rotate(Math.PI/2d).normalizedDirection(MAX_MOVE).add(drone.pos);
                            Point p2 = vec.rotate(-Math.PI/2d).normalizedDirection(MAX_MOVE).add(drone.pos);
                            if(p1.distanceTo(next.getTarget(drone)) < p2.distanceTo(next.getTarget(drone)) && p1.x>=0 && p1.x<=10000 && p1.y>=0 && p1.y<=10000) {
                                System.err.println("AVOID getTarget step 2 rotation=+ p.x="+p1.x+" p.y="+p1.y);
                                avoidVector = drone.pos.vectorTo(p1);
                            } else {
                                System.err.println("AVOID getTarget step 2 rotation=- p.x="+p2.x+" p.y="+p2.y);
                                avoidVector = drone.pos.vectorTo(p2);
                            }
                        }
                    } else {
                        finished = true;
                    }

                    if(avoidVector.size() == 0) {
                        avoidVector = drone.pos.vectorTo(next.getTarget(drone));
                    }
                    Point tmp = avoidVector.normalizedDirection(MAX_MOVE).add(drone.pos);
                    t = tmp;

                    // check whether there is a potential kill
                    List<Creature> killHits = otherPotential.stream()
                            .filter(m -> !additionnal.contains(m.id))
                            .filter(m -> tmp.distanceTo(m.pos.add(new Point(m.vx, m.vy)))<KILL_DISTANCE)
                            .toList();
                    killHits.forEach(m -> System.err.println("AVOID getTarget KILL HITS id="+m.id));
                    if(killHits.isEmpty()) {
                        run = false;
                    } else {
                        additionnal.addAll(killHits.stream().map(m->m.id).toList());
                    }

                }
                borders.forEach(b->creatures.remove(b.id));
                return t;
            }

            @Override
            public boolean isLightOn(Drone drone) {
                drone.disableLightTurnCounter = 2;
                return false;
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

            int getScore() {
                if(myScan) return 0;
                int score = type+1;
                score = (foeScan)?score:score*2;
                return score;
            }

        }

        static class BorderAvoid extends Creature {

            private BorderAvoid(int id, long x, long y) {
                this.id = id;
                this.pos.x = x;
                this.pos.y = y;
                this.visible = true;
                this.type = -1;
            }
            static List<BorderAvoid> getBorderAvoid(Drone drone) {
                List<BorderAvoid> borders = new ArrayList<>();
                borders.add(new BorderAvoid(1000,0,drone.pos.y));
                borders.add(new BorderAvoid(1001,10000,drone.pos.y));
                borders.add(new BorderAvoid(1002,drone.pos.x,10000));
                return borders;
            }
        }

        static class Drone {
            int id;
            boolean mine = false;
            Point pos = new Point(0,0);
            int emergency;
            int battery;
            final Deque<Strategy> strategy = new ArrayDeque<>();
            int initPosition = -1;
            int disableLightTurnCounter = 0;

            boolean light = false;
            Set<Integer> currentScan = new HashSet<>();
            Map<String, Set<Integer>> radars = new HashMap<>();

            public Drone() {
                radars.put("BL", new HashSet<>());
                radars.put("BR", new HashSet<>());
                radars.put("TL", new HashSet<>());
                radars.put("TR", new HashSet<>());
            }

            private Strategy peekNextMission() {
                if(strategy.isEmpty()) {
                    if(initPosition%2==0) {
                        strategy.add(new Dive(Depth.L3));
                        strategy.add(new Surface());
                    } else {
                        strategy.add(new Dive(Depth.L3));
                        strategy.add(new Surface());
                    }
                }
                return strategy.peek();
            }

            void resetRadar() {
                radars.values().stream().forEach(Set::clear);
            }

            List<Creature> getCreaturesByRadar() {
                List<Creature> all = new ArrayList<>();
                radars.values().forEach(r->r.stream().map(i->creatures.get(i)).forEach(c->all.add(c)));
                return all;
            }

            public void process() {
                if(emergency==1) {
                    strategy.clear();
                    System.out.println("WAIT 0");
                    return;
                }
                disableLightTurnCounter--;
                if(initPosition == -1) {
                    initPosition = this.pos.x<5000?0:1;
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

                System.out.println("MOVE "+target.x+" "+target.y+" "+(light?1:0));
            }
        }

        public static void main(String args[]) {
            Scanner in = new Scanner(System.in);
            creatureCount = in.nextInt();
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
                // update creatures which became not visibale on last turn based on their last velocity vector
                creatures.values().stream()
                        .filter(c -> !c.visible)
                        .filter(c -> c.wasVisible)
                        .forEach(c -> c.pos.add(new Point(c.vx,c.vy)));

                creatures.values().forEach( c -> {
                    //System.err.println("CREATURE id="+c.id+" x="+c.pos.x+" y="+c.pos.y);
                });

                int radarBlipCount = in.nextInt();
                drones.values().stream().forEach(Drone::resetRadar);
                for (int i = 0; i < radarBlipCount; i++) {
                    int droneId = in.nextInt();
                    int creatureId = in.nextInt();
                    String radar = in.next();
                    drones.get(droneId).radars.get(radar).add(creatureId);
                    //System.err.println("RADAR droneId="+droneId+" creatureId="+creatureId+" radar="+radar);
                }
                drones.values().stream().filter(c->c.mine).forEach(Drone::process);
            }
        }
    }