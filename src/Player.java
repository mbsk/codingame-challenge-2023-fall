    import java.util.*;
    import java.util.List;

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

        final static int LIGHT_ON_EVERY = 4;

        static int creatureCount;
        static Map<Integer, Creature> creatures = new HashMap<>();
        static Map<Integer, Drone> drones = new HashMap<>();

        enum Depth {
            L1(0,2500),L2(1,5000),L3(2,7500);
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

            public Vector add(Vector other) {
                Vector v = new Vector();
                v.x = this.x + other.x;
                v.y = this.y + other.y;
                return v;
            }

            public double size() {
                return Math.sqrt(Math.pow(this.x,2)+Math.pow(this.y,2));
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

            public boolean isLightOn(Drone drone) {
                return drone.turn%LIGHT_ON_EVERY==0;
            }

        }

        static class Dive extends Strategy {

            private final Depth depth;

            public Dive(Depth depth) {
                super(new Point(0, depth.middle));
                this.depth = depth;
            }

            public Point getTarget(Drone drone) {
                return new Point(drone.pos.x, target.y);
            }

            public boolean isFinished(Drone drone) {
                return drone.currentScan.stream().filter(i -> creatures.get(i).type == depth.type).count()>0;
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
        }

        static class Avoid extends Strategy {

            private final Set<Integer> monsters = new HashSet<>();
            private int step = Integer.MAX_VALUE;

            public Avoid(Point target, List<Creature> monsters) {
                super(target);
                addMonsters(monsters);
            }

            public void addMonsters(List<Creature> monsters) {
                monsters.forEach(m-> {
                    this.monsters.add(m.id);
                });
            }

            List<Integer> step1TargettingMe;
            Vector step1AvoidVector;

            @Override
            public Point getTarget(Drone drone) {
                System.err.println("AVOID getTarget droneId="+drone.id + " step="+step+" target.x="+target.x+" target.y="+target.y);
                /*List<Integer> targettingMe = monsters.stream()
                        .filter(i -> drone.pos.distanceTo(creatures.get(i).pos) <= (drone.light?LIGHT_1_RADIUS:LIGHT_0_RADIUS))
                        .toList();*/

                List<Integer> targettingMe = monsters.stream()
                        .map(i->creatures.get(i))
                        .filter(m -> drone.pos.distanceTo(m.pos.add(new Point(m.vx, m.vy))) < KILL_DISTANCE)
                        .map(m->m.id)
                        .toList();

                targettingMe.stream()
                        .map(i->creatures.get(i))
                        .forEach(m->System.err.println("AVOID targettingMe id="+m.id+" x="+m.pos.x+" y="+m.pos.y));

                Vector avoidVector = new Vector();

                /*if(!targettingMe.isEmpty()) {
                    step = 1;
                    System.err.println("AVOID getTarget step=1");
                    step1TargettingMe = targettingMe;
                    // flew in opposite direction
                    Vector vec = new Vector();
                    targettingMe.forEach(i -> {
                        Creature m = creatures.get(i);
                        double dist = drone.pos.distanceTo(m.pos);
                        vec.x += m.vx / dist;
                        vec.y += m.vy / dist;
                        System.err.println("AVOID getTarget step 1 : monster.id="+i+" dist="+dist+" vec.x="+vec.x+" vec.y="+vec.y);
                    });
                    avoidVector = vec;
                    step1AvoidVector = vec.copy();
                } else {
                    Set<Integer> processed = new HashSet<>();
                    if(step==1) {
                        step = 2;
                        processed.addAll(step1TargettingMe);
                        System.err.println("AVOID getTarget step=2 target.x="+target.x+" target.y="+target.y);
                        // turn by PI/2
                        // choose the +PI/2 or -PI/2 which result to be closer to target and not off the map
                        Point p1 = step1AvoidVector.rotate(Math.PI/2d).normalizedDirection(MAX_MOVE).add(drone.pos);
                        Point p2 = step1AvoidVector.rotate(-Math.PI/2d).normalizedDirection(MAX_MOVE).add(drone.pos);
                        //System.err.println("AVOID getTarget step=2 p1.x="+p1.x+" p1.y="+p1.y+" p2.x="+p2.x+" p2.y="+p2.y);
                        //System.err.println("AVOID getTarget step=2 dist1="+p1.distanceTo(target)+" dist2="+p2.distanceTo(target));
                        if(p1.distanceTo(target) < p2.distanceTo(target) && p1.x>=0 && p1.x<=10000 && p1.y>=0 && p1.y<=10000) {
                            System.err.println("AVOID getTarget step=2 rotation=+ p.x="+p1.x+" p.y="+p1.y);
                            avoidVector = drone.pos.vectorTo(p1);
                            //System.err.println("AVOID getTarget step=2 rotation=+ v.x="+avoidVector.x+" p.y="+avoidVector.y);
                        } else {
                            System.err.println("AVOID getTarget step=2 rotation=- p.x="+p2.x+" p.y="+p2.y);
                            avoidVector = drone.pos.vectorTo(p2);
                            //System.err.println("AVOID getTarget step=2 rotation=- v.x="+avoidVector.x+" v.y="+avoidVector.y);
                        }
                    } else if(step==2){
                        // go parrallel to the inital root
                        step=3;
                        System.err.println("AVOID getTarget step=3");
                        avoidVector.x = -step1AvoidVector.x;
                        avoidVector.y = -step1AvoidVector.y;
                    }

                    Vector vec = new Vector();
                    // avoid potential monsters going towards us (not yet in light)
                    monsters.stream()
                            .filter(i -> !processed.contains(i))
                            .map(i->creatures.get(i))
                            .filter(m -> drone.pos.distanceTo(m.pos.add(new Point(m.vx, m.vy))) < KILL_DISTANCE)
                            .forEach(m -> {
                                processed.add(m.id);
                                System.err.println("AVOID getTarget targettingMeNextTurn i="+m.id);
                                double dist = drone.pos.distanceTo(m.pos);
                                vec.x += m.vx / dist;
                                vec.y += m.vy / dist;
                            });
                    avoidVector.add(vec);
                    vec.reset();
                    System.err.println("AVOID getTarget step="+step+" v.x="+avoidVector.x+" v.y="+avoidVector.y);

                    // avoid monsters which have a trajectory convergent to us in next turn
                    Point nextPosition = avoidVector.normalizedDirection(MAX_MOVE).add(drone.pos);
                    monsters.stream()
                            .filter(i -> !processed.contains(i))
                            .map(i->creatures.get(i))
                            .filter(m -> nextPosition.distanceTo(m.pos.add(new Point(m.vx, m.vy))) < KILL_DISTANCE)
                            .forEach(m -> {
                                processed.add(m.id);
                                System.err.println("AVOID getTarget beingInKillZoneNextTurn i="+m.id);
                                Vector avoidMonsterVector = nextPosition.vectorTo(m.pos.add(new Point(m.vx, m.vy)));
                                double dist = avoidMonsterVector.size()*2; // *2 to diminuate this parameter influence
                                vec.x += avoidMonsterVector.x / dist;
                                vec.y += avoidMonsterVector.y / dist;
                            });
                    avoidVector.add(vec);
                    vec.reset();

                }*/
                if(targettingMe.size() > 1) {
                    step = 1;
                    System.err.println("AVOID getTarget step=1");
                    step1TargettingMe = targettingMe;
                    // flew in opposite direction
                    Vector vec = new Vector();
                    targettingMe.forEach(i -> {
                        Creature m = creatures.get(i);
                        double dist = drone.pos.distanceTo(m.pos);
                        vec.x += m.vx / dist;
                        vec.y += m.vy / dist;
                        System.err.println("AVOID getTarget step 1 : monster.id="+i+" dist="+dist+" vec.x="+vec.x+" vec.y="+vec.y);
                    });
                    avoidVector = vec;
                    step1AvoidVector = vec.copy();
                }



                if(avoidVector.size() == 0) {
                    avoidVector = drone.pos.vectorTo(target);
                }
                return avoidVector.normalizedDirection(MAX_MOVE).add(drone.pos);
            }

            @Override
            public boolean isLightOn(Drone drone) {
                drone.turn = 1; // reset turn to avoid light for several turns after avoidance finished
                return false;
            }

            @Override
            public boolean isFinished(Drone drone) {
                return step>=3;
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

            int getScore() {
                if(myScan) return 0;
                int score = type+1;
                score = (foeScan)?score:score*2;
                return score;
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
            private int turn = 0;
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

            public void process() {
                if(emergency==1) {
                    strategy.clear();
                    System.out.println("WAIT 0");
                    return;
                }
                turn=(turn+1)%LIGHT_ON_EVERY;
                if(initPosition == -1) {
                    initPosition = this.pos.x<5000?0:1;
                }

                // handle creature scan
                creatures.values().stream()
                        .filter(c -> c.visible) // keep only visible
                        .filter(c -> !c.myScan) // remove if already validated scan
                        .filter(c -> c.type != -1) // remove monsters
                        .filter(c -> this.pos.distanceTo(c.pos) <= (this.light?LIGHT_1_RADIUS:LIGHT_0_RADIUS))
                        .forEach(c -> currentScan.add(c.id));

                Strategy next = peekNextMission();
                if(next.isFinished(this)) {
                    strategy.poll();
                    next = peekNextMission();
                }
                System.err.println("DRONE.process droneId="+this.id+" next="+next.getClass().getSimpleName());

                // handle monsters
                List<Creature> monsters = creatures.values().stream()
                        .filter(c -> c.visible) // keep only visible
                        .filter(c -> c.type == -1) // keep only monsters
                        .filter(c -> this.pos.distanceTo(c.pos) <= LIGHT_MONSTERS_EXTRA+(this.light?LIGHT_1_RADIUS:LIGHT_0_RADIUS))
                        .toList();
                monsters.forEach(m -> System.err.println("MONSTERS drone="+this.id+" monster="+m.id));
                // monster avoidance management
                if(!monsters.isEmpty()) {
                    if(next instanceof Avoid) {
                        ((Avoid) next).addMonsters(monsters);
                    } else {
                        next = new Avoid(this.pos.copy(), next.getTarget(this), monsters);
                        strategy.push(next);
                    }
                }

                // get next waypoint
                Point target = next.getTarget(this);

                // light management
                light = this.pos.y > 2000 && battery>=5 && next.isLightOn(this);

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
                    drone.currentScan.add(creatureId);
                }
                creatures.values().forEach(creature -> creature.visible=false);
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
                // update not visible creatures based on their last velocity vector
                creatures.values().stream()
                        .filter(c -> !c.visible)
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