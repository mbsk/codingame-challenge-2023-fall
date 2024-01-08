    import java.util.*;
    import java.util.List;
    import java.util.function.Predicate;
    import java.util.stream.Collectors;
    import java.util.stream.IntStream;
    import java.util.stream.Stream;

    /**
     * Score points by scanning valuable fish faster than your opponent.
     **/
    class Player {

        final static int KILL_DISTANCE = 500;
        final static int MAX_MOVE = 600;
        final static int DEPTH_SURFACE = 495;
        final static int LIGHT_0_RADIUS = 800;
        final static int LIGHT_1_RADIUS = 2000;
        final static int LIGHT_MONSTERS_EXTRA = 300;

        final static int LIGHT_ON_EVERY = 2;

        /** Holds all creatures */
        static Map<Integer, Creature> creatures = new HashMap<>();
        /** Holds all drones */
        static Map<Integer, Drone> drones = new HashMap<>();
        /** Holds creatures position based on fused data from our drones radar */
        static CommonOperationPicture cop = new CommonOperationPicture();

        static int turn = 0;
        static int myScore = 0;
        static int foeScore = 0;
        static boolean isGloballyFinished=false;

        /**
         * pre calculate 72 azimuths vectors, for monsters escape
         */
        static List<Point> azimuths;
        static   {
            Vector unitVect =  new Vector();
            unitVect.x = 1;
            unitVect.y = 0;
            int azimuthSize = 72;
            azimuths = IntStream.range(0,azimuthSize).mapToObj(a->unitVect.rotate(2d*(double)a*Math.PI/azimuthSize).normalizedDirection(MAX_MOVE)).toList(); // calculate once for performance
        }

        /**
         * centralized log
         */
        static long time = 0;
        static void log(String s) {
            System.err.println((System.currentTimeMillis()-time)+" : "+s);
        }

        /**
         * Represents the layers fishes belongs to
         */
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
                this.end = end;
                this.middle = (this.end-this.start)/2;
            }

            static Depth getDepth(long depth) {
                return Stream.of(Depth.values()).filter(d->depth>=d.start && depth<=d.end).findFirst().orElse(null);
            }

        }

        /**
         * A 2D point representation
         */
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

        /**
         * A 2D Vector representation
         * */
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

        /**
         * Merge our drone radars, the position is more or less precise :
         * - if based only radars, middle of one of bounding cells delimitted by our drone (+ creatures depth limit)
         * - if visible : exact positions
         * - if crosses a cell boundaries : middle of boundary
         * - maintain position 4 turns after eing visible (either exactly or boudary crossing)
         */
        static class CommonOperationPicture {
            Map<Integer, RadarPlot> radar = new HashMap<>();
            long[] xPositions;
            long[] yPositions;

            public void update() {
                Map<Integer, RadarPlot> newradar = new HashMap<>();

                // get only my drones
                List<Drone> myDrones = drones.values().stream().filter(d->d.mine).toList();

                // order by x
                List<Drone> hOrderedDrones = myDrones.stream().sorted(Comparator.comparing(d -> d.pos.x)).toList();
                long[] newxPositions = new long[hOrderedDrones.size()];
                for (int i = 0; i < hOrderedDrones.size(); i++) {
                    final int order = i; // needed a variable "effectively" final to be used whithin streams
                    hOrderedDrones.get(i).radars.entrySet().stream()
                            .filter(e->"BL".equals(e.getKey()) || "TL".equals(e.getKey()))
                            .map(Map.Entry::getValue)
                            .forEach(j->this.updatePlot(j, order, true, newradar));
                    hOrderedDrones.get(i).radars.entrySet().stream()
                            .filter(e->"BR".equals(e.getKey()) || "TR".equals(e.getKey()))
                            .map(Map.Entry::getValue)
                            .forEach(j->this.updatePlot(j, order+1, true, newradar));
                    newxPositions[i] = hOrderedDrones.get(i).pos.x;
                }


                // order by y
                List<Drone> vOrderedDrones = myDrones.stream().sorted(Comparator.comparing(d -> d.pos.y)).toList();
                long[] newyPositions = new long[vOrderedDrones.size()];
                for (int i = 0; i < vOrderedDrones.size(); i++) {
                    final int order = i; // needed a variable "effectively" final to be used whithin streams
                    vOrderedDrones.get(i).radars.entrySet().stream()
                            .filter(e->"TL".equals(e.getKey()) || "TR".equals(e.getKey()))
                            .map(Map.Entry::getValue)
                            .forEach(j->this.updatePlot(j, order, false, newradar));
                    vOrderedDrones.get(i).radars.entrySet().stream()
                            .filter(e->"BL".equals(e.getKey()) || "BR".equals(e.getKey()))
                            .map(Map.Entry::getValue)
                            .forEach(j->this.updatePlot(j, order+1, false, newradar));
                    newyPositions[i] = vOrderedDrones.get(i).pos.y;
                }

                // calibrate col and line (make them 0 index)
                int oversize = IntStream.range(0, myDrones.size()).sum();
                newradar.values().forEach(p->{
                    p.col-=oversize;
                    p.line-=oversize;
                });

                //_radar.values().forEach(p->log("COP id="+p.id+" (c,l)="+p.col+","+p.line+" x="+creatures.get(p.id).pos.x+" y="+creatures.get(p.id).pos.y));

                // update creature approximate position (for none visible)
                // center of boundingbox + be more precise if change radar cell horizontally or vertically
                newradar.values().forEach(p-> {
                    Creature c = creatures.get(p.id);
                    p.lineWasVisible--;
                    p.colWasVisible--;
                    c.wasVisible--;
                    //log("COP UPDATE id="+p.id+" col="+p.col+" lin="+p.line+" wasC="+p.colWasVisible+" wasL="+p.lineWasVisible);
                    if(!c.visible && c.wasVisible<=0) {
                        Depth depth = Depth.BY_TYPE.get(c.type);
                        long xmin=p.bb.xmin,xmax=p.bb.xmax,ymin=p.bb.ymin,ymax=p.bb.ymax;
                        if(p.colWasVisible<=0) {
                            xmin = p.col==0?0:newxPositions[p.col-1];
                            xmax = p.col==myDrones.size()?10000:newxPositions[p.col];
                            c.pos.x = Math.round((float) (xmax + xmin) /2);
                        }
                        if(p.lineWasVisible<=0) {
                            ymin = p.line==0?0:newyPositions[p.line-1];
                            ymax = p.line==myDrones.size()?10000:newyPositions[p.line];
                            ymin = Math.max(ymin,depth.start);
                            ymax = Math.min(ymax,depth.end);
                            c.pos.y = Math.round((float) (ymax + ymin) /2);
                        }
                        RadarPlot previousPlot = radar.get(p.id);
                        if(previousPlot!=null && previousPlot.col!=p.col) {
                            // plot switched zone horizontally : we can be more precise for x
                            int i = p.col<previousPlot.col?p.col:p.col-1;
                            p.colWasVisible = 4;
                            c.pos.x = Math.round((float) (newxPositions[i] + xPositions[i]) /2);
                            xmin = Math.min(xPositions[i], newxPositions[i]);
                            xmax = Math.max(xPositions[i], newxPositions[i]);
                        }
                        if(previousPlot!=null && previousPlot.line!=p.line) {
                            // plot switched zone vertically : we can be more precise for y
                            int i = p.line<previousPlot.line?p.line:p.line-1;
                            p.lineWasVisible = 4;
                            long y1 = Math.min(Math.max(newyPositions[i], depth.start), depth.end);
                            long y2 = Math.min(Math.max(yPositions[i], depth.start), depth.end);
                            c.pos.y = Math.round((float) (y1+y2) /2);
                            ymin = Math.min(y1, y2);
                            ymax = Math.max(y1, y2);
                        }
                        p.bb = new BoundingBox(xmin, xmax, ymin, ymax);
                    } else {
                        p.bb = new BoundingBox(c.pos.x, c.pos.x, c.pos.y, c.pos.y);
                    }
                });

                this.radar = newradar;
                this.xPositions = newxPositions;
                this.yPositions = newyPositions;

                //radar.values().stream().sorted(Comparator.comparing(p->p.id)).forEach(p->log("COP id="+p.id+" (c,l)="+p.col+","+p.line+" x="+creatures.get(p.id).pos.x+" y="+creatures.get(p.id).pos.y
                 //       +" vis="+creatures.get(p.id).visible+" was="+creatures.get(p.id).wasVisible + " (wasC, wasL)="+p.colWasVisible+","+p.lineWasVisible));
            }

            private void updatePlot(Set<Integer> indexes, int order, boolean isCol, Map<Integer, RadarPlot> radar) {
                indexes.stream()
                        .map(creatures::get)
                        .forEach(c-> {
                            radar.putIfAbsent(c.id, new RadarPlot());
                            RadarPlot plot = radar.get(c.id);
                            plot.id = c.id;
                            RadarPlot previousPlot = this.radar.get(c.id);
                            if(previousPlot!=null) {
                                plot.colWasVisible = previousPlot.colWasVisible;
                                plot.lineWasVisible = previousPlot.lineWasVisible;
                                plot.bb = previousPlot.bb;
                            }
                            if(isCol) plot.col+=order;
                            else plot.line+=order;
                        });
            }

            public List<Creature> getRemainingCreatures() {
                return radar.values().stream().map(p->creatures.get(p.id)).toList();
            }

            public Point getMaxPointToTarget(Drone drone, Creature target) {
                BoundingBox bb = getBoundingBox(target);
                Point p1 = new Point(bb.xmin,bb.ymin);
                Point p2 = new Point(bb.xmin,bb.ymax);
                Point p3 = new Point(bb.xmax,bb.ymin);
                Point p4 = new Point(bb.xmax,bb.ymax);
                return Stream.of(p1,p2,p3,p4).max(Comparator.comparing(p -> drone.pos.distanceTo(p))).orElse(null);
            }

            private BoundingBox getBoundingBox(Creature target) {
                RadarPlot plot = radar.get(target.id);
                Depth depth = Depth.BY_TYPE.get(target.type);
                long xmin,xmax,ymin,ymax;
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
                return new BoundingBox(xmin, xmax, ymin, ymax);
            }

            public long getFoeRemainingCreaturesCount() {
                return getRemainingCreatures().stream().filter(c->c.type!=-1 && !c.foeScan).count();
            }

            private record BoundingBox(long xmin, long xmax, long ymin, long ymax) {}

        }

        /**
         * Represents a radar position
         */
        static class RadarPlot {
            int id;
            int col;
            int line;
            int colWasVisible=0;
            int lineWasVisible=0;
            CommonOperationPicture.BoundingBox bb= new CommonOperationPicture.BoundingBox(0,0,0,0);
        }

        /**
         * Drones executes missions, a stack of missions is the strategy
         * At the bottom of the stack is always the mission "Surface"
         * Missions are pushed/polled from/to the stack according to events
         * The game starts with a strategy queue filled with Surface and Dive
         * 4 missions were implemented :
         * - Surface : go to the Surface as fast as possible to score scanned creatures
         * - Dive : dive to scan creatures
         * - Avoid : if nearby a monster, handles escape trajectory
         * - Bouh : eject a creatures from the map
         */
        static abstract class Mission {
            // optionnal target Creature of the Mission
            Creature target = null;
            // x,y target of the mission
            Point destination = null;
            // msg to be displayed by the drone
            String msg = "";

            public abstract boolean isFinished(Drone drone);
            protected abstract void updateDestination(Drone drone);
            public abstract boolean isLightOn(Drone drone);

            protected void updateTarget(Drone d, Creature nextTarget) {
                log("updateTarget d.id="+d.id+" mission="+this.getClass().getSimpleName()+" current="+((target==null)?"NULL":target.id)+" next="+((nextTarget==null)?"NULL":nextTarget.id));
                if(nextTarget!=this.target) {
                    updateDestination(d);
                }
            }
        }

        /**
         * Goal : dive to scan creatures
         *
         * algorithm :
         * if currentScan is enough to win => go to surface
         * if need only one more creatures to win => list all of them and found the closest drone to scan it, all other are instructed to surface
         * if need only creatures from above depth to win => switch creatures to scan to that depth
         * otherwise look for closest creatures at deepest depth available
         * (on first dive, try to scan most left/right of each layer in order to maximize chance to eject them from the map)
         */
        static class Dive extends Mission {

            public void updateDestination(Drone drone) {
                log("======= DIVE updateDestination drone="+drone.id +" forceToSurface="+drone.diveToSurface);

                if((drone.forceToSurface || drone.diveToSurface) && drone.currentScan.isEmpty()) {
                    drone.diveToSurface = false;
                    drone.forceToSurface = false;
                }

                if(drone.forceToSurface || (!isGloballyFinished && canWinWithCurrentScan() && !drone.currentScan.isEmpty())) {
                    Surface s = new Surface();
                    s.updateDestination(drone);
                    this.target = s.target;
                    this.destination = s.destination;
                    drone.forceToSurface = true;
                } else {
                    // list creatures not yet scored by me
                    final List<Creature> toScan = cop.getRemainingCreatures().stream()
                            .filter(c->c.type!=-1)
                            .filter(c->!c.myScan)
                            .collect(Collectors.toList()); // collect to modifiable collection

                    // list of my drones
                    final List<Drone> myDrones = drones.values().stream().filter(d->d.mine).toList();
                    // ignore creatures currently scanned by our drones
                    myDrones.forEach(d-> toScan.removeAll(d.currentScan.stream().map(creatures::get).toList()));

                    // if our current target was scored, change it
                    if(target!=null && !toScan.contains(target)) target = null;

                    // find deepestType of fishes not yet scanned
                    int deepestType = toScan.stream().mapToInt(c->c.type).filter(i->i!=-1).max().orElse(-1);

                    if(deepestType<0) {
                        // nothing to scan : finished scannin, can we Bouh something ?
                        target = null;
                        destination = null;
                        log("!!!!!!! FINISHED !!!!!!!!!!! ");
                    } else {
                        // can we win if we score all current scans as first and one of the remaining creature to scan ?
                        List<Creature> oneLeftToWin = toScan.stream().filter(c->canWinWithCurrentScanAnd(c.id)).toList();

                        if(!oneLeftToWin.isEmpty() && !drone.currentScan.isEmpty()) {
                            // find the best combinaison (drone,target) to scan the remaining creature, all pther will go to Surface
                            class BestCombinaison {
                                Creature target = null;
                                Drone drone = null;
                                double distance = Double.MAX_VALUE;
                            }

                            BestCombinaison bc = new BestCombinaison();
                            myDrones.stream().filter(d-> {
                                            Mission mission = d.peekMission();
                                            return !d.currentScan.isEmpty() && (mission instanceof Dive
                                                        || (mission instanceof Avoid && ((Avoid) mission).next instanceof Dive));
                                        })
                                        .forEach(d-> oneLeftToWin.forEach(c-> {
                                            double dist = 0;
                                            dist += myDrones.stream().filter(dd -> dd.id != d.id).mapToLong(dd -> dd.pos.y).sum(); // all others drones going straight to surface
                                            dist += d.pos.distanceTo(c.pos); // add this drone going to scan the creature
                                            dist += c.pos.y; // add this creature depth in order to the drone to go surface
                                            if (dist < bc.distance) {
                                                bc.distance = dist;
                                                bc.drone = d;
                                                bc.target = c;
                                            }
                                        })
                            );

                            // if a combination was found, apply it,
                            if(bc.drone!= null) {
                                myDrones.stream().filter(d-> d.peekMission() instanceof Dive || (d.peekMission() instanceof Avoid && ((Avoid) d.peekMission()).next instanceof Dive))
                                        .forEach(d-> {
                                    if(bc.drone.id == d.id || d.currentScan.isEmpty()) {
                                        d.diveToSurface = false;
                                        assert d.strategy.peek() != null;
                                        if(d.id==drone.id || d.currentScan.isEmpty()) {
                                            target = bc.target;
                                        } else {
                                            d.strategy.peek().updateTarget(d,bc.target);
                                        }
                                    } else {
                                        d.diveToSurface = true;
                                        assert d.strategy.peek() != null;
                                        d.strategy.peek().destination = null;
                                        d.strategy.peek().target = null;
                                    }
                                });
                            }

                            log(">>> DIVE DEST drone="+drone.id+" ONELEFT "+((target==null)?"NULL":target.id) +" diveToSurface="+drone.diveToSurface);
                        }
                        else {
                            // if there are still 4 creatures at L3 dive straight down (it means it's the first dive in the game)
                            boolean diveStraightToL3 = deepestType==Depth.L3.type && toScan.stream().filter(c->c.type==deepestType).count()==4;
                            if(diveStraightToL3 && drone.pos.y < Depth.L2.start) {
                                target = null;
                                Vector vec = new Vector();
                                if(drone.pos.y < Depth.L1.start) {
                                    List<Creature> list = cop.getRemainingCreatures().stream().filter(c->c.type==Depth.L1.type).toList();
                                    Creature targeted = drone.initPosition<2?list.stream().min(Comparator.comparing(c->c.pos.x)).orElse(null):list.stream().max(Comparator.comparing(c->c.pos.x)).orElse(null);
                                    if(targeted!=null) vec = drone.pos.vectorTo(new Point(targeted.pos.x+(drone.initPosition<2?500:-500), Depth.L1.end));
                                } else {
                                    List<Creature> list = cop.getRemainingCreatures().stream().filter(c->c.type==Depth.L2.type).toList();
                                    Creature targeted = drone.initPosition<2?list.stream().min(Comparator.comparing(c->c.pos.x)).orElse(null):list.stream().max(Comparator.comparing(c->c.pos.x)).orElse(null);
                                    if(targeted!=null) vec = drone.pos.vectorTo(new Point(targeted.pos.x+(drone.initPosition<2?500:-500), Depth.L2.end));
                                }
                                if(vec.size()==0) destination = new Point(drone.pos.x, 10000);
                                else destination = drone.pos.add(vec.normalizedDirection(MAX_MOVE));
                                log(">>> DEST drone="+drone.id+" FIRST DESCENT");
                            }
                            else {
                                log("normal case");
                                Depth maxDepth = Depth.L3;
                                if(!isGloballyFinished && !drone.currentScan.isEmpty()) {
                                    boolean canWinWithL1L2 = canWinWithCurrentScanAndAbove(Depth.L2);
                                    List<Drone> dronesOnDive = myDrones.stream().filter(d -> !d.diveToSurface && !d.currentScan.isEmpty()).toList();
                                    if(canWinWithL1L2) {
                                        log("canWinWithL1L2");
                                        // force all drone to surface except the deepest (which is going to scan all remaining creatures on level L1 and/or L2)
                                        if (dronesOnDive.size() > 1) {
                                            Drone deepest = dronesOnDive.stream().max(Comparator.comparing(d -> d.pos.y)).orElse(null);
                                            //myDrones.stream().filter(d -> d.id != deepest.id).forEach(d -> {
                                            myDrones.forEach(d -> {
                                                d.diveToSurface = true;
                                                assert d.strategy.peek() != null;
                                                d.strategy.peek().updateDestination(d);
                                            });
                                        }
                                        maxDepth = Depth.L2;
                                    }
                                }

                                target = (target==null)? getDeepestClosestTarget(drone, toScan, maxDepth, null):target;
                            }
                        }

                    }
                }

                if(target == null) {
                    if(destination == null) destination = new Point(drone.pos.x, DEPTH_SURFACE);
                    if(destination.y != DEPTH_SURFACE) {
                        msg = "DIVE";
                    } else {
                        msg = "SURFACE";
                    }
                } else {
                    msg = "DIVE "+target.id;
                    destination = new Point(target.pos.x, target.pos.y);
                    // TODO : optimize approach to target => if sure to be closest than 1 turn, go to the best position for n+1 target
                }
                log("======= DIVE updateDestination END drone="+drone.id+" dest.x="+destination.x+" dest.y="+destination.y+" : "+msg);
            }

            public static Creature getDeepestClosestTarget(Drone drone, List<Creature> toScan, Depth maxDepth, Creature exclude) {
                Creature target = null;
                Depth depth = maxDepth;
                boolean run = true;
                while(run && depth!=Depth.L0) {
                    // target notScanned creatures by priority L3->L1
                    final Depth dd = depth;
                    Creature possibleTarget = toScan.stream()
                            .filter(c -> c.type == dd.type)
                            .filter(c -> exclude==null || c.id!=exclude.id)
                            .min(Comparator.comparing(c -> drone.pos.distanceTo(c.pos))).orElse(null);

                    if (possibleTarget == null) {
                        if (depth == Depth.L3) depth = Depth.L2;
                        else if (depth == Depth.L2) depth = Depth.L1;
                        else depth = Depth.L0;
                    } else {
                        target = possibleTarget;
                        run = false;
                    }
                }
                return target;
            }

            public boolean canWinWithCurrentScan() {
                List<Drone> myDrones = drones.values().stream().filter(d->d.mine).toList();

                // all our current scan
                Set<Integer> possibleScoredAsFirst = new HashSet<>();
                myDrones.forEach(d->possibleScoredAsFirst.addAll(d.currentScan.stream().toList()));

                boolean canWin = canWin(possibleScoredAsFirst);
                if(canWin) log("canWinWithCurrentScan");
                return canWin;
            }

            public boolean canWinWithCurrentScanAnd(int creature) {
                List<Drone> myDrones = drones.values().stream().filter(d->d.mine).toList();

                // all our current scan
                Set<Integer> possibleScoredAsFirst = new HashSet<>();
                myDrones.forEach(d->possibleScoredAsFirst.addAll(d.currentScan.stream().toList()));

                possibleScoredAsFirst.add(creature);

                boolean canWin = canWin(possibleScoredAsFirst);
                if(canWin) log("canWinWithCurrentScanAnd "+creature);
                return canWin;
            }

            public boolean canWinWithCurrentScanAndAbove(Depth depth) {
                List<Drone> myDrones = drones.values().stream().filter(d->d.mine).toList();

                Set<Integer> possibleScoredAsFirst = new HashSet<>();
                // all our scan which are not scored
                myDrones.forEach(d->possibleScoredAsFirst.addAll(d.currentScan.stream().toList()));
                // add all not scored scans from L1 and L2
                cop.getRemainingCreatures().stream().filter(c->c.type<=depth.type).map(c->c.id).forEach(possibleScoredAsFirst::add);

                boolean canWin = canWin(possibleScoredAsFirst);
                if(canWin) log("canWinWithCurrentScanAndRemainingAtMaxDepth "+depth.type);
                return canWin;
            }

            /**
             * calculate our potential score if we score all possibleScoredAsFirst as first and all other as second
             */
            public boolean canWin(Set<Integer> possibleScoredAsFirst) {
                Set<Integer> notScored = cop.getRemainingCreatures().stream().filter(c->c.type!=-1 && !c.myScan && !c.foeScan ).map(c->c.id).collect(Collectors.toSet());
                Set<Integer> remaining = cop.getRemainingCreatures().stream().filter(c->c.type!=-1 && !c.myScan).map(c->c.id).collect(Collectors.toSet());

                Set<Integer> myScoredAsFirst = possibleScoredAsFirst.stream().filter(notScored::contains).collect(Collectors.toSet());


                Set<Integer> currentScan = new HashSet<>();
                drones.values().stream().filter(d->d.mine).forEach(d->currentScan.addAll(d.currentScan.stream().toList()));
                int myPossibleScore = myScore // currentScore
                        + myScoredAsFirst.stream().map(creatures::get).mapToInt(c->(c.type+1)*2).sum() // all current scanned and if scored as first
                        + remaining.stream().filter(i->!myScoredAsFirst.contains(i)).map(creatures::get).mapToInt(c->(c.type+1)).sum(); // all remaining creatures
                // check type combo
                for(int type=0;type<=2;type++) {
                    int i = type;
                    if(creatures.values().stream().anyMatch(c->c.type==i && !c.myScan)) {
                        // is a combo possible (may be some needed creatures left the map)
                        boolean isComboPossible = creatures.values().stream().filter(c->c.type==i && (c.myScan || currentScan.contains(c.id) || cop.getRemainingCreatures().contains(c))).count() == 4;
                        if(isComboPossible) {
                            // a combo is possible (foe doesn't scored all of it)
                            boolean isComboAsFirstPossible = creatures.values().stream().anyMatch(c->c.type==i && !c.foeScan); // if other didn't scored all type
                            Predicate<Creature> isScoredAsFirst = c->myScoredAsFirst.contains(c.id) || c.myScan;
                            myPossibleScore+=(isComboAsFirstPossible && creatures.values().stream().filter(c->c.type==i).allMatch(isScoredAsFirst))?8:4;
                        }
                    }
                }
                // check color combo
                for(int color=0;color<=3;color++) {
                    int i = color;
                    if(creatures.values().stream().anyMatch(c->c.color==i && !c.myScan)) {
                        // is a combo possible (may be some needed creatures left the map)
                        boolean isComboPossible = creatures.values().stream().filter(c->c.color == i && (c.myScan || currentScan.contains(c.id) || cop.getRemainingCreatures().contains(c))).count() == 3;
                        if(isComboPossible) {
                            // a combo is possible
                            boolean isComboAsFirstPossible = creatures.values().stream().anyMatch(c -> c.color == i && !c.foeScan); // if foe didn't scored all color
                            Predicate<Creature> isScoredAsFirst = c -> myScoredAsFirst.contains(c.id) || c.myScan;
                            myPossibleScore += (isComboAsFirstPossible && creatures.values().stream().filter(c -> c.color == i).allMatch(isScoredAsFirst)) ? 6 : 3;
                        }
                    }
                }

                Set<Integer> foeScoredAtFirst = notScored.stream().filter(i->!myScoredAsFirst.contains(i)).collect(Collectors.toSet());
                remaining = cop.getRemainingCreatures().stream().filter(c->c.type!=-1 && !c.foeScan).map(c->c.id).collect(Collectors.toSet());
                currentScan.clear();
                drones.values().stream().filter(d->!d.mine).forEach(d->currentScan.addAll(d.currentScan.stream().toList()));

                int foePossibleScore = foeScore // currentScore
                        + foeScoredAtFirst.stream().map(creatures::get).mapToInt(c->(c.type+1)*2).sum() // all current scanned and if scored as first
                        + remaining.stream().filter(i->!foeScoredAtFirst.contains(i)).map(creatures::get).mapToInt(c->(c.type+1)).sum(); // all remaining creatures
                // check foe type combo
                for(int type=0;type<=2;type++) {
                    int i = type;
                    if(creatures.values().stream().anyMatch(c->c.type==i && !c.foeScan)) {
                        // is a combo possible (may be some needed creatures left the map)
                        boolean isComboPossible = creatures.values().stream().filter(c->c.type==i && (c.foeScan || currentScan.contains(c.id) || cop.getRemainingCreatures().contains(c))).count() == 4;
                        if(isComboPossible) {
                            // a combo is possible
                            boolean isComboAsFirstPossible = creatures.values().stream().anyMatch(c -> c.type == i && !c.myScan); // if we didn't scored all type
                            Predicate<Creature> isScoredAsFirst = c -> foeScoredAtFirst.contains(c.id) || c.foeScan;
                            foePossibleScore += (isComboAsFirstPossible && creatures.values().stream().filter(c -> c.type == i).anyMatch(isScoredAsFirst)) ? 8 : 4;
                        }
                    }
                }
                // check foe color combo
                for(int color=0;color<=3;color++) {
                    int i = color;
                    if(creatures.values().stream().anyMatch(c->c.color==i && !c.foeScan)) {
                        // is a combo possible (may be some needed creatures left the map)
                        boolean isComboPossible = creatures.values().stream().filter(c->c.color == i && (c.foeScan || currentScan.contains(c.id) || cop.getRemainingCreatures().contains(c))).count() == 3;
                        if(isComboPossible) {
                            // a combo is possible
                            boolean isComboAsFirstPossible = creatures.values().stream().anyMatch(c -> c.color == i && !c.myScan); // if I didn't scored all color
                            Predicate<Creature> isScoredAsFirst = c -> foeScoredAtFirst.contains(c.id) || c.foeScan;
                            foePossibleScore += (isComboAsFirstPossible && creatures.values().stream().filter(c -> c.color == i).anyMatch(isScoredAsFirst)) ? 6 : 3;
                        }
                    }
                }
                return myPossibleScore > foePossibleScore;
            }

            public boolean isFinished(Drone drone) {
                log("\n================================================");
                boolean finished =  (!drone.currentScan.isEmpty() && canWinWithCurrentScan())// if we have scanned something and we can win => we're DONE
                        || (drone.diveToSurface && cop.getRemainingCreatures().stream()
                            .filter(c->c.type<Depth.L3.type).noneMatch(c->drones.values().stream().filter(d->d.mine).anyMatch(d->d.currentScan.contains(c.id)) ))
                        || creatures.values().stream().filter(c->c.foeScan).count() >= 6 && myScore == 0; // the opponent is going up early => we have to EXPULSE

                log(">> DIVE isFinished drone="+drone.id+" "+finished);
                return finished;
            }

            @Override
            public boolean isLightOn(Drone drone) {
                log("\n================================================");
                final List<Creature> notScanned = cop.getRemainingCreatures().stream()
                        .filter(c->c.type!=-1)
                        .filter(c->!c.myScan)
                        .filter(c->!drone.currentScan.contains(c.id)) // ignore creatures currently scanned by our drones
                        .toList(); // collect to modifiable collection

                boolean light = (drone.pos.add(drone.pos.vectorTo(destination).normalizedDirection(MAX_MOVE)).y>2700 && turn% LIGHT_ON_EVERY ==0)
                        || (drone.pos.add(drone.pos.vectorTo(destination).normalizedDirection(MAX_MOVE)).y>Depth.L0.middle && notScanned.stream().anyMatch(c->drone.pos.distanceTo(c.pos)<2500)) // if there is a creature nearby
                        || (target!=null && drone.pos.vectorTo(destination).normalizedDirection(MAX_MOVE).distanceTo(target.pos)<2000); // if close to target

                // don't need to light if nothing to scan in next depth
                Depth at = Depth.getDepth(drone.pos.add(drone.pos.vectorTo(destination).normalizedDirection(MAX_MOVE)).y);
                if(at!=null && notScanned.stream().noneMatch(c->c.type==at.type)) light = false;

                log(">> DIVE isLightOn drone="+drone.id+" "+light);
                return light;
            }
        }

        /**
         * Goal : go to the Surface as fast as possible to score scanned creatures
         */
        static class Surface extends Mission {

            @Override
            public boolean isFinished(Drone drone) {
                boolean isFinished = drone.pos.y<500 // reached surface
                        || drone.currentScan.stream().map(creatures::get).allMatch(c -> c.myScan); // or all our currentScan were score by my other drones

                if(isFinished) {
                    drone.diveToSurface = false;
                    drone.forceToSurface = false;
                }

                log("\n================================================");
                log(">>SURFACE isFinished drone="+drone.id+" "+isFinished);

                return isFinished;
            }

            @Override
            public void updateDestination(Drone drone) {
                log("\n================================================");
                log(">>SURFACE updateDestination drone="+drone.id);
                msg = "SURFACE";
                target = null;
                destination = new Point(drone.pos.x, DEPTH_SURFACE);
            }

            @Override
            public boolean isLightOn(Drone drone) {
                log("\n================================================");
                boolean light = drone.pos.y>Depth.L1.start-200 && turn% LIGHT_ON_EVERY ==0;
                log(">>SURFACE isLightOn drone="+drone.id+" "+light);
                return light;
            }
        }

        /**
         * Goal : handles trajectory to escape nearby monsters
         * monsters are sorted in 3 lists
         * - absoluteDanger : to close to us, need to escape them going backwards (add all monsters speed vector, weighted with distance to us)
         * - potentialDanger : far enough to find the best trajectory avoiding those creatures, closest to our target
         * - other Potential : all others creatures
         *
         * the algorithm run in a loop
         * at the end of each iteration, we check if there is a kill hits (=a next position of a monster will be in the 500u range of our next position
         * if yes, the creatures is promotted one list above : otherPotential->potentialDanger->absoluteDanger and a new iteration calculate the best escape position
         */
        static class Avoid extends Mission {

            Mission next;
            boolean finished = false;
            int startTurn;

            public Avoid(Mission next) {
                this.next = next;
                target = next.target;
                msg = "AVOID";
                startTurn = turn;
            }

            @Override
            public void updateDestination(Drone drone) {

                // update destination of the next mission (to be up-to-date in our escape trajectory)
                next.updateDestination(drone);
                this.target = next.target;

                log("AVOID updateDestination droneId="+drone.id +" target="+((target==null)?"NULL":target.id)+ " next.dest.x="+next.destination.x+ " next.dest.y="+next.destination.y);


                // select visible monsters or were visible recently
                Set<Integer> monsters = new HashSet<>();
                for (Creature m:creatures.values()) {
                    if(m.type == -1 && (m.visible || m.wasVisible>0 || cop.radar.get(m.id).lineWasVisible>0 || cop.radar.get(m.id).colWasVisible>0)) {
                        monsters.add(m.id);
                    }
                }

                // select monsters which we have to avoid absolutely (aka we have to go backwards)
                Creature monster;
                Set<Integer> absoluteDanger = new HashSet<>();
                for(int i:monsters) {
                    monster=creatures.get(i);
                    if(drone.pos.distanceTo(monster.pos) <= 650 && drone.pos.distanceTo(monster.pos) >= drone.pos.distanceTo(monster.pos.add(new Point(monster.vx, monster.vy)))) {
                        absoluteDanger.add(i);
                    }
                }
                absoluteDanger.stream().map(creatures::get).forEach(m -> log("AVOID updateDestination ABSOLUTEDANGER id="+m.id+" m.x="+m.pos.x+" m.y="+m.pos.y+" dist="+drone.pos.distanceTo(m.pos)+" m.vis="+m.visible+" m.was="+m.wasVisible+" vx="+m.vx+" vy="+m.vy));

                // select monsters which are not a direct danger but need a trajectory adjustment to avoid them
                Set<Integer> potentialDanger = new HashSet<>();
                for(int i:monsters) {
                    if(absoluteDanger.contains(i)) continue;
                    monster=creatures.get(i);
                    if((monster.visible && drone.pos.distanceTo(monster.pos) >= drone.pos.distanceTo(monster.pos.add(new Point(monster.vx, monster.vy))) && drone.pos.distanceTo(monster.pos.add(new Point(monster.vx, monster.vy))) < LIGHT_1_RADIUS) // visibleAndGoingCloseToMe
                            || (!monster.visible && drone.pos.distanceTo(monster.pos)<2000)) // wasVisibleAndCloseToMe
                    {
                        potentialDanger.add(i);
                    }
                }
                potentialDanger.stream().map(creatures::get).forEach(m -> log("AVOID updateDestination POTENTIAL id="+m.id+" m.x="+m.pos.x+" m.y="+m.pos.y+" dist="+drone.pos.distanceTo(m.pos)+" m.vis="+m.visible+" m.was="+m.wasVisible+" vx="+m.vx+" vy="+m.vy+" m.nextx=" + m.pos.add(new Point(m.vx, m.vy)).x + " m.nexty=" + m.pos.add(new Point(m.vx, m.vy)).y));

                // select all monsters being globaly visible which could be a potential danger in next turn
                Set<Integer> otherPotential = monsters.stream()
                        .filter(m -> !absoluteDanger.contains(m))
                        .filter(m -> !potentialDanger.contains(m))
                        .collect(Collectors.toSet());
                // add creature from radar
                cop.radar.values().stream()
                        .filter(p -> !monsters.contains(p.id))
                        .map(p->creatures.get(p.id))
                        .filter(m -> m.type == -1)
                        .filter(m -> drone.pos.distanceTo(m.pos)<2000)
                        .map(m->m.id)
                        .forEach(otherPotential::add);

                otherPotential.stream().map(creatures::get).forEach(m -> log("AVOID updateDestination OTHER id="+m.id+" m.x="+m.pos.x+" m.y="+m.pos.y+" dist="+drone.pos.distanceTo(m.pos)+" m.vis="+m.visible+" m.was="+m.wasVisible+" vx="+m.vx+" vy="+m.vy));

                Set<Integer> additionnal = new HashSet<>();

                destination = new Point(5000,5000);
                boolean run = true;
                while (run) {
                    finished = false;
                    Vector avoidVector = new Vector();

                    if(!absoluteDanger.isEmpty() || !potentialDanger.isEmpty()) {
                        Set<Integer> list = new HashSet<>();
                        if (!absoluteDanger.isEmpty()) {
                            log("AVOID updateDestination step 1");
                            list.addAll(absoluteDanger);
                            otherPotential.addAll(potentialDanger);
                        } else {
                            log("AVOID updateDestination step 2");
                            list.addAll(potentialDanger);
                        }
                        list.addAll(additionnal);

                        if (!absoluteDanger.isEmpty()) {
                            Vector vec = new Vector();
                            list.stream()
                                    .map(creatures::get)
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
                                        log("AVOID updateDestination step : monster.id=" + m.id + " m.x=" + m.pos.x + " m.y=" + m.pos.y + " m.nextx=" + nextPos.x + " m.nexty=" + nextPos.y);
                                        log("AVOID updateDestination step : monster.id=" + m.id + " dist=" + drone.pos.distanceTo(m.pos) + " nextDist=" + dist + " m.vis="+m.visible+" m.was="+m.wasVisible+" vec.x=" + vec.normalizedDirection(100).x + " vec.y=" + vec.normalizedDirection(100).y);
                                        log("AVOID updateDestination step : c.id="+m.id+" v.x="+v.x+" v.y="+v.y);
                                    });
                            avoidVector = vec;
                        }
                        else {
                            // find the best azimuth
                            Point avoidPoint = null;
                            double minDist = Double.MAX_VALUE;
                            for (Point azimuth: azimuths) {
                                Point pt = drone.pos.add(azimuth);
                                // filter out points which are out of map
                                if(pt.x<0 || pt.x>10000 || pt.y<0 || pt.y>10000) continue;
                                // filter out points which are out of map
                                if(list.stream().map(creatures::get).anyMatch(m->pt.distanceTo(m.pos) <= (m.visible?LIGHT_0_RADIUS:LIGHT_0_RADIUS*2))) continue;
                                // filter out points which are in the LIGHT_0_RADIUS of next position of the monster
                                if(list.stream().map(creatures::get).anyMatch(m->pt.distanceTo(m.pos.add(new Point(m.vx, m.vy))) <= (m.visible?KILL_DISTANCE+150:LIGHT_0_RADIUS*2))) continue;
                                double dist = pt.distanceTo(next.destination);
                                if(dist<minDist) {
                                    //log("AZIMUTH MIN p.x="+pt.x+" p.y="+pt.y+" dist="+dist);
                                    minDist = dist;
                                    avoidPoint = pt;
                                }
                            }

                            if(avoidPoint!=null) {
                                // check if we can go directly to destination
                                avoidVector = drone.pos.vectorTo(avoidPoint);
                            }
                        }

                    } else {
                        finished = true;
                    }

                    if (avoidVector.size() == 0) {
                        // if avoid vector is
                        log("AVOID updateDestination step : going directly to next.destination");
                        avoidVector = drone.pos.vectorTo(next.destination);
                    }
                    destination = drone.pos.add(avoidVector.normalizedDirection(MAX_MOVE));
                    destination.y = Math.max(DEPTH_SURFACE, destination.y);
                    log("AVOID found dest.x="+destination.x+" dest.y="+destination.y);

                    otherPotential.forEach(i->{
                        log("check KILL HIT 1 c.id="+creatures.get(i).id);
                        log("check KILL HIT 2 c.id="+(drone.pos.add(drone.pos.vectorTo(destination).normalizedDirection(MAX_MOVE)).distanceTo(creatures.get(i).pos.add(new Point(creatures.get(i).vx, creatures.get(i).vy)))));
                        log("check KILL HIT 3 c.id="+(drone.pos.add(drone.pos.vectorTo(destination).normalizedDirection(MAX_MOVE)).distanceTo(creatures.get(i).pos)));
                    });
                    // check whether there is a potential kill
                    Set<Integer> killHitsList = new HashSet<>();
                    killHitsList.addAll(otherPotential);
                    killHitsList.addAll(potentialDanger);
                    Set<Integer> killHits = killHitsList.stream()
                            .map(creatures::get)
                            .filter(m -> !additionnal.contains(m.id))
                            .filter(m -> drone.pos.add(drone.pos.vectorTo(destination).normalizedDirection(MAX_MOVE)).distanceTo(m.pos.add(new Point(m.vx, m.vy))) < (m.visible?KILL_DISTANCE+50:KILL_DISTANCE*2)
                                    || drone.pos.add(drone.pos.vectorTo(destination).normalizedDirection(MAX_MOVE)).distanceTo(m.pos) < (m.visible?KILL_DISTANCE+150:KILL_DISTANCE*2))
                            .map(m->m.id)
                            .collect(Collectors.toSet());
                    killHits.stream().map(creatures::get).forEach(m -> log("AVOID updateDestination KILL HITS id=" + m.id));
                    if (killHits.isEmpty()) {
                        run = false;
                    } else {
                        additionnal.addAll(killHits);
                        if(killHits.stream().anyMatch(potentialDanger::contains)) absoluteDanger.addAll(killHits);
                        if(potentialDanger.isEmpty()) potentialDanger.addAll(killHits);
                    }

                }
                log("AVOID dest.x="+destination.x+" dest.y="+destination.y);
                Vector v = drone.pos.vectorTo(destination);
                if(destination.y > 9700) {
                    log("AVOID updateDestination OUT_OF_MAP y>9700 target.y="+destination.y);
                    destination = new Point(destination.x+((v.x>0)?1000:-1000), 10000);
                }
                if(destination.x < 300) {
                    log("AVOID updateDestination OUT_OF_MAP x<300 target.x="+destination.x);
                    destination = new Point(0, destination.y+((v.y>0 || destination.y<9700)?1000:-1000));
                }
                if(destination.x > 9700) {
                    log("AVOID updateDestination OUT_OF_MAP x>9700 target.x="+destination.x);
                    destination = new Point(0, destination.y+((v.y>0 || destination.y<9700)?1000:-1000));
                }

                log("AVOID updateDestination END dest.x="+destination.x+" dest.y="+destination.y+ " target="+((target==null)?"NULL":target.id)+" next.target="+((next.target==null)?"NULL":next.target.id));
                msg = "AVOID "+next.msg;
                if(finished) msg = next.msg;
            }

            @Override
            public boolean isLightOn(Drone drone) {
                // light if
                // - close to our target (because if scanned, it can change the next one)
                // - or a nearby monsters just disappeared (wasVisible=1)
                boolean light = drone.pos.y>Depth.L1.start
                        && (creatures.values().stream().anyMatch(m -> m.type == -1 && !m.visible && m.wasVisible==1 && drone.pos.distanceTo(m.pos)<2000)
                        || drone.pos.distanceTo(next.destination)<LIGHT_1_RADIUS+500);
                if(light) drone.disableLightTurnCounter = 0;
                log("\n================================================");
                log(">> AVOID isLightOn drone="+drone.id+ " "+light);
                return light;
            }

            @Override
            public boolean isFinished(Drone drone) {
                log("\n================================================");
                finished = finished || (next instanceof Surface && next.isFinished(drone));
                //log(">> AVOID isFinished 1 "+finished);
                //log(">> AVOID isFinished 2 "+next.getClass().getSimpleName());
                //log(">> AVOID isFinished 3 "+(next.isFinished(drone)));
                //log(">> AVOID isFinished 4 "+(drone.pos.y<Depth.L1.start-600));
                log(">> AVOID isFinished drone="+drone.id+ " "+finished);
                return finished;
            }
        }

        /**
         * Goal : eject creature from the map, in order to avoid an opponent to scan it
         */
        static class Bouh extends Mission {

            Mission next;
            boolean finished;

            public Bouh(Creature target, Mission next) {
                this.target = target;
                this.msg = "BOUH "+target.id;
                this.next = next;
            }

            @Override
            public boolean isFinished(Drone drone) {
                if(finished) return true;
                //log("BOUH isFinished 1 "+(target.foeScan));
                //log("BOUH isFinished 2 "+(!cop.getRemainingCreatures().contains(target)));
                boolean value = target.foeScan
                        || !cop.getRemainingCreatures().contains(target)
                        || drones.values().stream().anyMatch(d->{
                            Point n = target.pos.add(d.pos.vectorTo(target.pos).normalizedDirection(400));
                            double dist = d.pos.distanceTo(target.pos);
                            //log("BOUH isFinished d.id="+d.id+ " dist="+dist+" n.x="+n.x+" n.y="+n.y);
                            return dist<=1400 && (n.x>10000 || n.x<0);
                        })
                        || drones.values().stream().filter(d->!d.mine).anyMatch(d->d.currentScan.contains(target.id));
                log("\n================================================");
                log(">> BOUH isFinished drone="+drone.id+ " value="+value);
                finished = value;
                return value;
            }

            @Override
            public void updateDestination(Drone drone) {
                log("\n================================================");
                if(!finished) isFinished(drone);
                if(finished) {
                    log("BOUH updateDestination drone="+drone.id+" next="+next.getClass().getSimpleName());
                    next.updateDestination(drone);
                    destination = next.destination;
                    target = next.target;
                    msg = next.msg;
                } else {
                    log("BOUH updateDestination drone="+drone.id);
                    Point targetNextPos = target.pos.add(new Point(target.vx, target.vy));
                    long x = targetNextPos.x+(targetNextPos.x<5000?10:-10); // go 10u behind the creature
                    x = (targetNextPos.x<5000)?Math.max(1200,x):Math.min(8800,x); // maximum 1200u from border
                    destination = new Point(x, targetNextPos.y);
                }
            }

            @Override
            public boolean isLightOn(Drone drone) {
                if(finished) {
                    return next.isLightOn(drone);
                } else {
                    log("\n================================================");
                    final List<Creature> notScanned = cop.getRemainingCreatures().stream()
                            .filter(c->c.type!=-1)
                            .filter(c->!c.myScan)
                            .filter(c->!drone.currentScan.contains(c.id)) // ignore creatures currently scanned by our drones
                            .toList();

                    boolean light = (drone.pos.add(drone.pos.vectorTo(destination).normalizedDirection(MAX_MOVE)).y>2700 && turn% LIGHT_ON_EVERY ==0)
                            || (drone.pos.add(drone.pos.vectorTo(destination).normalizedDirection(MAX_MOVE)).y>Depth.L0.middle && notScanned.stream().anyMatch(c->drone.pos.distanceTo(c.pos)<2500)) // if there is a creature nearby
                            || (target!=null && drone.pos.vectorTo(destination).normalizedDirection(MAX_MOVE).distanceTo(target.pos)<2000); // if close to target

                    // don't need to light if nothing to scan in next depth
                    Depth at = Depth.getDepth(drone.pos.add(drone.pos.vectorTo(destination).normalizedDirection(MAX_MOVE)).y);
                    if(at!=null && notScanned.stream().noneMatch(c->c.type==at.type)) light = false;

                    log(">> BOUH isLightOn drone="+drone.id+" "+light);
                    return light;
                }

            }
        }

        /**
         * Represents a Creature
         */
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

        /**
         * Represents a Drone
         */
        static class Drone {
            int id;
            boolean mine = false;
            Point pos = new Point(0,0);
            int emergency;
            int battery;
            Set<Integer> currentScan = new HashSet<>();
            Map<String, Set<Integer>> radars = new HashMap<>();

            final Deque<Mission> strategy = new ArrayDeque<>();
            int initPosition = -1;
            int disableLightTurnCounter = 0;
            boolean diveToSurface = false;
            boolean forceToSurface = false;

            boolean light = false;

            public Drone() {
                radars.put("BL", new HashSet<>());
                radars.put("BR", new HashSet<>());
                radars.put("TL", new HashSet<>());
                radars.put("TR", new HashSet<>());
            }

            private Mission peekMission() {
                if(strategy.isEmpty()) {
                    strategy.push(new Surface());
                    strategy.push(new Dive());
                }
                return strategy.peek();
            }

            void resetRadar() {
                radars.values().forEach(Set::clear);
            }

            public void update() {

                disableLightTurnCounter--;
                if(initPosition == -1) {
                    List<Drone> hSortedDrones= drones.values().stream().sorted(Comparator.comparing(d -> d.pos.x)).toList();
                    initPosition = hSortedDrones.indexOf(this);
                }

                if(!isGloballyFinished) {
                    isGloballyFinished = new Dive().canWinWithCurrentScan() && cop.getFoeRemainingCreaturesCount() == 0;
                }

                Mission mission = peekMission();
                int counter = 0;
                // poll strategy stack until we reached
                while(mission.isFinished(this) && counter<2) {
                    //log("DRONE.process POLL droneId="+this.id+" mission="+mission.getClass().getSimpleName());
                    strategy.poll();
                    mission = peekMission();
                    if(strategy.size() == 1) counter++;
                }
                if(counter>=2) {
                    mission = new Surface();
                }
                // TODO if DIVE finished => expulse other fishes
                log("\n======================================================================");


                // handle monsters
                List<Creature> monsters = creatures.values().stream()
                        .filter(c -> c.type == -1) // keep only monsters
                        .filter(c -> c.visible || c.wasVisible>0 || cop.radar.get(c.id).lineWasVisible>0 || cop.radar.get(c.id).colWasVisible>0) // keep only visible
                        .filter(c -> this.pos.distanceTo(c.pos) <= LIGHT_MONSTERS_EXTRA+LIGHT_1_RADIUS+500)
                        .toList();
                // monster avoidance management
                if(!monsters.isEmpty() && !(mission instanceof Avoid)) {
                    mission = new Avoid(mission);
                    strategy.push(mission);
                }

                if(!(mission instanceof Bouh)) {
                    log("BOUH checkd drone="+this.id);

                    // handle BOUH nearby creatures
                    Creature toBouh = null;
                    double maxDist = Double.MAX_VALUE;
                    for (Creature c:cop.getRemainingCreatures()) {
                        if(c.type==-1) continue;
                        if(c.foeScan) continue;
                        if((this.initPosition) < 2 ? c.pos.x > this.pos.x : c.pos.x < this.pos.x) continue;
                        if(this.pos.distanceTo(c.pos) > 3000) continue;
                        if(drones.values().stream().anyMatch(d-> {
                            Point n = c.pos.add(d.pos.vectorTo(c.pos).normalizedDirection(400));
                            return d.pos.distanceTo(c.pos) <= 1400 && (n.x > 10000 || n.x < 0);
                        })) continue;
                        if(drones.values().stream().anyMatch(d -> !d.mine && (d.currentScan.contains(c.id) || d.pos.distanceTo(c.pos) < LIGHT_0_RADIUS+1000 || d.pos.distanceTo(c.pos) < this.pos.distanceTo(c.pos)))) continue;
                        if(toBouh!=null && c.type<=toBouh.type) continue;
                        log(">> BOUH eligible target="+ c.id);
                        double dist = this.pos.distanceTo(c.pos);
                        if(toBouh==null || (c.type>toBouh.type || dist<maxDist)) {
                            toBouh = c;
                            maxDist = dist;
                        }
                    }

                    if(toBouh!=null) {
                        log(">> BOUH FOUND target="+ toBouh.id);
                        Bouh bouh = new Bouh(toBouh, mission);
                        if(mission instanceof Avoid) {
                            Mission avoid = strategy.poll();
                            avoid.target = toBouh;
                            ((Avoid)avoid).next = bouh;
                            bouh.next = strategy.peek();
                            strategy.push(bouh);
                            strategy.push(avoid);
                        } else {
                            strategy.push(bouh);
                        }

                    }
                }


                log("DRONE.process droneId="+this.id+" mission="+mission.getClass().getSimpleName()+" : "+mission.msg);

                // get mission waypoint
                mission.updateDestination(this);

            }
            public String output() {
                if(emergency==1) {
                    strategy.clear();
                    return "WAIT 0 EMERGENCY";
                } else {
                    Mission mission = peekMission();
                    if(mission.destination==null) {
                        log("output drone="+this.id+" WHY DESTINATION IS EMPTY !!!!!");
                        mission.destination = new Point(this.pos.x, DEPTH_SURFACE);
                    }
                    // light management
                    boolean arrivingDepthL3 = mission.destination.y>this.pos.y && this.pos.add(this.pos.vectorTo(mission.destination).normalizedDirection(MAX_MOVE)).y>7000;
                    light = battery>=5 && (this.pos.y > Depth.L1.start-500) && (arrivingDepthL3 || (mission.isLightOn(this) && disableLightTurnCounter<=0));
                    if(light) this.disableLightTurnCounter=2;

                   return "MOVE "+mission.destination.x+" "+mission.destination.y+" "+(light?1:0)+" B="+this.battery+" "+mission.msg;
                }
            }
        }

        public static void main(String args[]) {
            Scanner in = new Scanner(System.in);
            int creatureCount = in.nextInt();
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
                    //log("SCAN droneId="+droneId+" creatureId="+creatureId);
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
                    //log("RADAR droneId="+droneId+" creatureId="+creatureId+" radar="+radar);
                }
                time=System.currentTimeMillis();
                cop.update();

                for (Drone d: drones.values()) {
                    if(d.mine) d.update();
                }
                for (Drone d: drones.values()) {
                    if(d.mine) System.out.println(d.output());
                }
                //drones.values().stream().filter(d->d.mine).forEach(Drone::update);
                //drones.values().stream().filter(d->d.mine).map(Drone::output).forEach(System.out::println);
            }
        }
    }