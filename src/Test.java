public class Test {

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
        double x;
        double y;


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

    }

    public static void main(String[] args) {
        Vector direction = new Vector();
        direction.x = -1584;
        direction.y = 852;
        Vector v1 = direction.rotate(Math.PI/2d);
        System.err.println("v.x="+v1.x+ " v.y="+v1.y);
        Point p1 = v1.normalizedDirection(600);
        System.err.println("p.x="+p1.x+ " p.y="+p1.y);
        Vector v2 = direction.rotate(-Math.PI/2d);
        System.err.println("v.x="+v2.x+ " v.y="+v2.y);
    }
}
