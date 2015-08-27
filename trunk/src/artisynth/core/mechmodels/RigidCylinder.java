package artisynth.core.mechmodels;

import java.io.*;
import java.util.*;

import maspack.geometry.*;
import maspack.util.*;
import maspack.matrix.*;
import maspack.spatialmotion.*;
import artisynth.core.modelbase.*;
import artisynth.core.util.*;

public class RigidCylinder extends RigidBody implements Wrappable {
   
   double myRadius = 1.0;

   public RigidCylinder() {
      super (null);
   }

   public double getRadius() {
      return myRadius;
   }

   public void setRadius (double r) {
      if (r != myRadius) {
         myRadius = r;
      }
   }      

   public RigidCylinder (String name, double r, double h, double density) {
      this (name, r, h, density, 20);
   }

   public RigidCylinder (
      String name, double r, double h, double density, int nsides) {

      super (name);
      PolygonalMesh mesh = MeshFactory.createCylinder (r, h, nsides);
      setMesh (mesh, null);
      // body.setDensity (density);
      double mass = Math.PI*r*r*h*density;
      setInertia (SpatialInertia.createCylinderInertia (mass, r, h));
      myRadius = r;
   }

   protected void writeItems (
      PrintWriter pw, NumberFormat fmt, CompositeComponent ancestor)
      throws IOException {
      super.writeItems (pw, fmt, ancestor);
      pw.println ("radius=" + fmt.format(getRadius()));
   }

   protected boolean scanItem (ReaderTokenizer rtok, Deque<ScanToken> tokens)
      throws IOException {

      rtok.nextToken();
      if (scanAttributeName (rtok, "radius")) {
         double r = rtok.scanNumber();
         setRadius (r);
         return true;
      }
      rtok.pushBack();
      return super.scanItem (rtok, tokens);
   }

   private int signCross2d (Point3d p0, Point3d p1) {
      double xprod = p0.x*p1.y - p0.y*p1.x;
      return xprod >= 0 ? 1 : -1;
   }

   public void surfaceTangent (
      Point3d pr, Point3d p0, Point3d p1, double lam, Vector3d sideNrm) {

      // transform p0 and p1 into local coordinates
      Point3d p0loc = new Point3d(p0);
      Point3d p1loc = new Point3d(p1);    
      p0loc.inverseTransform (getPose());
      p1loc.inverseTransform (getPose());

      // there are at most two tangent points. t0loc and t1loc are used to
      // store the values for these in local coordinates:
      Point3d t0loc = new Point3d();
      Point3d t1loc = new Point3d();

      double z0 = p0loc.z;
      double z1 = p1loc.z;
      p0loc.z = 0;
      p1loc.z = 0;

      if (QuadraticUtils.circleTangentPoints (
             t0loc, t1loc, p0loc, myRadius) == 0) {
         // project p0 or p1 to the surface
         double mag = p0loc.norm();
         if (mag != 0) {
            t0loc.scale (myRadius/mag, p0loc);
         }
         else {
            t0loc.scale (myRadius/p1loc.norm(), p1loc);
         }
      }
      else if (signCross2d (p0loc, p1loc) == signCross2d (p0loc, t1loc)) {
         // use t1 instead of t0
         t0loc.set (t1loc);
      }
      double l = LineSegment.projectionParameter (p0loc, p1loc, t0loc);
      t0loc.z = (1-l)*z0 + l*z1;
      pr.transform (getPose(), t0loc);
   }


   public double penetrationDistance (Vector3d nrm, Point3d p0) {
      Point3d p0loc = new Point3d(p0);
      p0loc.inverseTransform (getPose());
      nrm.set (p0loc);
      nrm.z = 0;
      double mag = nrm.norm();
      if (mag > 0) {
         nrm.scale (1/mag);
      }
      else {
         nrm.set (1, 0, 0);
      }
      nrm.transform (getPose());
      return mag-myRadius;
   }

}