/**
 * Copyright (c) 2014, by the Authors: John E Lloyd (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.core.femmodels;

import java.io.IOException;
import java.io.PrintWriter;
import java.util.Arrays;
import java.util.Deque;
import java.util.LinkedList;

import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.TransformGeometryContext;
import artisynth.core.util.ScanToken;
import maspack.geometry.DelaunayInterpolator;
import maspack.geometry.GeometryTransformer;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.util.ArraySort;
import maspack.util.IndentingPrintWriter;
import maspack.util.NumberFormat;
import maspack.util.ReaderTokenizer;

/**
 * A muscle element description that uses explicit
 * directions (either per element, or per integration point).
*/
public class MuscleElementDesc extends MuscleElementDescBase {

   Vector3d myDir = new Vector3d();
   Vector3d[] myDirs = null;

   public MuscleElementDesc () {
      super();
   }

   public MuscleElementDesc (FemElement3d elem, Vector3d dir) {
      super(elem);
      if (dir != null) {
         setDirection (dir);
      }
   }

   public static PropertyList myProps =
      new PropertyList (MuscleElementDesc.class, MuscleElementDescBase.class);

   static {
      myProps.add ("direction", "fibre direction", Vector3d.ZERO);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
     
   public void setDirection (Vector3d dir) {
      myDir.set (dir);
      myDir.normalize();
   }

   public Vector3d getDirection() {
      return myDir;
   }

   /**
    * Sets a list of directions for this MuscleElementDesc, one for each
    * integration point in the element. This will override the single
    * per-element direction specified by {@link #setDirection}.  If a
    * particular direction is <code>null</code>, then no stress will be applied
    * at the corresponding integration point. Supplying a <code>null</code>
    * value for <code>dirs</code> will disable per-integration point
    * directions.
    */
   public void setDirections (Vector3d[] dirs) {
      if (dirs == null) {
         myDirs = null;
      }
      else {
         myDirs = new Vector3d[dirs.length];
         for (int i=0; i<dirs.length; i++) {
            if (dirs[i] != null) {
               if (dirs[i].equals (Vector3d.ZERO)) {
                  throw new IllegalArgumentException (
                     "direction vector "+i+" is zero");
               }
               myDirs[i] = new Vector3d (dirs[i]);
               myDirs[i].normalize();
            }
         }
      }
   }

   public Vector3d[] getDirections() {
      return myDirs;
   }
   
   @Override
   public Vector3d getMuscleDirection(IntegrationPoint3d pnt) {
      return getMuscleDirection(pnt.getNumber());
   }
   
   @Override
   public Vector3d getMuscleDirection(int ipntIdx) {
      if (myDirs != null) {
         return myDirs[ipntIdx];
      }
      return myDir;
   }

   public void interpolateDirection (
      DelaunayInterpolator interp, Vector3d[] restDirs) {

      int[] idxs = new int[4];
      double[] wghts = new double[4];
      Vector3d dir = new Vector3d();
      Point3d loc = new Point3d();

      FemElement3d e = getElement();
      IntegrationPoint3d warpingPnt = e.getWarpingPoint();
      warpingPnt.computeRestPosition (loc, e.getNodes());
      interp.getInterpolation (wghts, idxs, loc);

      // Ideally, we just want to create a weighted average of directions.  But
      // there is a problem: we don't care about the *sign* of the direction,
      // in that -u is the same as u. But the sign can make a huge difference
      // when we average: (u + (-u)) != (u + u). 
      //
      // We handle this as follows: when we add a new direction, we adjust its
      // sign so that it is as closely aligned as possble with the accumulated
      // direction. We also start accumulating directions starting with those
      // that have the largest weight.
      //
      // This is probably not an "optimal" solution - will try to figure
      // this out more correctly later.

      // arrange weights into ascending order
      ArraySort.quickSort (wghts, idxs);
      dir.setZero();
      for (int i=3; i>=0; i--) {
         if (idxs[i] != -1) {
            double w = wghts[i];
            // first time through, dir == 0 and so dot product == 0.
            if (dir.dot (restDirs[idxs[i]]) < 0) {
               w = -w;
            }
            dir.scaledAdd (w, restDirs[idxs[i]]);
         }
      }
      setDirection (dir);
   }
   
   public void interpolateIpntDirection (
      DelaunayInterpolator interp, Vector3d[] restDirs) {

      int[] idxs = new int[4];
      double[] wghts = new double[4];
      Vector3d dir = new Vector3d();
      Point3d loc = new Point3d();

      FemElement3d e = getElement();
      
      Vector3d[] dirs = myDirs;
      if (dirs == null) {
         return;
      }
      
      IntegrationPoint3d[] ipnts = e.getIntegrationPoints();
      for (int j=0; j<e.numIntegrationPoints(); ++j) {
         
         if (dirs[j] != null) {
            
            ipnts[j].computePosition(loc, e);
            interp.getInterpolation (wghts, idxs, loc);
            
            // arrange weights into ascending order
            ArraySort.quickSort (wghts, idxs);
            dir.setZero();
            for (int i=3; i>=0; i--) {
               if (idxs[i] != -1) {
                  double w = wghts[i];
                  // first time through, dir == 0 and so dot product == 0.
                  if (dir.dot (restDirs[idxs[i]]) < 0) {
                     w = -w;
                  }
                  dir.scaledAdd (w, restDirs[idxs[i]]);
               }
            }
            
            dirs[j].set(dir);
         }
      }
      
      myDirs = dirs;
   }

   public void transformGeometry (
      GeometryTransformer gtr, TransformGeometryContext context, int flags) {

      // Transform the direction vectors associated with this ElementDesc.
      Point3d ref = new Point3d();
      if (gtr.isAffine()) {
         gtr.transformVec (myDir, ref);
         myDir.normalize();
         if (myDirs != null) {
            for (int i=0; i<myDirs.length; i++) {
               if (myDirs[i] != null) {
                  gtr.transformVec (myDirs[i], ref);
                  myDirs[i].normalize();
               }
            }
         }
      }
      else {
         // need to specify a reference position for each direction, since
         // vector transformations are position dependent.

         // compute element center as a reference position:
         myElement.getWarpingPoint().computePosition (ref, myElement);         
         gtr.transformVec(myDir, ref);
         myDir.normalize();
         if (myDirs != null) {
            IntegrationPoint3d[] ipnts = myElement.getIntegrationPoints();
            for (int i=0; i<myDirs.length; i++) {
               if (myDirs[i] != null) {
                  // compute integration point location as a reference position:
                  ipnts[i].computePosition (ref, myElement);
                  gtr.transformVec (myDirs[i], ref);
                  myDirs[i].normalize();
               }
            }
         }
      }
   }
 
   void scanDirections (ReaderTokenizer rtok) throws IOException {
      rtok.scanToken ('[');
      LinkedList<Vector3d> directions = new LinkedList<Vector3d>();
      while (rtok.nextToken() != ']') {
         if (rtok.tokenIsWord() && rtok.sval.equals ("null")) {
            directions.add (null);
         }
         else if (rtok.tokenIsNumber()) {
            rtok.pushBack();
            double x = rtok.scanNumber();
            double y = rtok.scanNumber();
            double z = rtok.scanNumber();
            directions.add (new Vector3d(x, y, z));
         }
         else {
            throw new IOException ("Expected null or number, "+rtok);
         }
      }
      setDirections (directions.toArray(new Vector3d[0]));
   }

   void printDirections (PrintWriter pw, NumberFormat fmt) {
      pw.println ("directions=[");
      IndentingPrintWriter.addIndentation (pw, 2);
      
      for (int i=0; i<myDirs.length; i++) {
         if (myDirs[i] != null) {
            pw.println (myDirs[i].toString (fmt));
         }
         else {
            pw.println ("null");
         }
      }
      IndentingPrintWriter.addIndentation (pw, -2);
      pw.println ("]");
   }

   @Override
   public boolean scanItem (ReaderTokenizer rtok, Deque<ScanToken> tokens)
      throws IOException {

      rtok.nextToken();
      if (scanAttributeName (rtok, "directions")) {
         scanDirections (rtok);
         return true;
      }
      rtok.pushBack();
      return super.scanItem (rtok, tokens);
   }

   public void writeItems (
      PrintWriter pw, NumberFormat fmt, CompositeComponent ancestor)
      throws IOException {
      super.writeItems (pw, fmt, ancestor);
      if (myDirs != null && myDirs.length > 0) {
         printDirections (pw, fmt);
      }
   }

   @Override
   public MuscleElementDesc clone() {
      
      MuscleElementDesc other;
      other = (MuscleElementDesc)(super.clone());

      other.myDir = myDir.clone();
      if (myDirs != null) {
         other.myDirs = Arrays.copyOf(myDirs, myDirs.length);
      }       
      return other;
   }

}
