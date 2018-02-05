/**
 * Copyright (c) 2014, by the Authors: Antonio Sanchez (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.core.mfreemodels;

import java.util.Arrays;

import artisynth.core.mechmodels.PointState;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public class MFreeVertex3d extends Vertex3d implements MFreePoint3d {

   public static double DEFAULT_COORDINATE_TOLERANCE = 1e-8;
   
   MFreeNode3d[] myDependentNodes;
   VectorNd myNodeCoords;
   PointState myState;
   private Point3d myRestPosition;
   
   public MFreeVertex3d(MFreeNode3d[] dependentNodes, VectorNd coords) {
      myState = new PointState();
      myRestPosition = new Point3d();
      setDependentNodes(dependentNodes, coords);
      setPosition(myRestPosition);
   }
  
   public MFreeNode3d[] getDependentNodes() {
      return myDependentNodes;
   }

   public void setDependentNodes(MFreeNode3d[] nodes, VectorNd coords) {
      myDependentNodes = Arrays.copyOf(nodes, nodes.length);
      myNodeCoords = new VectorNd(coords);
      reduceDependencies(DEFAULT_COORDINATE_TOLERANCE);
      
      updateRestPosition();
   }
   
   public void updateRestPosition() {
      myRestPosition.setZero();
      for (int i=0; i<myDependentNodes.length; i++) {
         myRestPosition.scaledAdd(myNodeCoords.get(i), myDependentNodes[i].getRestPosition());
      }
   }
   
   public Point3d getRestPosition() {
      return myRestPosition;
   }

   public VectorNd getNodeCoordinates() {
      return myNodeCoords;
   }

   public void setNodeCoordinates(VectorNd coords) {
      myNodeCoords.set(coords);
      updateRestPosition();
   }

   public void updatePosState() {
      myState.setPos(Point3d.ZERO);
      for (int i=0; i<myDependentNodes.length; i++) {
         myState.scaledAddPos(myNodeCoords.get(i), myDependentNodes[i].getFalsePosition());
      }
      pnt.set(myState.getPos());
   }

   public void updateVelState() {
      myState.setVel(Vector3d.ZERO);
      for (int i=0; i<myDependentNodes.length; i++) {
         myState.scaledAddVel(myNodeCoords.get(i), myDependentNodes[i].getFalseVelocity());
      }      
   }
   
   public void updatePosAndVelState() {
      updatePosState();
      updateVelState();
   }
   
   @Override
   public MFreeVertex3d clone() {
      MFreeVertex3d vtx = (MFreeVertex3d)super.clone();
      
      vtx.myDependentNodes = Arrays.copyOf(myDependentNodes, myDependentNodes.length);
      vtx.myNodeCoords = new VectorNd(myNodeCoords);
      vtx.updateRestPosition();

      return vtx;
   }
   
   public boolean reduceDependencies(double tol) {
      int ndeps = 0;
      boolean changed = false;
      for (int i=0; i<myDependentNodes.length; i++) {
         if (Math.abs(myNodeCoords.get(i)) <= tol) {
            changed = true;
            myNodeCoords.set(i, 0);
         } else {
            if (changed) {
               myDependentNodes[ndeps] = myDependentNodes[i];
               myNodeCoords.set(ndeps, myNodeCoords.get(i));
            }
            ++ndeps;
         }
      }
      if (changed) {
         myDependentNodes = Arrays.copyOf(myDependentNodes, ndeps);
         myNodeCoords.setSize(ndeps);
         myNodeCoords.scale(1.0/myNodeCoords.sum()); // re-sum to one   
      }
      
      return changed;
      
   }
   
}
