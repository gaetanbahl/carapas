/**
 * Copyright (c) 2014, by the Authors: Antonio Sanchez (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.core.mfreemodels;

import maspack.matrix.Point3d;
import maspack.matrix.VectorNd;

public interface MFreePoint3d {
   
   public Point3d getRestPosition();
   public MFreeNode3d[] getDependentNodes();
   public void setDependentNodes(MFreeNode3d[] nodes, VectorNd coords);
   public VectorNd getNodeCoordinates();
   public void setNodeCoordinates(VectorNd coords);
   public void updatePosState();
   public void updateVelState();
   public void updatePosAndVelState();
   public Point3d getPosition();
   
   /**
    * Removes all dependencies with have very low weights
    * ( |w| &lt;= tol )
    * @return true if modified
    */
   public boolean reduceDependencies(double tol);
   
   
}
