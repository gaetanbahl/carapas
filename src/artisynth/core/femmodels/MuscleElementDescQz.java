/**
 * Copyright (c) 2014, by the Authors: John E Lloyd (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.core.femmodels;

import artisynth.core.modelbase.TransformGeometryContext;
import maspack.geometry.GeometryTransformer;
import maspack.matrix.Matrix3d;
import maspack.matrix.Vector3d;

/**
 * Uses the z-axis of frame Q supplied when computed stress and tangent.
 */
public class MuscleElementDescQz extends MuscleElementDesc {
 
   public MuscleElementDescQz () {
      super();
   }

   public MuscleElementDescQz (FemElement3d elem) {
      this();
      setElement (elem);
   }

   @Override
   public void transformGeometry(
      GeometryTransformer gtr, TransformGeometryContext context, int flags) {
   }

   private Vector3d getMuscleDirection(IntegrationData3d dt) {
      if (dt == null) {
         return null;
      }
      
      Matrix3d frame = dt.getFrame();
      if (frame == null) {
         return null;
      }
      
      Vector3d dir = new Vector3d();
      frame.getColumn(2, dir);
      return dir;
   }
   
   @Override
   public Vector3d getMuscleDirection(IntegrationPoint3d pnt) {
      IntegrationData3d dt = null;
      if (pnt == myElement.getWarpingPoint()) {
         dt = myElement.getWarpingData();
      } else {
         dt = myElement.getIntegrationData()[pnt.getNumber()];
      }
      return getMuscleDirection(dt);
   }

   @Override
   public Vector3d getMuscleDirection(int ipntIdx) {
      IntegrationData3d dt = myElement.getIntegrationData()[ipntIdx];
      return getMuscleDirection(dt);
   }

  

}
