package artisynth.core.materials;

import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3dBase;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.SVDecomposition3d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.properties.PropertyList;
import maspack.properties.PropertyMode;
import maspack.properties.PropertyUtils;

/**
 * Base material for both isotropic and anisotropic linear materials
 * @author Antonio
 *
 */
public abstract class LinearMaterialBase extends FemMaterial {

   public static PropertyList myProps =
      new PropertyList (LinearMaterialBase.class, FemMaterial.class);

   protected static boolean DEFAULT_COROTATED = true;

   private boolean myCorotated = DEFAULT_COROTATED;
   PropertyMode myCorotatedMode = PropertyMode.Inherited;

   static {
      myProps.addInheritable (
         "corotated:Inherited isCorotated",
         "apply corotation", DEFAULT_COROTATED);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   protected LinearMaterialBase (){
   }
   
   protected LinearMaterialBase(boolean corotated) {
      myCorotated = corotated;
   }

   public boolean isInvertible() {
      return true;
   }   

   @Override
   public boolean isLinear() {
      return true;
   }

   public synchronized void setCorotated (boolean enable) {
      myCorotated = enable;
      myCorotatedMode =
         PropertyUtils.propagateValue (this, "corotated", myCorotated, myCorotatedMode);
      notifyHostOfPropertyChange();
   }

   public boolean isCorotated() {
      return myCorotated;
   }

   public void setCorotatedMode (PropertyMode mode) {
      myCorotatedMode =
         PropertyUtils.setModeAndUpdate (this, "corotated", myCorotatedMode, mode);
   }

   public PropertyMode getCorotatedMode() {
      return myCorotatedMode;
   }

   /** 
    * Computes the Cauchy stress from Cauchy strain and adds it to and existing
    * stress.
    * 
    * @param sigma value to which stress should be added
    * @param eps Cauchy stress
    * @param R (optional) Co-Rotation matrix, if any
    */
   public abstract void addStress (
      SymmetricMatrix3d sigma, SymmetricMatrix3d eps, Matrix3dBase R);

   private SVDecomposition3d mySVD = null;
   protected RotationMatrix3d computeRotation(Matrix3d F, SymmetricMatrix3d P) {
      if (mySVD == null) {
         mySVD = new SVDecomposition3d();
      }
      RotationMatrix3d R = new RotationMatrix3d();
      mySVD.polarDecomposition (R, P, F);
      return R;
   }
   
   /**
    * Computes strain
    * @param def
    * @param eps
    * @param rotation matrix if corotated
    * @return rotation matrix if corotated
    */
   protected Matrix3dBase computeStrain(Matrix3d F, SymmetricMatrix3d eps,
      Matrix3dBase R) {

      if (myCorotated) {
         if (R == null) {
            R = computeRotation(F, eps);
         } else {
            // rotate F
            Matrix3d A = new Matrix3d();
            A.mulTransposeLeft(R, F);
            eps.setSymmetric(A);
         }
      } else {
         eps.setSymmetric (F);
      }

      // subtract I to compute Cauchy strain in sigma
      eps.m00 -= 1;
      eps.m11 -= 1;
      eps.m22 -= 1;
      
      return R;
   }

   public boolean equals (FemMaterial mat) {
      if (!(mat instanceof LinearMaterialBase)) {
         return false;
      }
      LinearMaterialBase linm = (LinearMaterialBase)mat;
      if (myCorotated != linm.myCorotated) {
         return false;
      } else {
         return super.equals (mat);
      }
   }

   public LinearMaterialBase clone() {
      LinearMaterialBase mat = (LinearMaterialBase)super.clone();
      return mat;
   }

   @Override
   public void scaleDistance (double s) {
      if (s != 1) {
         super.scaleDistance (s);
      }
   }

   @Override
   public void scaleMass (double s) {
      if (s != 1) {
         super.scaleMass (s);
      }
   }

}
