package artisynth.core.materials;

import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3dBase;
import maspack.matrix.Matrix6d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.properties.PropertyList;
import maspack.properties.PropertyMode;

/**
 * Mainly used for debugging, delegates to a linear material
 */
public class LinearishMaterial extends FemMaterial {
   
   LinearMaterial base;
   
   public static PropertyList myProps =
      new PropertyList (LinearishMaterial.class, FemMaterial.class);
   
   static {
      myProps.addInheritable (
         "YoungsModulus:Inherited", "Youngs modulus", LinearMaterial.DEFAULT_E, "[0,inf]");
      myProps.addInheritable (
         "PoissonsRatio:Inherited", "Poissons ratio", LinearMaterial.DEFAULT_NU, "[-1,0.5]");
      myProps.addInheritable (
         "corotated:Inherited isCorotated",
         "apply corotation", LinearMaterial.DEFAULT_COROTATED);
   }
   
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public LinearishMaterial () {
      base = new LinearMaterial();
   }

   public LinearishMaterial (double E, double nu) {
      base = new LinearMaterial (E, nu, /*corotated=*/true);
   }

   public LinearishMaterial (double E, double nu, boolean corotated) {
      base = new LinearMaterial(E, nu, corotated);
   }
   
   public boolean isInvertible() {
      return true;
   }   
   
   @Override
   public boolean isLinear() {
      return true;
   }

   public synchronized void setPoissonsRatio (double nu) {
      base.setPoissonsRatio(nu);
      notifyHostOfPropertyChange();
   }

   public double getPoissonsRatio() {
      return base.getPoissonsRatio();
   }

   public void setPoissonsRatioMode (PropertyMode mode) {
      base.setPoissonsRatioMode(mode);
   }

   public PropertyMode getPoissonsRatioMode() {
      return base.getPoissonsRatioMode();
   }

   public synchronized void setYoungsModulus (double E) {
     base.setYoungsModulus(E);
      notifyHostOfPropertyChange();
   }

   public double getYoungsModulus() {
      return base.getYoungsModulus();
   }

   public void setYoungsModulusMode (PropertyMode mode) {
      base.setYoungsModulusMode(mode);
   }

   public PropertyMode getYoungsModulusMode() {
      return base.getYoungsModulusMode();
   }

   public synchronized void setCorotated (boolean enable) {
      base.setCorotated(enable);
      notifyHostOfPropertyChange();
   }

   public boolean isCorotated() {
      return base.isCorotated();
   }

   public void setCorotatedMode (PropertyMode mode) {
      base.setCorotatedMode(mode);
   }

   public PropertyMode getCorotatedMode() {
      return base.getCorotatedMode();
   }

   /** 
    * Computes the Cauchy stress from Cauchy strain and adds it to and existing
    * stress.
    * 
    * @param sigma value to which stress should be added
    * @param Eps Cauchy stress
    * @param R (optional) Co-Rotation matrix, if any
    */
   public void addStress (
      SymmetricMatrix3d sigma, SymmetricMatrix3d Eps, Matrix3dBase R) {

      base.addStress(sigma, Eps, R);
   }

   public void computeStress (
      SymmetricMatrix3d sigma, SolidDeformation def, Matrix3d Q,
      FemMaterial baseMat) {

      base.computeStress(sigma, def, Q, baseMat);
   }

   public void computeTangent (
      Matrix6d D, SymmetricMatrix3d stress, SolidDeformation def, 
      Matrix3d Q, FemMaterial baseMat) {

      base.computeTangent(D, stress, def, Q, baseMat);
   }

   public boolean equals (FemMaterial mat) {
      if (!(mat instanceof LinearishMaterial)) {
         return false;
      }
      LinearishMaterial linm = (LinearishMaterial)mat;
      return base.equals(linm.base);
   }

   public LinearishMaterial clone() {
      LinearishMaterial mat = (LinearishMaterial)super.clone();
      mat.base = base.clone();
      return mat;
   }

   @Override
   public void scaleDistance (double s) {
      base.scaleDistance(s);
   }

   @Override
   public void scaleMass (double s) {
      base.scaleMass(s);
   }
}
