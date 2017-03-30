package artisynth.core.materials;

import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3dBase;
import maspack.matrix.Matrix6d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.properties.PropertyList;
import maspack.properties.PropertyMode;
import maspack.properties.PropertyUtils;

public class LinearMaterial extends LinearMaterialBase {

   public static PropertyList myProps =
      new PropertyList (LinearMaterial.class, LinearMaterialBase.class);

   protected static double DEFAULT_NU = 0.33;
   protected static double DEFAULT_E = 500000;

   private double myNu = DEFAULT_NU;
   private double myE = DEFAULT_E;

   PropertyMode myNuMode = PropertyMode.Inherited;
   PropertyMode myEMode = PropertyMode.Inherited;

   static {
      myProps.addInheritable (
         "YoungsModulus:Inherited", "Youngs modulus", DEFAULT_E, "[0,inf]");
      myProps.addInheritable (
         "PoissonsRatio:Inherited", "Poissons ratio", DEFAULT_NU, "[-1,0.5]");
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public LinearMaterial (){
      this(DEFAULT_E, DEFAULT_NU, DEFAULT_COROTATED);
   }

   public LinearMaterial (double E, double nu) {
      this (E, nu, DEFAULT_COROTATED);
   }

   public LinearMaterial (double E, double nu, boolean corotated) {
      super (corotated);
      setYoungsModulus (E);
      setPoissonsRatio (nu);
   }

   public synchronized void setPoissonsRatio (double nu) {
      myNu = nu;
      myNuMode =
         PropertyUtils.propagateValue (this, "PoissonsRatio", myNu, myNuMode);
      notifyHostOfPropertyChange();
   }

   public double getPoissonsRatio() {
      return myNu;
   }

   public void setPoissonsRatioMode (PropertyMode mode) {
      myNuMode =
         PropertyUtils.setModeAndUpdate (this, "PoissonsRatio", myNuMode, mode);
   }

   public PropertyMode getPoissonsRatioMode() {
      return myNuMode;
   }

   public synchronized void setYoungsModulus (double E) {
      myE = E;
      myEMode =
         PropertyUtils.propagateValue (this, "YoungsModulus", myE, myEMode);
      notifyHostOfPropertyChange();
   }

   public double getYoungsModulus() {
      return myE;
   }

   public void setYoungsModulusMode (PropertyMode mode) {
      myEMode =
         PropertyUtils.setModeAndUpdate (this, "YoungsModulus", myEMode, mode);
   }

   public PropertyMode getYoungsModulusMode() {
      return myEMode;
   }

   @Override
   public void addStress (
      SymmetricMatrix3d sigma, SymmetricMatrix3d eps, Matrix3dBase R) {

      // save for the rotated case
      double m00 = sigma.m00;
      double m11 = sigma.m11;
      double m22 = sigma.m22;
      double m01 = sigma.m01;
      double m02 = sigma.m02;
      double m12 = sigma.m12;

      if (R != null) {
         sigma.setZero();
      }

      // lam and mu are the first and second Lame parameters
      double lam = myE*myNu/((1+myNu)*(1-2*myNu));
      double mu = myE/(2*(1+myNu));

      double lamtrEps = lam*eps.trace();
      sigma.scaledAdd (2*mu, eps);
      sigma.m00 += lamtrEps;
      sigma.m11 += lamtrEps;
      sigma.m22 += lamtrEps;

      if (R != null) {
         sigma.mulLeftAndTransposeRight (R);

         sigma.m00 += m00;
         sigma.m11 += m11;
         sigma.m22 += m22;

         sigma.m01 += m01;
         sigma.m02 += m02;
         sigma.m12 += m12;

         sigma.m10 += m01;
         sigma.m20 += m02;
         sigma.m21 += m12;
      }
      
   }

   public void computeStress (
      SymmetricMatrix3d sigma, SolidDeformation def, Matrix3d Q,
      FemMaterial baseMat) {

      Matrix3dBase R = computeStrain(def.getF(), sigma, def.getR());
      
      // lam and mu are the first and second Lame parameters
      double lam = myE*myNu/((1+myNu)*(1-2*myNu));
      double mu = myE/(2*(1+myNu));

      // convert sigma from strain to stress
      // sigma = 2*mu*eps + lamda*trace(eps)*I
      double lamtrEps = lam*(sigma.m00+sigma.m11+sigma.m22);
      sigma.scale (2*mu);
      sigma.m00 += lamtrEps;
      sigma.m11 += lamtrEps;
      sigma.m22 += lamtrEps;

      // rotate stress back to original frame
      if (isCorotated()) {
         sigma.mulLeftAndTransposeRight (R);
      }
      
   }

   public void computeTangent (
      Matrix6d D, SymmetricMatrix3d stress, SolidDeformation def, 
      Matrix3d Q, FemMaterial baseMat) {

      D.setZero();

      // lam and mu are the first and second Lame parameters
      double lam = myE*myNu/((1+myNu)*(1-2*myNu));
      double mu = myE/(2*(1+myNu));
      D.m00 = lam + 2*mu;
      D.m01 = lam;
      D.m02 = lam;
      D.m10 = lam;
      D.m11 = lam + 2*mu;
      D.m12 = lam;
      D.m20 = lam;
      D.m21 = lam;
      D.m22 = lam + 2*mu;
      D.m33 = mu;
      D.m44 = mu;
      D.m55 = mu;

      // XXX isotropic materials are invariant under rotation
      //      if (myCorotated) {
      //
      //         // need to rotate this tensor from linear frame into material one
      //         Matrix3d F = def.getF();
      //         RotationMatrix3d R = new RotationMatrix3d();
      //
      //         Matrix3dBase dR = def.getR();
      //         if (dR == null) {
      //            if (mySVD == null) {
      //               mySVD = new SVDecomposition3d();
      //            }
      //            // use sigma to store P; this will be converted to Cauchy strain
      //            mySVD.polarDecomposition (R, (Matrix3d)null, F);
      //         } else {
      //            R.set(dR);
      //         }
      //
      //         // R rotates from linear frame to the material one. Transpose
      //         // of R rotates from material frame to linear one.
      //         R.transpose();
      //         TensorUtils.rotateTangent2 (D, D, R);
      //      }
   }

   public boolean equals (FemMaterial mat) {
      if (!(mat instanceof LinearMaterial)) {
         return false;
      }
      LinearMaterial linm = (LinearMaterial)mat;
      if (myNu != linm.myNu ||
         myE != linm.myE) {
         return false;
      }
      else {
         return super.equals (mat);
      }
   }

   public LinearMaterial clone() {
      LinearMaterial mat = (LinearMaterial)super.clone();
      return mat;
   }

   @Override
   public void scaleDistance (double s) {
      if (s != 1) {
         super.scaleDistance (s);
         setYoungsModulus (myE/s);
      }
   }

   @Override
   public void scaleMass (double s) {
      if (s != 1) {
         super.scaleMass (s);
         setYoungsModulus (myE*s);
      }
   }

}
