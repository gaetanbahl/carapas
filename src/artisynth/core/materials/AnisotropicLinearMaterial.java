package artisynth.core.materials;

import maspack.matrix.DenseMatrix;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3dBase;
import maspack.matrix.Matrix6d;
import maspack.matrix.Matrix6dBase;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.VectorNd;
import maspack.properties.PropertyList;

public class AnisotropicLinearMaterial extends LinearMaterialBase {

   private static Matrix6d DEFAULT_STIFFNESS_TENSOR = createIsotropicStiffness(LinearMaterial.DEFAULT_E, 
      LinearMaterial.DEFAULT_NU);
   private static VectorNd DEFAULT_STIFFNESS_TENSOR_VEC = toRowMajor(DEFAULT_STIFFNESS_TENSOR);

   private Matrix6d myC;  // anisotropic stiffness matrix

   public static PropertyList myProps =
      new PropertyList (AnisotropicLinearMaterial.class, LinearMaterialBase.class);

   static {
      myProps.add ("stiffnessTensor getRasterizedStiffnessTensor setRasterizedStiffnessTensor", "6x6 anisotropic stiffness tensor,"
         + " in row-major form", DEFAULT_STIFFNESS_TENSOR_VEC, "D36");
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   private static VectorNd toRowMajor(DenseMatrix A) {
      VectorNd vec = new VectorNd(A.rowSize()*A.colSize());
      int idx = 0;
      for (int i=0; i<A.rowSize(); ++i) {
         for (int j=0; j<A.colSize(); ++j) {
            vec.set(idx++, A.get(i, j));
         }
      }
      return vec;
   }
   
   public static Matrix6d createIsotropicStiffness(double E, double nu) {
      Matrix6d D = new Matrix6d();
      double lam = E*nu/((1+nu)*(1-2*nu));
      double mu = E/(2*(1+nu));
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
      return D;
   }

   public AnisotropicLinearMaterial () {
      this(LinearMaterial.DEFAULT_E, LinearMaterial.DEFAULT_NU, LinearMaterial.DEFAULT_COROTATED);
   }

   public AnisotropicLinearMaterial (double E, double nu) {
      this (E, nu, DEFAULT_COROTATED);
   }

   public AnisotropicLinearMaterial (double E, double nu, boolean corotated) {
      super(corotated);
      setStiffnessTensor(createIsotropicStiffness(E, nu));
   }

   public AnisotropicLinearMaterial (Matrix6dBase C) {
      this(C, DEFAULT_COROTATED);
   }

   public AnisotropicLinearMaterial (Matrix6dBase C, boolean corotated) {
      this.myC = new Matrix6d(C);
      setCorotated(corotated);
   }

   public Matrix6d getStiffnessTensor() {
      return myC;
   }
   
   public VectorNd getRasterizedStiffnessTensor() {
      return toRowMajor(myC);
   }
   
   public void setRasterizedStiffnessTensor(VectorNd C) {
      int idx =  0;
      for (int i=0; i<myC.rowSize(); ++i) {
         for (int j=0; j<myC.colSize(); ++j) {
            myC.set(i, j, C.get(idx++));
         }
      }
      notifyHostOfPropertyChange("stiffness tensor");
   }

   public void setStiffnessTensor(Matrix6d C) {
      if (this.myC == null) {
         this.myC = new Matrix6d(C);
      } else {
         this.myC.set(C);
      }
      notifyHostOfPropertyChange("stiffness tensor");
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

      sigma.m00 += myC.m00*Eps.m00 + myC.m01*Eps.m11 + myC.m02*Eps.m22 + 2*myC.m03*Eps.m01 + 2*myC.m04*Eps.m12 + 2*myC.m05*Eps.m02;
      sigma.m11 += myC.m10*Eps.m00 + myC.m11*Eps.m11 + myC.m12*Eps.m22 + 2*myC.m13*Eps.m01 + 2*myC.m14*Eps.m12 + 2*myC.m15*Eps.m02;
      sigma.m22 += myC.m20*Eps.m00 + myC.m21*Eps.m11 + myC.m22*Eps.m22 + 2*myC.m23*Eps.m01 + 2*myC.m24*Eps.m12 + 2*myC.m25*Eps.m02;
      sigma.m01 += myC.m30*Eps.m00 + myC.m31*Eps.m11 + myC.m32*Eps.m22 + 2*myC.m33*Eps.m01 + 2*myC.m34*Eps.m12 + 2*myC.m35*Eps.m02;
      sigma.m12 += myC.m40*Eps.m00 + myC.m41*Eps.m11 + myC.m42*Eps.m22 + 2*myC.m43*Eps.m01 + 2*myC.m44*Eps.m12 + 2*myC.m45*Eps.m02;
      sigma.m02 += myC.m50*Eps.m00 + myC.m51*Eps.m11 + myC.m52*Eps.m22 + 2*myC.m53*Eps.m01 + 2*myC.m54*Eps.m12 + 2*myC.m55*Eps.m02;

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

      SymmetricMatrix3d eps  = new SymmetricMatrix3d();

      Matrix3dBase R = computeStrain(def.getF(), eps, def.getR());

      // multiply
      sigma.m00 = myC.m00*eps.m00 + myC.m01*eps.m11 + myC.m02*eps.m22 + 2*myC.m03*eps.m01 + 2*myC.m04*eps.m12 + 2*myC.m05*eps.m02;
      sigma.m11 = myC.m10*eps.m00 + myC.m11*eps.m11 + myC.m12*eps.m22 + 2*myC.m13*eps.m01 + 2*myC.m14*eps.m12 + 2*myC.m15*eps.m02;
      sigma.m22 = myC.m20*eps.m00 + myC.m21*eps.m11 + myC.m22*eps.m22 + 2*myC.m23*eps.m01 + 2*myC.m24*eps.m12 + 2*myC.m25*eps.m02;
      sigma.m01 = myC.m30*eps.m00 + myC.m31*eps.m11 + myC.m32*eps.m22 + 2*myC.m33*eps.m01 + 2*myC.m34*eps.m12 + 2*myC.m35*eps.m02;
      sigma.m12 = myC.m40*eps.m00 + myC.m41*eps.m11 + myC.m42*eps.m22 + 2*myC.m43*eps.m01 + 2*myC.m44*eps.m12 + 2*myC.m45*eps.m02;
      sigma.m02 = myC.m50*eps.m00 + myC.m51*eps.m11 + myC.m52*eps.m22 + 2*myC.m53*eps.m01 + 2*myC.m54*eps.m12 + 2*myC.m55*eps.m02;
      sigma.m10 = sigma.m01;
      sigma.m20 = sigma.m02;
      sigma.m21 = sigma.m12;

      // rotate stress back to original frame
      if (isCorotated()) {
         sigma.mulLeftAndTransposeRight (R);
      }
   }

   public void computeTangent (
      Matrix6d D, SymmetricMatrix3d stress, SolidDeformation def, 
      Matrix3d Q, FemMaterial baseMat) {

      D.set(myC);

      if (isCorotated()) {

         // need to rotate this tensor from linear frame into material one
         Matrix3d F = def.getF();
         Matrix3dBase dR = def.getR();
         RotationMatrix3d R = new RotationMatrix3d();
         if (dR == null) {
            R = computeRotation(F, null);
         } else {
            R.set(dR);
         }

         // R rotates from linear frame to the material one. Transpose
         // of R rotates from material frame to linear one.
         R.transpose();
         TensorUtils.rotateTangent2 (D, D, R);
      }
   }

   public boolean equals (FemMaterial mat) {
      if (!(mat instanceof AnisotropicLinearMaterial)) {
         return false;
      }
      AnisotropicLinearMaterial linm = (AnisotropicLinearMaterial)mat;
      if (!myC.equals(linm)) {
         return false;
      }
      else {
         return super.equals (mat);
      }
   }

   public AnisotropicLinearMaterial clone() {
      AnisotropicLinearMaterial mat = (AnisotropicLinearMaterial)super.clone();
      mat.myC = myC.clone();
      return mat;
   }

   @Override
   public void scaleDistance (double s) {
      if (s != 1) {
         super.scaleDistance (s);
         myC.scale(1.0/s);
      }
   }

   @Override
   public void scaleMass (double s) {
      if (s != 1) {
         super.scaleMass (s);
         myC.scale(s);
      }
   }

}
