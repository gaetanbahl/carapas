package artisynth.core.materials;

import artisynth.core.materials.BulkIncompressibleBehavior.BulkPotential;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix6d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.properties.PropertyList;
import maspack.properties.PropertyMode;
import maspack.properties.PropertyUtils;

public class IncompressibleMaterial extends FemMaterial {
   
   PropertyMode myKappaMode = PropertyMode.Inherited;
   PropertyMode myBulkPotentialMode = PropertyMode.Inherited;
   
   public static PropertyList myProps =
      new PropertyList(IncompressibleMaterial.class, FemMaterial.class);

   public enum BulkPotential {
      QUADRATIC,
      LOGARITHMIC
   };
   
   static {
      myProps.addInheritable (
         "bulkModulus:Inherited", "Bulk modulus", 
         BulkIncompressibleBehavior.DEFAULT_KAPPA);
      myProps.addInheritable ("bulkPotential:Inherited", "Incompressibility potential function",
         BulkIncompressibleBehavior.DEFAULT_BULK_POTENTIAL);
   }
   
   BulkIncompressibleBehavior myIncompBehaviour;

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   public IncompressibleMaterial() {
      super();
      myIncompBehaviour = new BulkIncompressibleBehavior();
   }
   
   public IncompressibleMaterial (double kappa) {
      this();
      setBulkModulus (kappa);
   }

   public synchronized void setBulkModulus (double nu) {
      myIncompBehaviour.setBulkModulus(nu);
      myKappaMode =
         PropertyUtils.propagateValue (this, "bulkModulus", nu, myKappaMode);
      notifyHostOfPropertyChange();
   }

   public double getBulkModulus() {
      return myIncompBehaviour.getBulkModulus();
   }

   public void setBulkModulusMode (PropertyMode mode) {
      myKappaMode =
         PropertyUtils.setModeAndUpdate (this, "bulkModulus", myKappaMode, mode);
   }

   public PropertyMode getBulkModulusMode() {
      return myKappaMode;
   }

   public synchronized void setBulkPotential(BulkIncompressibleBehavior.BulkPotential potential) {
      myIncompBehaviour.setBulkPotential(potential);
      myBulkPotentialMode =
         PropertyUtils.propagateValue (
            this, "bulkPotential", potential, myBulkPotentialMode);
      notifyHostOfPropertyChange();
   }
   
   public synchronized void setBulkPotential (BulkPotential potential) {
      switch (potential) {
         case LOGARITHMIC:
            setBulkPotential(BulkIncompressibleBehavior.BulkPotential.LOGARITHMIC);
            break;
         case QUADRATIC:
            setBulkPotential(BulkIncompressibleBehavior.BulkPotential.QUADRATIC);
         default:
            break;
         
      }
   }

   public BulkPotential getBulkPotential() {
      BulkIncompressibleBehavior.BulkPotential oldPotential = myIncompBehaviour.getBulkPotential();
      switch (oldPotential) {
         case LOGARITHMIC:
            return BulkPotential.LOGARITHMIC;
         case QUADRATIC:
            return BulkPotential.QUADRATIC;
         default:
      }
      return null;
   }

   public void setBulkPotentialMode (PropertyMode mode) {
      myBulkPotentialMode =
         PropertyUtils.setModeAndUpdate (
            this, "bulkPotential", myBulkPotentialMode, mode);
   }

   public PropertyMode getBulkPotentialMode() {
      return myBulkPotentialMode;
   }

   public double getEffectiveModulus (double J) {
      return myIncompBehaviour.getEffectiveModulus(J);
   }

   public double getEffectivePressure (double J) {
      return myIncompBehaviour.getEffectivePressure(J);
   }

   public boolean isIncompressible() {
      return true;
   }
   
   @Override
   public BulkIncompressibleBehavior getIncompressibleBehavior() {
      return myIncompBehaviour;
   }

   public boolean equals (FemMaterial mat) {
      if (!(mat instanceof IncompressibleMaterial)) {
         return false;
      }
      IncompressibleMaterial imat = (IncompressibleMaterial)mat;
      if (myIncompBehaviour.equals(imat.myIncompBehaviour)) {
         return false;
      }
      return super.equals (mat);
   }

   // Sanchez, March 27, 2013
   // useful for separating incompressibility stuff from computeStressAndStiffness() function
   public void computePressureStress(SymmetricMatrix3d sigma, double p) {
      myIncompBehaviour.computePressureStress(sigma, p);
   }
   
   public void addPressureStress(SymmetricMatrix3d sigma, double p) {
      sigma.m00 += p;
      sigma.m11 += p;
      sigma.m22 += p;
   }

   public void computePressureTangent(Matrix6d D, double p) {
      myIncompBehaviour.computePressureTangent(D, p);
   }
   
   public void addPressureTangent(Matrix6d D, double p) {
      TensorUtils.addScaledIdentityProduct (D, p);
      TensorUtils.addScaledIdentity (D, -2*p);
   }

   public void computeStress (
      SymmetricMatrix3d sigma, SolidDeformation def, Matrix3d Q,
      FemMaterial baseMat) {

      double avgp = def.getAveragePressure();
      myIncompBehaviour.computePressureStress(sigma, avgp);
   }
   
   public void computeTangent (
      Matrix6d D, SymmetricMatrix3d stress, SolidDeformation def, 
      Matrix3d Q, FemMaterial baseMat) {

      // mean pressure
      double p = def.getAveragePressure();      
      myIncompBehaviour.computePressureTangent(D, p);
   }

   @Override
   public void scaleDistance (double s) {
      if (s != 1) {
         super.scaleDistance (s);
         setBulkModulus (getBulkModulus()/s);
      }
   }

   @Override
   public void scaleMass (double s) {
      if (s != 1) {
         super.scaleMass (s);
         setBulkModulus (getBulkModulus()*s);
      }
   }

   @Override
   public boolean isInvertible() {
      // right now, true only with a quadratic bulk potential,
      // and for IncompressibleMaterial specifically; not any
      // of the base classes
      return (getClass() == IncompressibleMaterial.class &&
              getBulkPotential() == BulkPotential.QUADRATIC);
   }

}
   
   
