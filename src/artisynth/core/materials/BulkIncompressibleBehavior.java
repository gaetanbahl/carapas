package artisynth.core.materials;

import maspack.matrix.Matrix6d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.util.InternalErrorException;

/**
 * Incompressible behaviour based on Bulk Modulus, allows either quadratic or
 * logarithmic potential function.
 */
public class BulkIncompressibleBehavior implements IncompressibleBehavior {

   public static final double DEFAULT_KAPPA = 100000;
   public static final BulkPotential DEFAULT_BULK_POTENTIAL = BulkPotential.QUADRATIC;
   
   private double myKappa = DEFAULT_KAPPA; // bulk modulus
   protected BulkPotential myBulkPotential = DEFAULT_BULK_POTENTIAL;

   public enum BulkPotential {
      QUADRATIC,
      LOGARITHMIC
   };
         
   public BulkIncompressibleBehavior() {
      this(DEFAULT_BULK_POTENTIAL, DEFAULT_KAPPA);
   }
   
   public BulkIncompressibleBehavior(BulkPotential potential, double kappa) {
      this.myBulkPotential = potential;
      this.myKappa = kappa;
   }
   
   public void setBulkPotential(BulkPotential potential) {
      myBulkPotential = potential;
   }
   
   public BulkPotential getBulkPotential() {
      return myBulkPotential;
   }
   
   public void setBulkModulus(double nu) {
      myKappa = nu;
   }
   
   public double getBulkModulus() {
      return myKappa;
   }
   
   @Override
   public double getEffectiveModulus(double J) {
      switch (myBulkPotential) {
         case QUADRATIC: {
            return myKappa;
         }
         case LOGARITHMIC: {
            return myKappa*(1-Math.log(J))/(J*J);
         }
         default: {
            throw new InternalErrorException (
               "Unimplemented potential " + myBulkPotential);
         }
      }
   }

   @Override
   public double getEffectivePressure(double J) {
      switch (myBulkPotential) {
         case QUADRATIC: {
            return myKappa*(J-1);
         }
         case LOGARITHMIC: {
            return myKappa*(Math.log(J))/J;
         }
         default: {
            throw new InternalErrorException (
               "Unimplemented potential " + myBulkPotential);
         }
      }
   }
   
   @Override
   public boolean equals(Object obj) {
      if (obj == this) {
         return true;
      }
      if (obj == null || !(obj instanceof BulkIncompressibleBehavior)) {
         return false;
      }
      
      BulkIncompressibleBehavior other = (BulkIncompressibleBehavior)obj;
      if (other.myBulkPotential != myBulkPotential) {
         return false;
      }
      if (other.myKappa != myKappa) {
         return false;
      }
      return true;
   }

   @Override
   public void computePressureStress(SymmetricMatrix3d sigma, double p) {
      sigma.setZero();
      sigma.m00 = p;
      sigma.m11 = p;
      sigma.m22 = p;
   }

   @Override
   public void computePressureTangent(Matrix6d D, double p) {
      D.setZero();
      TensorUtils.addScaledIdentityProduct (D, p);
      TensorUtils.addScaledIdentity (D, -2*p);
      D.setLowerToUpper();
   }

}
