package artisynth.core.materials;

import maspack.matrix.Matrix6d;
import maspack.matrix.SymmetricMatrix3d;

/**
 * Interface for describing incompressible material behaviour
 */
public interface IncompressibleBehavior {

   public double getEffectiveModulus(double J);
   public double getEffectivePressure(double J);
   
   public void computePressureStress(SymmetricMatrix3d sigma, double p);
   public void computePressureTangent(Matrix6d D, double p);
   
}
