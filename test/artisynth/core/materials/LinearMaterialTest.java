package artisynth.core.materials;

import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix6d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.SymmetricMatrix3d;

public class LinearMaterialTest {

   public void doRandomCorotationTest() {
      
      LinearMaterial mat = new LinearMaterial(150, 0.25, true);
      
      // random F
      Matrix3d F = new Matrix3d();
      F.setDiagonal(10, 0.5, 0.2);
      // F.setRandom();
      F.scale(1.0/Math.cbrt(F.determinant()));  // determinant 1?
      System.out.println("F determinant: " + F.determinant());
      
      System.out.println("F: ");
      System.out.println(F);
      
      // rotate about X by 90
      RotationMatrix3d R = new RotationMatrix3d();
      R.setIdentity();
      
      SolidDeformation def = new SolidDeformation();
      def.setF(F);
      def.setR(R);
      
      SymmetricMatrix3d sigma1 = new SymmetricMatrix3d();
      mat.computeStress(sigma1, def, null, null);
      System.out.println("Stress 1:");
      System.out.println(sigma1.toString("%.2f").replaceAll("-0.00", "0.00"));
      
      Matrix6d tangent1 = new Matrix6d();
      mat.computeTangent(tangent1, sigma1, def, null, null);
      System.out.println("Tangent 1:");
      System.out.println(tangent1.toString("%.2f").replaceAll("-0.00", "0.00"));
      
      // now rotate F, make sure same
      R = RotationMatrix3d.ROT_X_90;
      R.setRandom();
      F.mul(R, F);
      def.setF(F);
      def.setR(R);
      
      RotationMatrix3d Rinv = new RotationMatrix3d(R);
      Rinv.transpose();
      
      Matrix6d rtangent = new Matrix6d();
      TensorUtils.rotateTangent2(rtangent, tangent1, R);
      System.out.println("Rotated tangent 1:");
      System.out.println(rtangent.toString("%.2f").replaceAll("-0.00", "0.00"));
      TensorUtils.rotateTangent2(rtangent, rtangent, Rinv);
      System.out.println("Un-rotated rotated tangent 1:");
      System.out.println(rtangent.toString("%.2f").replaceAll("-0.00", "0.00"));
      
      SymmetricMatrix3d sigma2 = new SymmetricMatrix3d();
      mat.computeStress(sigma2, def, null, null);
      
      System.out.println("Stress 2:");
      System.out.println(sigma2.toString("%.2f").replaceAll("-0.00", "0.00"));
      
      Matrix6d tangent2 = new Matrix6d();
      mat.computeTangent(tangent2, sigma2, def, null, null);
      
      TensorUtils.rotateTangent(tangent2, tangent2, R);
      
      System.out.println("Tangent 2:");
      System.out.println(tangent2.toString("%.2f").replaceAll("-0.00", "0.00"));
      
      // rotate stress back
      sigma2.mulTransposeLeftAndRight(R);
      
      if (!sigma1.epsilonEquals(sigma2, 1e-5)) {
         throw new RuntimeException("Stress test failed");
      }
      
   }
   
   public static void main(String[] args) {
      
      LinearMaterialTest test = new LinearMaterialTest();
      test.doRandomCorotationTest();
      
   }
   
}
