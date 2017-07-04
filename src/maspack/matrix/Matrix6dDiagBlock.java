package maspack.matrix;

import maspack.matrix.Matrix.Partition;

public class Matrix6dDiagBlock extends Matrix6dBlock {
   /**
    * Creates a new Matrix6dDiagBlock.
    */
   public Matrix6dDiagBlock() {
      super();
   }

   /**
    * Creates a new Matrix6dDiagBlock with specified diagonal elements.
    */
   public Matrix6dDiagBlock(double m00, double m11, double m22, double m33, 
      double m44, double m55) {
      super();
      set(m00, m11, m22, m33, m44, m55);
   }

   public void set (double m00, double m11, double m22, double m33, double m44,
      double m55) {
      this.m00 = m00;
      this.m11 = m11;
      this.m22 = m22;
      this.m33 = m33;
      this.m44 = m44;
      this.m55 = m55;
   }

   public void set (int i, int j, double val) {
      if (i == j) {
         switch (i) {
            case 0:
               m00 = val;
               return;
            case 1:
               m11 = val;
               return;
            case 2:
               m22 = val;
               return;
            case 3:
               m33 = val;
               return;
            case 4:
               m44 = val;
               return;
            case 5:
               m55 = val;
               return;
            default:
               throw new ArrayIndexOutOfBoundsException ("" + i + "," + j);
         }
      }
   }

   /**
    * {@inheritDoc}
    */
   public boolean valueIsNonZero (int i, int j) {
      return i == j;
   }

   /**
    * {@inheritDoc}
    */
   public int numNonZeroVals() {
      return 6;
   }

   /**
    * {@inheritDoc}
    */
   public int numNonZeroVals (Partition part, int numRows, int numCols) {
      if (numRows > 6 || numCols > 6) {
         throw new IllegalArgumentException (
            "specified sub-matrix is out of bounds");
      }
      if (numRows > numCols) {
         return numCols;
      }
      else {
         return numRows;
      }
   }

   /**
    * {@inheritDoc}
    */
   public void mulAdd (double[] y, int yIdx, double[] x, int xIdx) {
      y[yIdx + 0] += m00 * x[xIdx + 0];
      y[yIdx + 1] += m11 * x[xIdx + 1];
      y[yIdx + 2] += m22 * x[xIdx + 2];
      
      y[yIdx + 3] += m33 * x[xIdx + 3];
      y[yIdx + 4] += m44 * x[xIdx + 4];
      y[yIdx + 5] += m55 * x[xIdx + 5];
   }

   /**
    * {@inheritDoc}
    */
   public void mulTransposeAdd (double[] y, int yIdx, double[] x, int xIdx) {
      y[yIdx + 0] += m00 * x[xIdx + 0];
      y[yIdx + 1] += m11 * x[xIdx + 1];
      y[yIdx + 2] += m22 * x[xIdx + 2];
      
      y[yIdx + 3] += m33 * x[xIdx + 3];
      y[yIdx + 4] += m44 * x[xIdx + 4];
      y[yIdx + 5] += m55 * x[xIdx + 5];
   }

   /**
    * {@inheritDoc}
    */
   public int getBlockCRSIndices (
      int[] colIdxs, int colOff, int[] offsets, Partition part) {

      colIdxs[offsets[0]++] = colOff;
      colIdxs[offsets[1]++] = colOff+1;
      colIdxs[offsets[2]++] = colOff+2;
      
      colIdxs[offsets[3]++] = colOff+3;
      colIdxs[offsets[4]++] = colOff+4;
      colIdxs[offsets[5]++] = colOff+5;
      
      return 6;
   }

   /**
    * {@inheritDoc}
    */
   public void addNumNonZerosByRow (int[] offsets, int idx, Partition part) {
      offsets[idx]++;
      offsets[idx+1]++;
      offsets[idx+2]++;
      
      offsets[idx+3]++;
      offsets[idx+4]++;
      offsets[idx+5]++;
   }

   /**
    * {@inheritDoc}
    */
   public int getBlockCCSIndices (
      int[] rowIdxs, int rowOff, int[] offsets, Partition part) {

      rowIdxs[offsets[0]++] = rowOff;
      rowIdxs[offsets[1]++] = rowOff+1;
      rowIdxs[offsets[2]++] = rowOff+2;
      
      rowIdxs[offsets[3]++] = rowOff+3;
      rowIdxs[offsets[4]++] = rowOff+4;
      rowIdxs[offsets[5]++] = rowOff+5;
      
      return 6;
   }

   /**
    * {@inheritDoc}
    */
   public void addNumNonZerosByCol (int[] offsets, int idx, Partition part) {
      offsets[idx]++;
      offsets[idx+1]++;
      offsets[idx+2]++;
      
      offsets[idx+3]++;
      offsets[idx+4]++;
      offsets[idx+5]++;
   }

   /**
    * {@inheritDoc}
    */
   public int getBlockCRSValues (double[] vals, int[] offsets, Partition part) {
      vals[offsets[0]++] = m00;
      vals[offsets[1]++] = m11;
      vals[offsets[2]++] = m22;
      
      vals[offsets[3]++] = m33;
      vals[offsets[4]++] = m44;
      vals[offsets[5]++] = m55;
      
      return 6;
   }

   /**
    * {@inheritDoc}
    */
   public int getBlockCCSValues (double[] vals, int[] offsets, Partition part) {
      vals[offsets[0]++] = m00;
      vals[offsets[1]++] = m11;
      vals[offsets[2]++] = m22;
      
      vals[offsets[3]++] = m33;
      vals[offsets[4]++] = m44;
      vals[offsets[5]++] = m55;
      
      return 6;
   }

}
