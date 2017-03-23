package artisynth.core.materials;

import java.util.Arrays;

/**
 * Material field based solely on subvolume and coordinate indices, such
 * as might be defined by integration points within a finite element volume.
 */
public class IndexedMaterialField implements MaterialField {

   private int dims;
   private double[] vals;
   private int[] subvolOffsets;

   /**
    * Constructs based on a populated array of values, ordered by:
    * <pre>
    * MaterialCoordinate coord = ...
    * int idx = subvolOffsets[coord.getSubvolumeIndex()] + dims*coord.getCoordinateIndex() + dim;
    * </pre>
    * i.e. sorted by subvolume, then coordinate index, then dim.  The last subvolumeOffset value
    * should be the total number of values in vals.
    * @param dims
    * @param subvolOffsets
    * @param vals
    */
   public IndexedMaterialField(int dims, int[] subvolOffsets, double[] vals) {
      this.dims = dims;
      this.subvolOffsets = subvolOffsets;
      this.vals = vals;
   }

   @Override
   public double size() {
      return dims;
   }

   @Override
   /**
    * Evaluates the field at the given coordinate based on subvolume and coordinate indices.  
    * If the subvolume index is unknown, returns 0.  If the coordinate index is -1,
    * returns the average of values across the subvolume.
    */
   public double eval(MaterialCoordinate coord, int dim) {
      int sidx = coord.getSubvolumeIndex();
      int cidx = coord.getCoordinateIndex();

      if (sidx < 0) {
         return 0;
      } else if (cidx == -1) {
         double s = 0;
         for (int i=subvolOffsets[sidx]+dim; i<subvolOffsets[sidx+1]; i+=dims) {
            s += vals[i];
         }
         return s;
      }
      
      int idx = subvolOffsets[sidx] + dims*cidx+dim;
      return vals[idx];
   }

   @Override
   public boolean equals(Object obj) {
      if (obj == this) {
         return true;
      }
      if (obj == null || !(obj instanceof IndexedMaterialField)) {
         return false;
      }
      IndexedMaterialField other = (IndexedMaterialField)obj;
      if (other.dims != dims) {
         return false;
      }
      if (!Arrays.equals(subvolOffsets, other.subvolOffsets)) {
         return false;
      }
      if (!Arrays.equals(vals, other.vals)) {
         return false;
      }
      return true;
   }
   
   @Override
   public IndexedMaterialField clone() {
      IndexedMaterialField out;
      try {
         out = (IndexedMaterialField)super.clone();
      } catch (CloneNotSupportedException e) {
         e.printStackTrace();
         return null;
      }
      out.dims = dims;
      out.vals = Arrays.copyOf(vals, vals.length);
      out.subvolOffsets = Arrays.copyOf(subvolOffsets, subvolOffsets.length);
      return out;
   }

}
