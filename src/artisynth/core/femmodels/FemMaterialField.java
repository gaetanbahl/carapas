package artisynth.core.femmodels;

import java.util.Arrays;

import artisynth.core.materials.MaterialCoordinate;
import artisynth.core.modelbase.RenderableComponentBase;
import maspack.properties.PropertyList;
import maspack.render.PointRenderProps;
import maspack.render.RenderList;
import maspack.render.RenderObject;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.ColorMixing;
import maspack.render.color.ColorMap;
import maspack.render.color.JetColorMap;
import maspack.util.DoubleInterval;
import maspack.util.DynamicDoubleArray;

/**
 * Implementation of a material field defined by a FEM model,
 * evaluates the field at all integration points within the FEM.
 * 
 * XXX TODO: account for when elements are added or removed from
 * the FEM? Currently only works if field constructed once FEM
 * is finalized.
 */
public class FemMaterialField
   extends RenderableComponentBase implements MaterialFieldComponent {

   FemModel3d myFem;
   DynamicDoubleArray myVals; // stores field values in one long array
   int[] myOffsets;           // map to offset within vals
   int myDims;
   double myScale;
   
   public enum RenderStyle {
      SOLID,
      NORM,
      DIM
   }
   RenderStyle renderStyle;
   ColorMap renderColorMap;
   DoubleInterval renderFieldRange;
   int renderDim;
   RenderObject myRobj;
   boolean myRobjValid;
   
   public static PropertyList myProps =
      new PropertyList (FemMaterialField.class, RenderableComponentBase.class);

   static {
      myProps.add ("scale", "scale factor for entire field", 1.0);
      myProps.add ("renderProps", "render properties", new PointRenderProps());
      myProps.add ("colorMap", "fem material", new JetColorMap());
      myProps.add ("renderStyle", "style of render", RenderStyle.NORM);
      myProps.add ("renderFieldRange", "rendering range of the of the field", new DoubleInterval(0,1));
      myProps.add ("renderDimension", "dimension to render", 0);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   @Override
   public RenderProps createRenderProps() {
      return new PointRenderProps();
   }
   
   public FemMaterialField(FemModel3d fem, int dim) {
      this.myFem = fem;
      this.myDims = dim;
      this.myVals = new DynamicDoubleArray();
      myOffsets = new int[myFem.numElements()];
      Arrays.fill(myOffsets, -1);
      myRobj = null;
      renderFieldRange = new DoubleInterval(0, 1);
      renderStyle = RenderStyle.NORM;
      renderDim = 0;
      myRobjValid = false;
      myScale = 1.0;
   }
   
   public double getScale() {
      return myScale;
   }
   
   public void setScale(double s) {
      myScale = s;
      myRobjValid = false;
   }
   
   public boolean hasNonZeroValue(FemElement3d elem) {
      int offset = myOffsets[elem.getNumber()];
      if (offset < 0) {
         return false;
      }
      int n = elem.numIntegrationPoints()*myDims;
      for (int i=0; i<n; ++i) {
         if (myVals.get(i) != 0) {
            return true;
         }
      }
      return false;
   }
   
   /**
    * Sets the field at the supplied element index
    * to the given array of values
    * @param elem element index
    * @param IntegrationPoint3d pt integration point
    * @param v values
    */
   public void set(FemElement3d elem, IntegrationPoint3d pt, double[] v) {
      if (v.length != myDims) {
         throw new IllegalArgumentException("Dimension mismatch: " + v.length + " vs " + myDims);
      }
      int eidx = elem.getNumber();
      int offset = myOffsets[eidx];
      if (offset < 0) {
         // tag on the end
         offset = myVals.size();
         myOffsets[eidx] = offset;
         myVals.resize(myVals.size() + myDims*(elem.numIntegrationPoints()));
         maybeClearRenderObject();
      } 
      
      // set values
      offset += pt.getNumber()*myDims;
      for (int i=0; i<myDims; ++i) {
         myVals.set(offset+i, v[i]);
      }
      myRobjValid = false;
   }
   
   /**
    * Sets the field at the supplied element and integration point to the
    * supplied value.
    * If this is a multidimensional field, all dimensions are set to this value
    * @param elem element
    * @param pt integration point
    * @param v value
    */
   public void set(FemElement3d elem, IntegrationPoint3d pt, double v) {
      int eidx = elem.getNumber();
      int offset = myOffsets[eidx];
      if (offset < 0) {
         // tag on the end
         offset = myVals.size();
         myOffsets[eidx] = offset;
         myVals.resize(myVals.size() + myDims*(elem.numIntegrationPoints()));
         maybeClearRenderObject();
      } 
      
      // set values
      offset += pt.getNumber()*myDims;
      for (int i=0; i<myDims; ++i) {
         myVals.set(offset+i, v);
      }
      myRobjValid = false;
   }
   
   /**
    * Returns the field value at the specified location and dimension,
    * not including scale
    * @param elem
    * @param pt
    * @param dim
    * @return value
    */
   public double get(FemElement3d elem, IntegrationPoint3d pt, int dim) {
      int eidx = elem.getNumber();
      int offset = myOffsets[eidx];
      if (offset < 0) {
         return 0;
      } 
      
      // set values
      offset += pt.getNumber()*myDims;
      return myVals.get(offset + dim);
   }
   
   /**
    * Sets the field at the supplied element to the supplied value.
    * If this is a multidimensional field, all dimensions are set to this value
    * @param elem element
    * @param v value
    */
   public void set(FemElement3d elem, double v) {
      int eidx = elem.getNumber();
      int offset = myOffsets[eidx];
      if (offset < 0) {
         // tag on the end
         offset = myVals.size();
         myOffsets[eidx] = offset;
         myVals.resize(myVals.size() + myDims*(elem.numIntegrationPoints()));
         maybeClearRenderObject();
      } 
      
      // set values
      int nn = myDims*elem.numIntegrationPoints();
      for (int i=0; i<nn; ++i) {
         myVals.set(offset+i, v);
      }
   }
   
   private void maybeClearRenderObject() {
      if (myRobj != null) {
         myRobj.dispose();
      }
      myRobj = null;
   }
   
   @Override
   public double size() {
      return myDims;
   }
   

   /**
    * Evaluates the field value at the specified location and dimension,
    * including scale
    * @param elem
    * @param pt
    * @param dim
    * @return value
    */
   public double eval(FemElement3d elem, IntegrationPoint3d pt, int dim) {
      return get(elem, pt, dim)*myScale;
   }

   @Override
   public double eval(MaterialCoordinate coord, int dim) {
      int eidx = coord.getSubvolumeIndex();
      int cidx = coord.getCoordinateIndex();
      int offset = myOffsets[eidx];
      if (offset < 0) {
         return 0;
      }
      return myVals.get(offset+myDims*cidx+dim)*myScale;
   }

   @Override
   public FemMaterialField clone() {
      FemMaterialField copy = null;;
      try {
         copy = (FemMaterialField)super.clone();
      } catch (CloneNotSupportedException e) {
         throw new RuntimeException(e);
      }
      copy.myScale = myScale;
      copy.myDims = myDims;
      copy.myVals = myVals.clone();
      copy.myOffsets = Arrays.copyOf(myOffsets, myOffsets.length);
      copy.myRobj = null;
      copy.renderColorMap = renderColorMap;
      copy.renderDim = renderDim;
      copy.renderFieldRange = renderFieldRange.clone();
      return copy;
   }
   
   public void setColorMap(ColorMap map) {
      if (!map.equals(renderColorMap)) {
         this.renderColorMap = map;
         myRobjValid = false;
      }
   }
   
   public ColorMap getColorMap() {
      return renderColorMap;
   }
   
   public DoubleInterval getRenderFieldRange() {
      return renderFieldRange;
   }
   
   public void setRenderFieldRange(DoubleInterval range) {
      if (!renderFieldRange.equals(range)) {
         renderFieldRange = range;
         myRobjValid = false;
      }
   }
   
   public void setRenderDimension(int dim) {
      if (renderDim != dim) {
         renderDim = dim;
         myRobjValid = false;
      }
   }
   
   public int getRenderDimension() {
      return renderDim;
   }
   
   public RenderStyle getRenderStyle() {
      return renderStyle;
   }
   
   public void setRenderStyle(RenderStyle style) {
      if (style != renderStyle) {
         renderStyle = style;
         myRobjValid = false;
      }
   }
   
   private RenderObject buildRenderObject() {
      RenderObject robj = new RenderObject();
      
      for (int i=0; i<myOffsets.length; ++i) {
         int offset = myOffsets[i];
         if (offset >= 0) {
            
            FemElement3d elem = myFem.getElementByNumber(i);
            IntegrationPoint3d[] ipnts = elem.getIntegrationPoints();

            for (int k=0; i<ipnts.length; ++k) {
               float[] rcoords = new float[3];
               float[] color = new float[4];
               
               ipnts[k].computeCoordsForRender(rcoords, elem.getNodes());
               robj.addPosition(rcoords);
               robj.addColor(color);
               int vidx = robj.addVertex();
               robj.addPoint(vidx);
               
               switch (renderStyle) {
                  case NORM: {
                     // set color to norm of field
                     double c = 0;
                     for (int j=0; j<myDims; ++j) {
                        double v = myVals.get(offset+j);
                        c += v*v;
                     }
                     c = Math.sqrt(c)*myScale;
                     // map to [0,1]
                     c = (c-renderFieldRange.getLowerBound())/renderFieldRange.getRange();
                     if (c < 0) {
                        c = 0;
                     } else if (c > 1) {
                        c = 1;
                     }
                     renderColorMap.getRGB(c, color);
                     color[3] = 1;
                     break;
                  }
                  case DIM : {
                     // set color to norm of field
                     double c = myVals.get(offset+renderDim)*myScale;
                     c = (c-renderFieldRange.getLowerBound())/renderFieldRange.getRange();
                     if (c < 0) {
                        c = 0;
                     } else if (c > 1) {
                        c = 1;
                     }
                     renderColorMap.getRGB(c, color);
                     break;
                  }
                  default:
                     break;
               }
               offset += myDims;
            }
         }
      }
      
      return robj;
   }
   
   private void updateRenderObject(RenderObject robj) {
      
      int vidx = 0;
      
      for (int i=0; i<myOffsets.length; ++i) {
         int offset = myOffsets[i];
         if (offset >= 0) {
            FemElement3d elem = myFem.getElementByNumber(i);
            IntegrationPoint3d[] ipnts = elem.getIntegrationPoints();
            
            for (int k=0; i<ipnts.length; ++k) {
               float[] rcoords = robj.getPosition(vidx);
               byte[] color = robj.getColor(vidx);
               
               ipnts[k].computeCoordsForRender(rcoords, elem.getNodes());
               
               switch (renderStyle) {
                  case NORM: {
                     // set color to norm of field
                     double c = 0;
                     for (int j=0; j<myDims; ++j) {
                        double v = myVals.get(offset+j);
                        c += v*v;
                     }
                     c = Math.sqrt(c)*myScale;
                     // map to [0,1]
                     c = (c-renderFieldRange.getLowerBound())/renderFieldRange.getRange();
                     if (c < 0) {
                        c = 0;
                     } else if (c > 1) {
                        c = 1;
                     }
                     renderColorMap.getRGB(c, color);
                     color[3] = 1;
                     break;
                  }
                  case DIM : {
                     // set color to norm of field
                     double c = myVals.get(offset+renderDim)*myScale;
                     c = (c-renderFieldRange.getLowerBound())/renderFieldRange.getRange();
                     if (c < 0) {
                        c = 0;
                     } else if (c > 1) {
                        c = 1;
                     }
                     renderColorMap.getRGB(c, color);
                     break;
                  }
                  default:
                     break;
               }
               
               offset += myDims;
               ++vidx;
            }
         }
      }
      
      robj.notifyPositionsModified();
      robj.notifyColorsModified();
      
   }
   
   @Override
   public synchronized void prerender(RenderList list) {
      super.prerender(list);
      
      // rebuild render object
      if (myRobj == null) {
         myRobj = buildRenderObject();
      } else {
         updateRenderObject(myRobj);
      }
      myRobjValid = true;
   }

   @Override
   public synchronized void render(Renderer renderer, int flags) {
      
      if (myRobj == null) {
         myRobj = buildRenderObject();
      } else if (!myRobjValid) {
         updateRenderObject(myRobj);
      }
      myRobjValid = true;
      
      RenderProps rprops = getRenderProps();
      
      renderer.setPointColoring(rprops, isSelected());
      
      ColorMixing cmix = renderer.getVertexColorMixing();
      switch (renderStyle) {
         case DIM:
         case NORM:
            renderer.setVertexColorMixing(ColorMixing.REPLACE);
            renderer.drawPoints(myRobj, rprops.getPointStyle(), rprops.getPointSize());
            break;
         case SOLID:
            renderer.setVertexColorMixing(ColorMixing.NONE);
            renderer.drawPoints(myRobj, rprops.getPointStyle(), rprops.getPointSize());
            break;
      }
      renderer.setVertexColorMixing(cmix);
      
   }

}
