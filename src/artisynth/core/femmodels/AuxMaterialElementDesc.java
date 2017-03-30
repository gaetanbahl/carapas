/**
 * Copyright (c) 2014, by the Authors: John E Lloyd (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.core.femmodels;

import java.io.IOException;
import java.io.PrintWriter;
import java.util.Arrays;
import java.util.Deque;

import artisynth.core.femmodels.AuxMaterialBundle.FractionRenderType;
import artisynth.core.materials.BulkIncompressibleBehavior;
import artisynth.core.materials.ConstitutiveMaterial;
import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.MaterialBase;
import artisynth.core.materials.MaterialCoordinate;
import artisynth.core.materials.SolidDeformation;
import artisynth.core.materials.ViscoelasticBehavior;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.DynamicActivityChangeEvent;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableComponentBase;
import artisynth.core.modelbase.TransformGeometryContext;
import artisynth.core.modelbase.TransformableGeometry;
import artisynth.core.util.ScalableUnits;
import artisynth.core.util.ScanToken;
import maspack.geometry.GeometryTransformer;
import maspack.matrix.AffineTransform3dBase;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix6d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.Shading;
import maspack.util.IndentingPrintWriter;
import maspack.util.NumberFormat;
import maspack.util.ReaderTokenizer;

/**
 * A class wrapping the description of each FEM element. It implements the 
 * AuxiliaryMaterial required to mix material types together within a single
 * element.
 */
public class AuxMaterialElementDesc extends RenderableComponentBase
   implements ConstitutiveMaterial, ScalableUnits, TransformableGeometry {

   FemElement3d myElement;
   private FemMaterial myMat;
   
   private float[] rcoords = new float[3];
   
   // fraction to scale material's contribution
   private double myFrac = 1;
   private double[] myFracs;     

   protected float[] myWidgetColor; // render color for elements

   public AuxMaterialElementDesc() {
      super();
   }
   
   public AuxMaterialElementDesc(FemElement3d elem) {
      this();
      setElement(elem);
      setFraction(1);
      setMaterial(null);
   }

   public AuxMaterialElementDesc(FemElement3d elem, FemMaterial mat) {
      this();
      setElement(elem);
      setFraction(1);
      setMaterial(mat);
   }

   public AuxMaterialElementDesc (FemElement3d elem, FemMaterial mat, double frac) {
      this();
      setElement (elem);
      setFraction(frac);
   }

   public static PropertyList myProps =
      new PropertyList (AuxMaterialElementDesc.class, RenderableComponentBase.class);

   static {
      myProps.add ("renderProps", "render properties", null);
      myProps.add ("fraction", "material fraction", 1);
      myProps.add (
         "material", "fem material parameters", null);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public void setFraction (double frac) {
      myFrac = frac;
   }

   public void setFractions(double [] fracs) {
      if (fracs == null) {
         myFracs = null;
         return;
      }
      myFracs = new double[fracs.length];
      for (int i=0; i<fracs.length; i++) {
         myFracs[i] = fracs[i];
      }

   }

   public double getFraction() {
      return myFrac;
   }

   public double[] getFractions() {
      return myFracs;
   }

   public FemMaterial getMaterial() {
      return myMat;
   }

   public void setMaterial (FemMaterial mat) {
      myMat = (FemMaterial)MaterialBase.updateMaterial (
         this, "material", myMat, mat);
      //      myMat = mat;

      // issue DynamicActivityChange in case solve matrix symmetry has changed:
      notifyParentOfChange (DynamicActivityChangeEvent.defaultEvent);      

   }

   @Override
   public boolean isInvertible() {
      FemMaterial mat = getEffectiveMaterial();
      return mat == null || mat.isInvertible();
   }
   
   @Override
   public boolean isLinear() {
      FemMaterial mat = getEffectiveMaterial();
      return mat == null || mat.isLinear();
   }
   
   @Override
   public boolean isCorotated() {
      FemMaterial mat = getEffectiveMaterial();
      return mat == null || mat.isCorotated();
   }

   protected FemMaterial getEffectiveMaterial() {
      if (myMat != null) {
         return myMat;
      }
      CompositeComponent grandParent = getGrandParent();
      if (grandParent instanceof AuxMaterialBundle) {
         return ((AuxMaterialBundle)grandParent).getMaterial();
      }
      return null;
   }

   public void updateBounds(Vector3d pmin, Vector3d pmax) {
      super.updateBounds(pmin, pmax);
      if (myElement != null)
         myElement.updateBounds(pmin, pmax);
   }

   @Override
   public void computeStress(
      SymmetricMatrix3d sigma, SolidDeformation def, Matrix3d Q,
      FemMaterial baseMat) {
      
      FemMaterial mat = getEffectiveMaterial();
      MaterialCoordinate coord = def.getMaterialCoordinate();
      // int eidx = coord.getSubvolumeIndex();
      int iidx = coord.getCoordinateIndex();
      
      if (mat != null) {
         double frac = myFrac;
         if (myFracs != null) {
            frac = myFracs[iidx];
         }
         if (frac > 0) {
            mat.computeStress(sigma, def, Q, baseMat);
            sigma.scale(frac);
         } else {
            sigma.setZero();
         }
      }
      
   }

   @Override
   public void computeTangent(
      Matrix6d D, SymmetricMatrix3d stress, SolidDeformation def, Matrix3d Q,
      FemMaterial baseMat) {
      
      FemMaterial mat = getEffectiveMaterial();
      int iidx = def.getMaterialCoordinate().getCoordinateIndex();
      
      if (mat != null) {
         double frac = myFrac;
         if (myFracs != null) {
            frac = myFracs[iidx];
         }
         if (frac > 0) {
            mat.computeTangent (D, stress, def, Q, baseMat);
            D.scale(frac);
         } else {
            D.setZero();
         }
      }
   }

   public boolean hasSymmetricTangent() {
      FemMaterial mat = getEffectiveMaterial();
      if (mat != null) {
         return mat.hasSymmetricTangent();
      }
      else {
         return true;
      }
   }

   public FemElement3d getElement() {
      return myElement;
   }

   public void setElement (FemElement3d elem) {
      myElement = elem;
   }

   public void transformGeometry(AffineTransform3dBase X) {
      TransformGeometryContext.transform (this, X, 0);      
   }

   public void transformGeometry (
      GeometryTransformer gtr, TransformGeometryContext context, int flags) {
      // nothing to at the top level
   }

   public void addTransformableDependencies (
      TransformGeometryContext context, int flags) {
      // no dependencies
   }

   public void scaleDistance (double s) {
      if (myMat != null) {
         myMat.scaleDistance (s);
      }
      if (myRenderProps != null) {
         myRenderProps.scaleDistance (s);
      }
   }

   public void scaleMass (double s) {
      if (myMat != null) {
         myMat.scaleMass (s);
      }
   }

   void referenceElement() {
      myElement.addAuxiliaryMaterial (this);
      //myElement.addBackReference (this);
   }

   void dereferenceElement() {
      //myElement.removeBackReference (this);
      myElement.removeAuxiliaryMaterial (this);
   }

   @Override
   public void connectToHierarchy () {
      super.connectToHierarchy ();
      if (MuscleBundle.getAncestorFem (this) != null) {
         referenceElement();
      }
   }

   @Override
   public void disconnectFromHierarchy() {
      if (MuscleBundle.getAncestorFem (this) != null) {
         dereferenceElement();
      }
      super.disconnectFromHierarchy();
   }

   public void printElementReference (PrintWriter pw, CompositeComponent ancestor)
      throws IOException {
      pw.print ("element=" +
         ComponentUtils.getWritePathName (ancestor, myElement));
   }

   @Override
   public boolean scanItem (ReaderTokenizer rtok, Deque<ScanToken> tokens)
      throws IOException {

      rtok.nextToken();
      if (scanAndStoreReference (rtok, "element", tokens)) {
         return true;
      }
      rtok.pushBack();
      return super.scanItem (rtok, tokens);
   }
   
   protected boolean postscanItem (
      Deque<ScanToken> tokens, CompositeComponent ancestor) throws IOException {

      if (postscanAttributeName (tokens, "element")) {
         setElement (postscanReference (tokens, FemElement3d.class, ancestor));
         return true;
      }
      return super.postscanItem (tokens, ancestor);
   }   

   public void write (PrintWriter pw, NumberFormat fmt, Object ref)
      throws IOException {
      CompositeComponent ancestor =
         ComponentUtils.castRefToAncestor (ref);
      pw.print ("[ ");
      IndentingPrintWriter.addIndentation (pw, 2);
      printElementReference (pw, ancestor);
      pw.println ("");
      // pw.print (" " + fmt.format (myStiffness) +
      // " " + fmt.format (myDamping) +
      // " " + fmt.format (myRestLength));
      getAllPropertyInfo().writeNonDefaultProps (this, pw, fmt);
      IndentingPrintWriter.addIndentation (pw, -2);
      pw.println ("]");
   }


   @Override
   public void render(Renderer renderer, int flags) {
      render (renderer, myRenderProps, flags);
   }

   public void render(Renderer renderer, RenderProps props, int flags) {
      
      double widgetSize = 0;
      double rad = 0;
      FractionRenderType renderType = AuxMaterialBundle.DEFAULT_FRACTION_RENDER_TYPE;
      ModelComponent gparent = getGrandParent();
      
      
      if (gparent instanceof AuxMaterialBundle) {
         AuxMaterialBundle bundle = (AuxMaterialBundle)gparent;
         widgetSize = bundle.getElementWidgetSize();
         rad = bundle.getFractionRenderRadius();
         renderType = bundle.getFractionRenderType();
      }  
      
      if (widgetSize != 0 || rad > 0) {
         Shading savedShading = renderer.setPropsShading (props);
         renderer.setFaceColoring (props, myWidgetColor, isSelected());
         if (widgetSize != 0) {
            myElement.renderWidget (renderer, widgetSize, props);
         }
         if (rad > 0) {
            renderFraction (
               renderer, props, rad, renderType);
         }
         renderer.setShading (savedShading);
      }
   }
   
   public void renderFraction( Renderer renderer, RenderProps props, double rad, FractionRenderType renderType ) {
      switch(renderType) {
         case ELEMENT:
         case ELEMENT_SCALED:
            renderElementFraction(renderer, props, rad, renderType == FractionRenderType.ELEMENT_SCALED);
            break;
         case INTEGRATION_POINT:
         case INTEGRATION_POINT_SCALED:
            renderINodeFraction(renderer, props, rad, renderType == FractionRenderType.INTEGRATION_POINT_SCALED);
            break;
      }
   }
   
   Matrix3d RF = new Matrix3d();
   private void renderElementFraction(  Renderer renderer, RenderProps props, double rad, boolean scaled) {
      
      
      myElement.computeRenderCoordsAndGradient(RF, rcoords);
      
      double r = rad;
      
      if (scaled) {
         if (myFracs != null) {
            r = 0;
            for (double f : myFracs) {
               r += f;
            }
            r = r/myFracs.length;
            r = r*rad;
         } else {
            r = r*myFrac;
         }
      }
      
      renderer.drawSphere(rcoords, r);
   }
   
   private void renderINodeFraction(  Renderer renderer, RenderProps props, double rad, boolean scaled) {
      
      IntegrationPoint3d[] ipnt = myElement.getIntegrationPoints();
      
      for (int i=0; i<ipnt.length; i++) {
      
         boolean drawSphere = true;
         double r = rad;
         if (scaled) {
            if (myFracs != null) {
               r = r*myFracs[i];
            } else {
               r = r*myFrac;
            }
         }
         
         if (drawSphere) {
            ipnt[i].computeCoordsForRender(rcoords, myElement.getNodes());
            renderer.drawSphere(rcoords, r);  
         }
         
         
      }
      
   }

   @Override
   public boolean isIncompressible() {
      FemMaterial mat = getEffectiveMaterial();
      if (mat == null) {
         return false; 
      }
      return mat.isIncompressible();
   }
   
   @Override
   public BulkIncompressibleBehavior getIncompressibleBehavior() {
      FemMaterial mat = getEffectiveMaterial();
      if (mat == null) {
         return null; 
      }
      return mat.getIncompressibleBehavior();
   }

   @Override
   public boolean isViscoelastic() {
      FemMaterial mat = getEffectiveMaterial();
      if (mat == null) {
         return false; 
      }
      return mat.isViscoelastic();
   }

   @Override
   public ViscoelasticBehavior getViscoBehavior() {
      FemMaterial mat = getEffectiveMaterial();
      if (mat == null) {
         return null; 
      }
      return mat.getViscoBehavior();
   }

   @Override
   public AuxMaterialElementDesc clone() {
      AuxMaterialElementDesc other;
      try {
         other = (AuxMaterialElementDesc)super.clone();
      } catch (CloneNotSupportedException e) {
         throw new RuntimeException(e);
      }

      other.rcoords = Arrays.copyOf(rcoords, rcoords.length);
      
      // fraction to scale material's contribution
      if (myFracs != null) {
         other.myFracs = Arrays.copyOf(myFracs, myFracs.length);
      }
      if (myWidgetColor != null) {
         other.myWidgetColor = Arrays.copyOf(myWidgetColor, myWidgetColor.length);
      }
      return other;
   }
   
}
