/**
 * Copyright (c) 2014, by the Authors: Antonio Sanchez (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.core.mfreemodels;

import java.io.IOException;
import java.io.PrintWriter;
import java.util.Arrays;
import java.util.Deque;

import artisynth.core.femmodels.AuxiliaryMaterial;
import artisynth.core.femmodels.IntegrationData3d;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.MaterialBase;
import artisynth.core.materials.SolidDeformation;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.DynamicActivityChangeEvent;
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
public class MFreeAuxMaterialElementDesc
extends RenderableComponentBase
implements AuxiliaryMaterial, ScalableUnits, TransformableGeometry {

   MFreeElement3d myElement;
   private FemMaterial myMat;

   // fraction to scale material's contribution
   private double myFrac = 1;
   private double[] myFracs;     

   protected float[] myWidgetColor; // render color for elements

   public MFreeAuxMaterialElementDesc(MFreeElement3d elem) {
      super();
      setElement(elem);
      setFraction(1);
      setMaterial(null);
   }

   public MFreeAuxMaterialElementDesc(MFreeElement3d elem, FemMaterial mat) {
      super();

      setElement(elem);
      setFraction(1);
      setMaterial(mat);
   }

   public MFreeAuxMaterialElementDesc (MFreeElement3d elem, FemMaterial mat, double frac) {
      super();
      setElement (elem);
      setFraction(frac);
   }

   public static PropertyList myProps =
      new PropertyList (MFreeAuxMaterialElementDesc.class, RenderableComponentBase.class);

   static {
      myProps.add ("renderProps", "render properties", null);
      myProps.add ("fraction", "material fraction", 1);
      myProps.add (
         "material", "muscle material parameters", null);
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
      if (grandParent instanceof MFreeAuxMaterialBundle) {
         return ((MFreeAuxMaterialBundle)grandParent).getMaterial();
      }
      return null;
   }

   public void updateBounds(Vector3d pmin, Vector3d pmax) {
      super.updateBounds(pmin, pmax);
      if (myElement != null)
         myElement.updateBounds(pmin, pmax);
   }
   
   @Override
   public void computeStress (
      SymmetricMatrix3d sigma, SolidDeformation def,
      IntegrationPoint3d pt, IntegrationData3d dt, FemMaterial baseMat) {
      
      FemMaterial mat = getEffectiveMaterial();
      if (mat != null) {
         double frac = myFrac;
         if (myFracs != null) {
            int idx = myElement.getIntegrationPointIndex(pt);
            frac = myFracs[idx];
         }

//         if (mat instanceof LinearMaterial) {
//            LinearMaterial lmat = (LinearMaterial)mat;
//            Matrix3d R = null;
//            if (lmat.isCorotated()) {
//               R = myElement.myWarper.R;
//            }
//            addLinearStress(frac, sigma, pt, R);
//         } else {
         if (frac > 0) {
            Matrix3d Q = dt.getFrame();
            if (Q == null) {
               Q = Matrix3d.IDENTITY;
            }
            mat.computeStress(sigma, def, Q, baseMat);
            sigma.scale(frac);
         } else {
            sigma.setZero();
         }
//         }
      }
      
   }

   @Override
   public void computeTangent(Matrix6d D, SymmetricMatrix3d stress,
      SolidDeformation def, IntegrationPoint3d pt, IntegrationData3d dt, FemMaterial baseMat) {
      
      FemMaterial mat = getEffectiveMaterial();
      if (mat != null) {
         double frac = myFrac;
         if (myFracs != null) {
            int idx = myElement.getIntegrationPointIndex(pt);
            frac = myFracs[idx];
         }

         if (frac > 0) {
            Matrix3d Q = dt.getFrame();
            if (Q == null) {
               Q = Matrix3d.IDENTITY;
            }           
            mat.computeTangent(D, stress, def, Q, baseMat);
            D.scale(frac);
         } else {
            D.setZero();
         }
      } else {
         D.setZero();
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

   public MFreeElement3d getElement() {
      return myElement;
   }

   public void setElement (MFreeElement3d elem) {
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
      if (MFreeMuscleBundle.getAncestorModel (this) != null) {
         referenceElement();
      }
   }

   @Override
   public void disconnectFromHierarchy() {
      if (MFreeMuscleBundle.getAncestorModel (this) != null) {
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
         setElement (postscanReference (
            tokens, MFreeElement3d.class, ancestor));
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
      // this is just stub code for now
      if (false) {
         Shading savedShading = renderer.setPropsShading (myRenderProps);
         renderer.setFaceColoring (
            myRenderProps, myWidgetColor, isSelected());
         myElement.renderWidget (renderer, myRenderProps, 0);
         renderer.setShading (savedShading);
      }      
   }

   @Override
   public MFreeAuxMaterialElementDesc clone() {
      MFreeAuxMaterialElementDesc copy;
      try {
         copy = (MFreeAuxMaterialElementDesc)super.clone();
      } catch (CloneNotSupportedException e) {
         throw new RuntimeException(e);
      }
      
      if (myFracs != null) {
         copy.myFracs = Arrays.copyOf(myFracs, myFracs.length);
      }
      copy.myWidgetColor = Arrays.copyOf(myWidgetColor, myWidgetColor.length);
      
      return copy;
   }

}
