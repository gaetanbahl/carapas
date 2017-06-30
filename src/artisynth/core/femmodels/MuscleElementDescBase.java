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
import java.util.List;

import artisynth.core.femmodels.MuscleBundle.DirectionRenderType;
import artisynth.core.materials.BulkIncompressibleBehavior;
import artisynth.core.materials.ConstitutiveMaterial;
import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.MaterialBase;
import artisynth.core.materials.MuscleMaterial;
import artisynth.core.materials.SolidDeformation;
import artisynth.core.materials.ViscoelasticBehavior;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.ExcitationSource;
import artisynth.core.mechmodels.ExcitationSourceList;
import artisynth.core.mechmodels.ExcitationUtils;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.DynamicActivityChangeEvent;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableComponentBase;
import artisynth.core.modelbase.TransformGeometryContext;
import artisynth.core.modelbase.TransformableGeometry;
import artisynth.core.util.ScalableUnits;
import artisynth.core.util.ScanToken;
import maspack.matrix.AffineTransform3dBase;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix6d;
import maspack.matrix.SymmetricMatrix3d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.Shading;
import maspack.render.color.ColorUtils;
import maspack.util.NumberFormat;
import maspack.util.ReaderTokenizer;

/**
 * A class wrapping the description of each FEM element belonging to a
 * MuscleBundle. It implements the AuxiliaryMaterial required to effect muscle
 * activation within the element.
*/
public abstract class MuscleElementDescBase
   extends RenderableComponentBase
   implements ConstitutiveMaterial, AuxiliaryMaterial,
   ExcitationComponent, ScalableUnits, TransformableGeometry {

   FemElement3d myElement;
   private MuscleMaterial myMuscleMat;

   private double myExcitation = 0;
   protected ExcitationSourceList myExcitationSources;
   protected CombinationRule myComboRule = CombinationRule.Sum;

   // the following are set if an activation color is specified:
   protected float[] myDirectionColor; // render color for directions
   protected float[] myWidgetColor; // render color for elements
 
   // minimum activation level
   protected static final double minActivation = 0.0;
   // maximum activation level
   protected static final double maxActivation = 1.0;

   public MuscleElementDescBase () {
      super();
   }

   public MuscleElementDescBase (FemElement3d elem) {
      this();
      setElement (elem);
   }

   public static PropertyList myProps =
      new PropertyList (MuscleElementDesc.class, RenderableComponentBase.class);

   static {
      myProps.add ("renderProps", "render properties", null);
      myProps.addReadOnly (
         "netExcitation", "total excitation including excitation sources");
      myProps.add ("excitation", "internal muscle excitation", 0.0, "[0,1] NW");
      myProps.add (
         "muscleMaterial", "muscle material parameters", null);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
    
   public MuscleMaterial getMuscleMaterial() {
      return myMuscleMat;
   }

   public void setMuscleMaterial (MuscleMaterial mat) {
      myMuscleMat = (MuscleMaterial)MaterialBase.updateMaterial (
         this, "muscleMaterial", myMuscleMat, mat);
      // issue DynamicActivityChange in case solve matrix symmetry has changed:
      notifyParentOfChange (DynamicActivityChangeEvent.defaultEvent);            
   }

   public boolean isInvertible() {
      MuscleMaterial mat = getEffectiveMuscleMaterial();
      return mat == null || mat.isInvertible();
   }

   @Override
   public boolean isLinear() {
      //      MuscleMaterial mat = getEffectiveMuscleMaterial();
      //      return mat == null || mat.isLinear();
      return false;
   }
   
   @Override
   public boolean isCorotated() {
      //      MuscleMaterial mat = getEffectiveMuscleMaterial();
      //      return mat == null || mat.isCorotated();
      return false;
   }

   /**
    * {@inheritDoc}
    */
   public double getExcitation() {
      return myExcitation;
   }

   /**
    * {@inheritDoc}
    */
   public void initialize (double t) {
      if (t == 0) {
         setExcitation (0);
      }
   }

   /**
    * {@inheritDoc}
    */
   public void setExcitation (double a) {
      // set activation within valid range
      double valid_a = a;
      valid_a = (valid_a > maxActivation) ? maxActivation : valid_a;
      valid_a = (valid_a < minActivation) ? minActivation : valid_a;
      myExcitation = valid_a;
   }

   /**
    * {@inheritDoc}
    */
   public void setCombinationRule (CombinationRule rule) {
      myComboRule = rule;
   }

   /**
    * {@inheritDoc}
    */
   public CombinationRule getCombinationRule() {
      return myComboRule;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void addExcitationSource (ExcitationComponent ex) {
      addExcitationSource (ex, 1);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void addExcitationSource (ExcitationComponent ex, double gain) {
      if (myExcitationSources == null) {
         myExcitationSources = new ExcitationSourceList();
      }
      myExcitationSources.add (ex, gain);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public boolean removeExcitationSource (ExcitationComponent ex) {
      boolean removed = false;
      if (myExcitationSources != null) {
         removed = myExcitationSources.remove (ex);
         if (myExcitationSources.size() == 0) {
            myExcitationSources = null;
         }
      }
      return removed;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double getExcitationGain (ExcitationComponent ex) {
      return ExcitationUtils.getGain (myExcitationSources, ex);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public boolean setExcitationGain (ExcitationComponent ex, double gain) {
      return ExcitationUtils.setGain (myExcitationSources, ex, gain);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public double getNetExcitation() {
      double net = ExcitationUtils.combineWithAncestor (
         this, myExcitationSources, /*up to grandparent=*/2, myComboRule);
      return net;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void getSoftReferences (List<ModelComponent> refs) {
      super.getSoftReferences (refs);
      if (myExcitationSources != null) {
         myExcitationSources.getSoftReferences (refs);
      }
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public void updateReferences (boolean undo, Deque<Object> undoInfo) {
      super.updateReferences (undo, undoInfo);
      myExcitationSources = ExcitationUtils.updateReferences (
         this, myExcitationSources, undo, undoInfo);
   }

   /**
    * {@inheritDoc}
    */
   public double getDefaultActivationWeight() {
      return 0;
   }
   
   public void updateBounds(Vector3d pmin, Vector3d pmax) {
      super.updateBounds(pmin, pmax);
      if (myElement != null)
	 myElement.updateBounds(pmin, pmax);
   }
   
   void setExcitationColors (RenderProps props) {
      ModelComponent gparent = getGrandParent();
      if (gparent instanceof MuscleBundle) {
         MuscleBundle bundle = (MuscleBundle)gparent;
         float[] excitationColor = bundle.myExcitationColor;
         if (excitationColor != null) {
            double s =
               Math.min(getNetExcitation()/bundle.myMaxColoredExcitation, 1);
            float[] baseColor;
            if (myDirectionColor == null) {
               myDirectionColor = new float[4];
            }
            myDirectionColor[3] = (float)props.getAlpha ();
            baseColor = props.getLineColorF();
            ColorUtils.interpolateColor (
               myDirectionColor, baseColor, excitationColor, s);
            if (myWidgetColor == null) {
               myWidgetColor = new float[4];
            }
            baseColor = props.getFaceColorF();
            ColorUtils.interpolateColor (
               myWidgetColor, baseColor, excitationColor, s);
            myWidgetColor[3] = (float)props.getAlpha ();
            
         }
         else {
            myDirectionColor = null;
            myWidgetColor = null;
         }
      }
   }

   public void prerender (RenderList list) {
      // This is to ensure that the invJ0 in the warping data is updated in the
      // current (simulation) thread.
      myElement.getWarpingData();
      setExcitationColors (myRenderProps);
   }
   
   public abstract Vector3d getMuscleDirection(IntegrationPoint3d pnt);
   
   public abstract Vector3d getMuscleDirection(int ipntIdx);

   protected void renderINodeDirection(Renderer renderer, RenderProps props,
      float[] coords0, float[] coords1, Matrix3d F, Vector3d dir, double len) {
      
      IntegrationPoint3d[] ipnt = myElement.getIntegrationPoints();
      IntegrationData3d[] idata = myElement.getIntegrationData();   
      
      for (int i=0; i<ipnt.length; i++) {
      
         Vector3d mdir = getMuscleDirection(ipnt[i]);
         boolean drawLine = false;
         if (mdir != null) {
            drawLine = true;
            dir.set(mdir);
         }
         
         if (drawLine) {
            ipnt[i].computeGradientForRender(F, myElement.getNodes(), idata[i].myInvJ0);
            ipnt[i].computeCoordsForRender(coords0, myElement.getNodes());
            F.mul(dir,dir);
            
            double size = myElement.computeDirectedRenderSize (dir);
            dir.scale(0.5*size);
            dir.scale(len);
            
            coords0[0] -= (float)dir.x / 2;
            coords0[1] -= (float)dir.y / 2;
            coords0[2] -= (float)dir.z / 2;
            coords1[0] = coords0[0] + (float)dir.x;
            coords1[1] = coords0[1] + (float)dir.y;
            coords1[2] = coords0[2] + (float)dir.z;
            
            renderer.drawLine(
               props, coords0, coords1, myDirectionColor,
               /*capped=*/true, /*highlight=*/false);   
         }
      }
      
   }
   
   protected void renderElementDirection(Renderer renderer, RenderProps props,
      float[] coords0, float[] coords1, Matrix3d F, Vector3d dir, double len) {
      
      myElement.computeRenderCoordsAndGradient (F, coords0);

      dir.setZero();
      int count = 0;
      for (IntegrationPoint3d pt : myElement.getIntegrationPoints()) {
         Vector3d mdir = getMuscleDirection(pt);
         if (mdir != null) {
            dir.add(mdir);
            ++count;
         }
      }
      
      if (count > 0) {
         dir.normalize();
         F.mul (dir, dir);
         
         double size = myElement.computeDirectedRenderSize (dir);      
         dir.scale (0.5*size);
         dir.scale(len);
               
         coords0[0] -= (float)dir.x/2;
         coords0[1] -= (float)dir.y/2;
         coords0[2] -= (float)dir.z/2;
               
         coords1[0] = coords0[0] + (float)dir.x;
         coords1[1] = coords0[1] + (float)dir.y;
         coords1[2] = coords0[2] + (float)dir.z;
               
         renderer.drawLine (
            props, coords0, coords1, myDirectionColor, 
            /*capped=*/true, isSelected());
      }
      
   }
   
   void renderDirection (
      Renderer renderer, RenderProps props,
      float[] coords0, float[] coords1, Matrix3d F, Vector3d dir, double len, DirectionRenderType type) {

      
      switch(type) {
         case ELEMENT:
            renderElementDirection(renderer, props, coords0, coords1, F, dir, len);
            break;
         case INTEGRATION_POINT:
            renderINodeDirection(renderer, props, coords0, coords1, F, dir, len);
            break;
      }
      
   }
      
   public void render (Renderer renderer, int flags) {
      render (renderer, myRenderProps, flags);
   }   

   public void render (Renderer renderer, RenderProps props, int flags) {
      double widgetSize = 0;
      double directionLength = 0;
      ModelComponent gparent = getGrandParent();
      DirectionRenderType renderType = DirectionRenderType.ELEMENT;
      
      if (gparent instanceof MuscleBundle) {
         MuscleBundle bundle = (MuscleBundle)gparent;
         widgetSize = bundle.getElementWidgetSize();
         directionLength = bundle.getDirectionRenderLen();
         renderType = bundle.getDirectionRenderType();
      }      
      if (widgetSize != 0) {
         Shading savedShading = renderer.setPropsShading (props);
         renderer.setFaceColoring (props, myWidgetColor, isSelected());
         myElement.renderWidget (renderer, widgetSize, props);
         renderer.setShading (savedShading);
      }
      if (directionLength > 0) {
         Matrix3d F = new Matrix3d();
         Vector3d dir = new Vector3d();
         float[] coords0 = new float[3];
         float[] coords1 = new float[3]; 

         renderDirection (
            renderer, props, coords0, coords1, F, dir,
            directionLength, renderType);
      }
   }

   private MuscleMaterial getEffectiveMuscleMaterial () {
      if (myMuscleMat != null) {
         return myMuscleMat;
      }
      else {
         ModelComponent gparent = getGrandParent();
         if (gparent instanceof MuscleBundle) {
            return ((MuscleBundle)gparent).getEffectiveMuscleMaterial();
         }
      }
      return null;      
   }

//   public void addTangent (
//      Matrix6d D, SymmetricMatrix3d stress, IntegrationPoint3d pt, 
//      IntegrationData3d dt, FemMaterial baseMat) {
//      
//      MuscleMaterial mat = getEffectiveMuscleMaterial();
//      if (mat != null) {
//         Vector3d dir = null;
//         if (myDirs != null) {
//            dir = myDirs[pt.getNumber()];
//         }
//         else {
//            dir = myDir;
//         }
//         if (dir != null) {
//            mat.addTangent (D, stress, getNetExcitation(), dir, pt, baseMat);
//         }
//      }
//   }
   
   public void computeTangent (
      Matrix6d D, SymmetricMatrix3d stress,
      SolidDeformation def, Matrix3d Q, FemMaterial baseMat) {

      MuscleMaterial mat = getEffectiveMuscleMaterial();
      if (mat != null) {
         int iidx = def.getMaterialCoordinate().getCoordinateIndex();
         Vector3d dir = getMuscleDirection(iidx);
         if (dir != null) {
            mat.computeTangent (D, stress, getNetExcitation(), dir, def, baseMat);
         }
      }
   }
  
//   public void addStress (
//      SymmetricMatrix3d sigma, IntegrationPoint3d pt, 
//      IntegrationData3d dt, FemMaterial baseMat) {
//      
//      MuscleMaterial mat = getEffectiveMuscleMaterial();
//      if (mat != null) {
//         Vector3d dir = null;
//         if (myDirs != null) {
//            dir = myDirs[pt.getNumber()];
//         }
//         else {
//            dir = myDir;
//         }
//         if (dir != null) {
//            mat.addStress (sigma, getNetExcitation(), dir, pt, baseMat);
//         }
//      }
//   }
// 

 
   // @Override
   // public void computeStress (
   //    SymmetricMatrix3d sigma, SolidDeformation def,
   //    Matrix3d Q, FemMaterial baseMat) {
      
   //    sigma.setZero();
   //    MuscleMaterial mat = getEffectiveMuscleMaterial();
   //    if (mat != null) {
   //       Vector3d dir = null;
   //       if (myDirs != null) {
   //          dir = myDirs[pt.getNumber()];
   //       }
   //       else {
   //          dir = myDir;
   //       }
   //       if (dir != null) {
   //          mat.addStress (sigma, getNetExcitation(), dir, pt, baseMat);
   //       }
   //    }
   // }
   
   public void computeStress (
      SymmetricMatrix3d sigma, SolidDeformation def,
      Matrix3d Q, FemMaterial baseMat) {
      
      MuscleMaterial mat = getEffectiveMuscleMaterial();
      
      if (mat != null) {
         Vector3d dir = getMuscleDirection(def.getMaterialCoordinate().getCoordinateIndex());
         if (dir != null) {
            mat.computeStress (sigma, getNetExcitation(), dir, def, baseMat);
         }
      }
   }
   
//   private SymmetricMatrix3d tmpStress = new SymmetricMatrix3d();
//   @Override
//   public void addStressAndTangent(SymmetricMatrix3d sigma, Matrix6d D,
//      IntegrationPoint3d pt, IntegrationData3d dt, FemMaterial baseMat) {
//      computeStress(tmpStress, pt, dt, baseMat);
//      sigma.add(tmpStress);
//      addTangent(D, tmpStress, pt, dt, baseMat);
//   }
   
//   @Override
//   public void computeStressAndTangent(SymmetricMatrix3d sigma, Matrix6d D,
//      IntegrationPoint3d pt, IntegrationData3d dt, FemMaterial baseMat) {
//      computeStress(sigma, pt, dt, baseMat);
//      computeTangent(D, sigma, pt, dt, baseMat);
//   }
   
   @Override
   public void computeStress(
      SymmetricMatrix3d sigma, SolidDeformation def, IntegrationPoint3d pt,
      IntegrationData3d dt, FemMaterial baseMat) {
      computeStress(sigma, def, dt.myFrame, baseMat);
      
   }

   @Override
   public void computeTangent(
      Matrix6d D, SymmetricMatrix3d stress, SolidDeformation def,
      IntegrationPoint3d pt, IntegrationData3d dt, FemMaterial baseMat) {
      computeTangent(D, stress, def, dt.myFrame, baseMat);
   }

   public boolean hasSymmetricTangent() {
      MuscleMaterial mat = getEffectiveMuscleMaterial();
      if (mat != null) {
         return mat.hasSymmetricTangent();
      }
      else {
         return true;
      }
   }

   // public void getDependencies (
   //    List<ModelComponent> deps, ModelComponent ancestor) {
   //    super.getDependencies (deps, ancestor);
   //    // TODO: see why these are NOT included in Muscle code:
   //    if (myExcitationSources != null) {
   //       for (ExcitationComponent ex : myExcitationSources) {
   //          if (ex != getGrandParent()) {
   //             ComponentUtils.addDependencies (deps, ex, ancestor);
   //          }
   //       }
   //    }
   // }

   public FemElement3d getElement() {
      return myElement;
   }

   public void setElement (FemElement3d elem) {
      myElement = elem;
   }
   
   public void transformGeometry(AffineTransform3dBase X) {
      TransformGeometryContext.transform (this, X, 0);
   }  
   
   public void addTransformableDependencies (
      TransformGeometryContext context, int flags) {
      // no dependencies
   }

   public void scaleDistance (double s) {
      if (myMuscleMat != null) {
         myMuscleMat.scaleDistance (s);
      }
      if (myRenderProps != null) {
         myRenderProps.scaleDistance (s);
      }
   }

   public void scaleMass (double s) {
      if (myMuscleMat != null) {
         myMuscleMat.scaleMass (s);
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
      //ExcitationUtils.addAncestorAsSource (this, /*up to grandparent*/2);
   }

   @Override
      public void disconnectFromHierarchy() {
      //ExcitationUtils.removeAncestorAsSource (this, /*up to grandparent*/2);
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
      else if (scanAttributeName (rtok, "excitationSources")) {
         myExcitationSources =
            ExcitationUtils.scan (rtok, "excitationSources", tokens);
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
      else if (postscanAttributeName (tokens, "excitationSources")) {
         myExcitationSources.postscan (tokens, ancestor);
         return true;
      }  
      return super.postscanItem (tokens, ancestor);
   }

   public void writeItems (
      PrintWriter pw, NumberFormat fmt, CompositeComponent ancestor)
      throws IOException {

      super.writeItems (pw, fmt, ancestor);
      printElementReference (pw, ancestor);
      pw.println ("");
      if (myExcitationSources != null) {
         myExcitationSources.write (pw, "excitationSources", fmt, ancestor);
      }      
   }

   @Override
   public boolean isIncompressible() {
      return false;
   }

   @Override
   public BulkIncompressibleBehavior getIncompressibleBehavior() {
      return null;
   }
   
   @Override
   public boolean isViscoelastic() {
      return false;
   }

   @Override
   public ViscoelasticBehavior getViscoBehavior() {
      return null;
   }

   @Override
   public MuscleElementDesc clone() {
      
      MuscleElementDesc other;
      try {
         other = (MuscleElementDesc)(super.clone());
      } catch (CloneNotSupportedException e) {
         throw new RuntimeException("Cannot clone super");
      }
      
      if (myExcitationSources != null) {
         other.myExcitationSources = new ExcitationSourceList();
         for (ExcitationSource ex : myExcitationSources) {
            other.addExcitationSource(ex.getComponent(), ex.getGain());
         }
      }

      // the following are set if an activation color is specified:
      if (myDirectionColor != null) {
         other.myDirectionColor = Arrays.copyOf(myDirectionColor, myDirectionColor.length);
      }
      if (myWidgetColor != null) {
         other.myWidgetColor = Arrays.copyOf(myWidgetColor, myWidgetColor.length);
      }
      
      return other;
   }

}
