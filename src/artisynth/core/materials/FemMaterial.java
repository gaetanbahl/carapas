package artisynth.core.materials;

import artisynth.core.modelbase.PropertyChangeEvent;
import artisynth.core.modelbase.PropertyChangeListener;
import maspack.properties.PropertyList;
import maspack.properties.PropertyUtils;
import maspack.util.DynamicArray;

/**
 * FEM material base implementing {@link ConstitutiveMaterial}.  Keeps track
 * of subclasses for use in selector widgets.
 */
public abstract class FemMaterial extends MaterialBase implements ConstitutiveMaterial {

   static DynamicArray<Class<?>> mySubclasses = new DynamicArray<>(
      new Class<?>[] {
         LinearMaterial.class,
         StVenantKirchoffMaterial.class,
         MooneyRivlinMaterial.class,
         CubicHyperelastic.class,
         OgdenMaterial.class,
         FungMaterial.class,
         NeoHookeanMaterial.class,
         IncompNeoHookeanMaterial.class,
         AnisotropicLinearMaterial.class,
         NullMaterial.class,
         IncompressibleMaterial.class
      });


   /**
    * Allow adding of classes (for use in control panels)
    * @param cls
    */
   public static void registerSubclass(Class<? extends FemMaterial> cls) {
      if (!mySubclasses.contains(cls)) {
         mySubclasses.add(cls);
      }
   }

   public static Class<?>[] getSubClasses() {
      return mySubclasses.getArray();
   }

   ViscoelasticBehavior myViscoBehavior;

   // protected void notifyHostOfPropertyChange (String name) {
   //    if (myPropHost instanceof PropertyChangeListener) {
   //       ((PropertyChangeListener)myPropHost).propertyChanged (
   //          new PropertyChangeEvent (this, name));
   //    }
   // }

   protected void notifyHostOfPropertyChange () {
      notifyHostOfPropertyChange ("???");
      // if (myPropHost instanceof FemModel) {
      //    ((FemModel)myPropHost).invalidateStressAndStiffness();
      //    ((FemModel)myPropHost).invalidateRestData();
      // }
      // else if (myPropHost instanceof FemElement) {
      //    ((FemElement)myPropHost).invalidateRestData();
      // }
   }

   public static PropertyList myProps =
      new PropertyList(FemMaterial.class, MaterialBase.class);

   static {
      myProps.add (
         "viscoBehavior", "visco elastic material behavior", null);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   @Override
   public boolean isInvertible() {
      return false;
   }

   @Override
   public boolean isIncompressible() {
      return false;
   }

   @Override
   public boolean isLinear() {
      return false;
   }

   @Override
   public boolean isCorotated() {
      return false;
   }

   @Override
   public boolean isViscoelastic() {
      return myViscoBehavior != null;
   }

   @Override
   public ViscoelasticBehavior getViscoBehavior () {
      return myViscoBehavior;
   }

   /**
    * Allows setting of viscoelastic behaviour
    * @param veb
    */
   public void setViscoBehavior (ViscoelasticBehavior veb) {
      if (veb != null) {
         ViscoelasticBehavior newVeb = veb.clone();
         PropertyUtils.updateCompositeProperty (
            this, "viscoBehavior", null, newVeb);
         myViscoBehavior = newVeb;
         notifyHostOfPropertyChange ("viscoBehavior");
      }
      else if (myViscoBehavior != null) {
         PropertyUtils.updateCompositeProperty (
            this, "viscoBehavior", myViscoBehavior, null);
         myViscoBehavior = null;
         notifyHostOfPropertyChange ("viscoBehavior");
      }
   }

   @Override
   public BulkIncompressibleBehavior getIncompressibleBehavior() {
      return null;
   }

   public boolean equals (FemMaterial mat) {
      return true;
   }

   public boolean equals (Object obj) {
      if (obj instanceof FemMaterial) {
         FemMaterial mat = (FemMaterial)obj;
         if (PropertyUtils.equalValues (myViscoBehavior, mat.myViscoBehavior)) {
            return equals (mat);
         }
      }
      return false;
   }

   public FemMaterial clone() {
      FemMaterial mat = (FemMaterial)super.clone();
      mat.setViscoBehavior (myViscoBehavior);
      return mat;
   }

}


