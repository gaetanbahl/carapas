package artisynth.core.gui.editorManager;

import java.awt.Color;
import java.awt.Cursor;
import java.awt.event.ActionEvent;
import java.util.HashMap;
import java.util.LinkedList;

import javax.swing.JCheckBox;
import javax.swing.JComboBox;

import maspack.geometry.BVFeatureQuery;
import maspack.matrix.Point3d;
import maspack.render.MouseRayEvent;
import maspack.render.RenderProps;
import maspack.render.Renderer.Shading;
import maspack.render.GL.GLViewer;
import maspack.util.InternalErrorException;
import artisynth.core.driver.Main;
import artisynth.core.gui.editorManager.AddComponentAgent;
import artisynth.core.gui.editorManager.AddComponentsCommand;
import artisynth.core.gui.editorManager.ComponentListWidget;
import artisynth.core.gui.selectionManager.SelectionEvent;
import artisynth.core.gui.selectionManager.SelectionFilter;
import artisynth.core.materials.LigamentAxialMaterial;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.ComponentListView;
import artisynth.core.modelbase.ComponentUtils;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.CompositeState;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.MutableCompositeComponent;
import artisynth.core.workspace.RootModel;
import artisynth.models.carapas.Ligament;
import artisynth.models.carapas.VirtualHipFractureSurgery;

public class LigamentAgent<C extends AxialSpring> extends AddComponentAgent<C> {

   private VirtualHipFractureSurgery sim;
   private MechModel myModel;
   protected CompositeComponent myAncestor;
   protected ComponentList<C> myList;
   private boolean myContinuousAdd = false;
   private Point myPointA;
   private Point myPointB;
   private JCheckBox myContinuousAddToggle;
   private LinkedList<ModelComponent> myTmpList =
      new LinkedList<ModelComponent>();
   private CompositeState mySaveState = null;

   private static HashMap<Class,ModelComponent> myPrototypeMap;
   private static RootModel myLastRootModel = null;

   protected void initializePrototype (ModelComponent comp, Class type) {
      if (type == AxialSpring.class) {
         AxialSpring axial = (AxialSpring)comp;
         RenderProps.setLineRadius (axial, getDefaultLineRadius());
         LigamentAxialMaterial mat = new LigamentAxialMaterial();
         axial.setMaterial (mat);
         
      } else if (type == Ligament.class) {
         Ligament axial = (Ligament)comp;
         RenderProps.setLineRadius (axial, getDefaultLineRadius()*0.001);
         LigamentAxialMaterial mat = new LigamentAxialMaterial();
         axial.setMaterial (mat);
         
      }
      else if (type == Muscle.class) {
         Muscle muscle = (Muscle)comp;
         RenderProps.setLineRadius (muscle, getDefaultLineRadius());
      }
   }

   protected void setInitialState() {
      setState (State.SelectingPointA);
   }

   protected void resetState() {
      setState (State.Complete);
   }

   private class PointFilter implements SelectionFilter {
      public boolean objectIsValid (
         ModelComponent c, java.util.List<ModelComponent> currentSelections) {
         return (c instanceof Point &&
                 ComponentUtils.withinHierarchy (c, myAncestor) &&
                 (myState != State.SelectingPointB || c != myPointA));
      }
   }

   private enum State {
      SelectingPointA, SelectingPointB, Confirm, Complete
   };

   private State myState = State.Complete;
   private JComboBox myLigamentSelector;
   

   private void setState (State state) {
      switch (state) {
         case SelectingPointA: {
            installLocationListener();
            myInstructionBox.setText ("Select first point");
            myAddButton.setText ("Stop");
            myAddButton.setActionCommand ("Stop");
            myProgressBox.setText ("");
            installSelectionFilter (new PointFilter());
            //myContinuousAddToggle.setEnabled (false);
            myTmpList.clear();
            break;
         }
         case SelectingPointB: {
            if (myContinuousAdd) {
               myInstructionBox.setText (
                  "Select next point or click 'Stop' to finish");
            }
            else {
               myInstructionBox.setText ("Select second point");
            }
            myAddButton.setText ("Stop");
            myAddButton.setActionCommand ("Stop");
            System.out.println ("setting state B");
            myProgressBox.setText (getPointName (myPointA));
            installSelectionFilter (new PointFilter());
            //myContinuousAddToggle.setEnabled (false);
            break;
         }
         case Complete: {
            uninstallLocationListener();
            myInstructionBox.setText ("Click 'Add' to add more Ligaments or click 'Done' to finish");
            myAddButton.setText ("Add");
            myAddButton.setActionCommand ("Add");
            myProgressBox.setText ("");
            uninstallSelectionFilter();
            //myContinuousAddToggle.setEnabled (true);
            myPointA = null;
            myPointB = null;
            myTmpList.clear();
            break;
         }
         case Confirm: {
            myInstructionBox.setText ("Hit OK to confirm, Done to cancel/quit");
            myAddButton.setText ("OK");
            myAddButton.setActionCommand ("OK");
            myProgressBox.setText (getPointName (myPointA) + " - "
            + getPointName (myPointB));
            uninstallSelectionFilter();
            //myContinuousAddToggle.setEnabled (false);
            myTmpList.clear();
            break;
         }
         default: {
            throw new InternalErrorException ("Unhandled state " + state);
         }
      }
      myState = state;
   }
   
   public LigamentAgent(Main main, VirtualHipFractureSurgery sim) {
      this(main, (ComponentList<C>)sim.getMechMod ().axialSprings(), sim.getMechMod ());
      myModel = sim.getMechMod ();
      this.sim = sim;  
      Ligament proto = (Ligament)this.getPrototypeComponent (Ligament.class);
      LigamentAxialMaterial mat = (LigamentAxialMaterial)proto.getMaterial ();
      mat.setElongStiffness (sim.getSacrotuberalStiffness ());
   }

   public LigamentAgent (Main main, ComponentList<C> list,
   CompositeComponent ancestor) {
      super (main, list, ancestor);
      myList = list;
      myAncestor = ancestor;
   }

   public void setPoints (Point pointA, Point pointB) {
      myPointA = pointA;
      myPointB = pointB;
      setState (State.Confirm);
   }

   protected void createDisplay() {
      createDisplayFrame ("Add Ligaments");

      //addComponentType (AxialSpring.class, new String[] {"name", "restLength", "renderProps"} );
      addComponentType(Ligament.class,new String[] {"name", "restLength", "renderProps"});
      //addComponentType (Muscle.class, new String[] { "excitation", "name" });
      //addBasicProps (Muscle.class, new String[] { "renderProps", "muscleType"
      //   });

      createComponentList ("Existing axial springs:",
                           new AxialSpringListWidget (myList, myAncestor));
      //createSeparator();
      createNameField();
      createTypeSelector ("Spring type");
      createLigamentSelector();

      createPropertyFrame ("Default TYPE propeties:");
      // createSeparator();
      createProgressBox();
      createInstructionBox();
      //createContinuousAddToggle();
      createOptionPanel ("Add Done");
      
   }
   
   String[] ligamentTypes = {"Sacrotuberal", "Inguinal", "Capsule", "Other"};
   
   protected void createLigamentSelector() {
      myLigamentSelector = new JComboBox<String>(ligamentTypes);
      myLigamentSelector.addActionListener (this);
      addToContentPane(myLigamentSelector);
   }

   protected void createContinuousAddToggle() {
      myContinuousAddToggle = new JCheckBox ("add continuously");
      myContinuousAddToggle.setSelected (myContinuousAdd);
      myContinuousAddToggle.addActionListener (this);
      addToContentPane (myContinuousAddToggle);
   }

   public void setContinuousAdd (boolean continuous) {
      if (continuous != myContinuousAdd) {
         myContinuousAdd = continuous;
         if (myContinuousAddToggle != null
             && myContinuousAddToggle.isSelected() != continuous) {
            myContinuousAddToggle.setSelected (continuous);
         }
      }
   }

   public boolean getContinuousAdd() {
      return myContinuousAdd;
   }

   private String getPointName (Point point) {
      return ComponentUtils.getPathName (myAncestor, point);
   }

   @Override
   public void selectionChanged (SelectionEvent e) {
      ModelComponent c = e.getLastAddedComponent();
      if (myState == State.SelectingPointA) {
         if (c instanceof Point) {
            myPointA = (Point)c;
            setState (State.SelectingPointB);
         }
      }
      else if (myState == State.SelectingPointB) {
         if (c instanceof Point) {
            myPointB = (Point)c;
            createAndAddSpring();
            if (!myContinuousAdd) {
               setState (State.Complete);
            }
            else {
               myPointA = myPointB;
               myPointB = null;
               myProgressBox.setText (getPointName (myPointA));
            }
         }
      }
   }
   
   public void handleLocationEvent (GLViewer viewer, MouseRayEvent rayEvent) {
      double mind = Double.POSITIVE_INFINITY;
      RigidBody nearestBody = null;
      Point3d nearestIntersection = null;
      for (RigidBody body : myModel.rigidBodies()) {
         if (body.getMesh() != null) {
            Point3d isectPoint = BVFeatureQuery.nearestPointAlongRay (
               body.getMesh (),
               rayEvent.getRay().getOrigin(), rayEvent.getRay().getDirection());             
            if (isectPoint != null) {
               double d = isectPoint.distance (rayEvent.getRay().getOrigin());
               if (d < mind) {
                  mind = d;
                  nearestBody = body;
                  nearestIntersection = isectPoint;
               }
            }
         }
      }
      if (nearestBody != null) { // transform to body coordinates
         nearestIntersection.inverseTransform (nearestBody.getPose());
         createAndAddMarker (nearestIntersection, nearestBody);
      }
   }
   
   private void createAndAddMarker (Point3d pnt, Frame frame) {
      FrameMarker marker = new FrameMarker (pnt);
      marker.setFrame (frame);
      marker.setName (getNameFieldValue());
      RenderProps.setPointRadius (marker, getDefaultPointRadius());
      RenderProps.setShading (marker, Shading.SMOOTH);
      RenderProps.setPointColor (marker, Color.decode ("#FFCCCC"));
      //setProperties (marker, getPrototypeComponent (myComponentType));
      // update properties in the prototype as well ...
      //setProperties (myPrototype, myPrototype);

      addComponent (new AddComponentsCommand (
         "add FrameMarker", marker,
         (MutableCompositeComponent<?>)myModel.frameMarkers()));

   }

   protected HashMap<Class,ModelComponent> getPrototypeMap() {
      RootModel root = myMain.getRootModel();
      if (root != null && root != myLastRootModel) {
         myPrototypeMap = new HashMap<Class,ModelComponent>();
         myLastRootModel = root;
      }
      return myPrototypeMap;
   }

   private void createAndAddSpring() {
      AxialSpring spring;

      try {
         spring = (AxialSpring)myComponentType.newInstance();
      }
      catch (Exception e) {
         throw new InternalErrorException ("Cannot create instance of "
         + myComponentType + " or cast it to AxialSpring");
      }
      spring.setFirstPoint (myPointA);
      spring.setSecondPoint (myPointB);
      spring.setName (getNameFieldValue());
      myNameField.setValue (null);
      setProperties (spring, getPrototypeComponent (myComponentType));
      // update properties in the prototype as well ...
      setProperties (myPrototype, myPrototype);
      spring.setMaterial (((AxialSpring)getPrototypeComponent (myComponentType)).getMaterial());
      spring.setRestLengthFromPoints ();
      RenderProps.setLineRadius (spring, getDefaultLineRadius());
      RenderProps.setPointRadius (spring, getDefaultPointRadius());
      RenderProps.setLineColor (spring, Color.decode("#FFCCCC"));
      RenderProps.setShading (spring, Shading.SMOOTH);

      if (myContinuousAdd) {
         if (myTmpList.size() == 0) {
            mySaveState = myUndoManager.getModelState();
         }
         myTmpList.add (spring);
         myList.add ((C)spring);
         clearNameField();
         myMain.rerender(); // XXX shouldn't need this
      }
      else {
         uninstallSelectionFilter();
         //addComponent (new AddComponentsCommand (
            //"add AxialSpring", spring, myList));
         sim.registerLigament (spring, (String) myLigamentSelector.getSelectedItem ());
      }
      
      
   }

   private void registerContinuousUndoCommand() {
      LinkedList<MutableCompositeComponent<?>> parents =
         new LinkedList<MutableCompositeComponent<?>>();
      LinkedList<ModelComponent> comps =
         (LinkedList<ModelComponent>)myTmpList.clone();
      for (int i = 0; i < comps.size(); i++) {
         parents.add (myList);
      }
      myUndoManager.addCommand (new AddComponentsCommand (
         "add AxialSprings", comps, parents), mySaveState);
      // select all springs just added
      uninstallSelectionFilter();
      mySelectionManager.clearSelections();
      for (ModelComponent c : comps) {
         mySelectionManager.addSelected (c);
      }
   }

   public void actionPerformed (ActionEvent e) {
      if (e.getSource() == myContinuousAddToggle) {
         System.out.println ("Toggle");
         setContinuousAdd (myContinuousAddToggle.isSelected());
      }
      else if (e.getActionCommand() == "Stop") {
         if (myContinuousAdd && myTmpList.size() > 0) {
            registerContinuousUndoCommand();
         }
         setState (State.Complete);
         myMain.rerender();
      }
      else if (e.getActionCommand() == "OK") {
         createAndAddSpring();
         setState (State.Complete);
         myMain.rerender();
      }
      else if (e.getSource() == myLigamentSelector) {
         AxialSpring proto = (AxialSpring)this.getPrototypeComponent (AxialSpring.class);
         LigamentAxialMaterial mat = (LigamentAxialMaterial)proto.getMaterial ();
         if ((String) myLigamentSelector.getSelectedItem () == "Sacrotuberal") {
            mat.setElongStiffness (sim.getSacrotuberalStiffness ());
         } else if((String) myLigamentSelector.getSelectedItem () == "Inguinal") {
            mat.setElongStiffness (sim.getInguinalStiffness ());;
         } else if((String) myLigamentSelector.getSelectedItem () == "Capsule") {
            mat.setElongStiffness (sim.getCapsuleStiffness ());
         }
         this.myPropertyPanel.updateWidgetValues ();
      }
      else {
         super.actionPerformed (e);
      }
   }

   protected boolean isContextValid() {
      return (ComponentUtils.withinHierarchy (
                 myAncestor, myMain.getRootModel()));
   }

}
