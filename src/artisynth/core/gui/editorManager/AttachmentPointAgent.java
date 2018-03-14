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
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.PointStyle;
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
import artisynth.core.mechmodels.Particle;
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

public class AttachmentPointAgent<C extends AxialSpring> extends AddComponentAgent<C> {

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
      setState (State.SelectingPoint);
   }

   protected void resetState() {
      setState (State.Complete);
   }

   private class PointFilter implements SelectionFilter {
      public boolean objectIsValid (
         ModelComponent c, java.util.List<ModelComponent> currentSelections) {
         return (c instanceof Point &&
                 ComponentUtils.withinHierarchy (c, myAncestor) &&
                 ( c != myPointA));
      }
   }

   private enum State {
      SelectingPoint, Confirm, Complete
   };

   private State myState = State.Complete;
   

   private void setState (State state) {
      switch (state) {
         case SelectingPoint: {
            installLocationListener();
            myInstructionBox.setText ("Select attachment point");
            myAddButton.setText ("Stop");
            myAddButton.setActionCommand ("Stop");
            myProgressBox.setText ("");
            installSelectionFilter (new PointFilter());
            //myContinuousAddToggle.setEnabled (false);
            myTmpList.clear();
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
   
   public AttachmentPointAgent(Main main, VirtualHipFractureSurgery sim) {
      this(main, (ComponentList<C>)sim.getMechMod ().axialSprings(), sim.getMechMod ());
      myModel = sim.getMechMod ();
      this.sim = sim;  
      Ligament proto = (Ligament)this.getPrototypeComponent (Ligament.class);
      LigamentAxialMaterial mat = (LigamentAxialMaterial)proto.getMaterial ();
      mat.setElongStiffness (sim.getSacrotuberalStiffness ());
   }

   public AttachmentPointAgent (Main main, ComponentList<C> list,
   CompositeComponent ancestor) {
      super (main, list, ancestor);
      myList = list;
      myAncestor = ancestor;
   }

   public void setPoints (Point pointA, Point pointB) {
      myPointA = pointA;
      myPointB = pointB;
   }

   protected void createDisplay() {
      createDisplayFrame ("Add Attachment Points");

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

      createPropertyFrame ("Default TYPE propeties:");
      // createSeparator();
      createProgressBox();
      createInstructionBox();
      //createContinuousAddToggle();
      createOptionPanel ("Add Done");
      
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
      if (myState == State.SelectingPoint) {
         if (c instanceof Point) {
            //myPointB = (Point)c;
            //createAndAddSpring();
            //setState (State.Complete);
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

         createAndAddSpring();
         setState(State.Complete);
      }
   }
   
   private void createAndAddMarker (Point3d pnt, Frame frame) {
      FrameMarker marker = new FrameMarker (pnt);
      marker.setFrame (frame);
      marker.setFixed (true);
      marker.setName (getNameFieldValue());
      RenderProps.setPointRadius (marker, getDefaultPointRadius());
      RenderProps.setPointColor (marker, Color.decode("#FF3333"));
      RenderProps.setShading (marker, Shading.SMOOTH);
      //setProperties (marker, getPrototypeComponent (myComponentType));
      // update properties in the prototype as well ...
      //setProperties (myPrototype, myPrototype);

      addComponent (new AddComponentsCommand (
         "add FrameMarker", marker,
         (MutableCompositeComponent<?>)myModel.frameMarkers()));
      
      this.myPointB = marker;

   }
   
   private Particle createAndAddParticle(Point pnt) {
      Particle particle = new Particle(1.0, pnt.getPosition ());
      particle.setDynamic (false);
      particle.setFixed (true);
      RenderProps.setPointStyle (particle, PointStyle.POINT);
      
      addComponent (new AddComponentsCommand (
         "add Particle", particle, (MutableCompositeComponent<?>)myModel.particles()));
      return particle;
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
      
      Particle p = createAndAddParticle(myPointB);
      spring.setPoints(myPointB, p);
      
      spring.setName (getNameFieldValue());
      myNameField.setValue (null);
      setProperties (spring, getPrototypeComponent (myComponentType));
      // update properties in the prototype as well ...
      setProperties (myPrototype, myPrototype);
      spring.setMaterial (((AxialSpring)getPrototypeComponent (myComponentType)).getMaterial());
      spring.setRestLengthFromPoints ();
      LigamentAxialMaterial mat = (LigamentAxialMaterial)spring.getMaterial ();
      mat.setElongStiffness (sim.getAttachmentPointsStiffness ());
      RenderProps.setLineRadius (spring, getDefaultLineRadius());
      RenderProps.setPointRadius (p, getDefaultPointRadius());
      RenderProps.setLineStyle (spring, LineStyle.LINE);

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
         addComponent (new AddComponentsCommand (
            "add AxialSpring", spring, myList));
      }
      
      sim.registerLigament (spring, "Attachment Point");
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
      else {
         super.actionPerformed (e);
      }
   }

   protected boolean isContextValid() {
      return (ComponentUtils.withinHierarchy (
                 myAncestor, myMain.getRootModel()));
   }

}
