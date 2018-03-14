package artisynth.core.gui.editorManager;

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

public class MeasureAgent extends AddComponentAgent<FrameMarker> {

   private VirtualHipFractureSurgery sim;
   private MechModel myModel;
   protected CompositeComponent myAncestor;
   protected ComponentList<FrameMarker> myList;
   private Point myPointA;
   private Point myPointB;
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
      SelectingPointA, SelectingPointB, Complete
   };

   private State myState = State.Complete;
   

   private void setState (State state) {
      switch (state) {
         case SelectingPointA: {
            installLocationListener();
            myInstructionBox.setText ("Select first point");
            installSelectionFilter (new PointFilter());
            //myContinuousAddToggle.setEnabled (false);
            myTmpList.clear();
            break;
         }
         case SelectingPointB: {
            myInstructionBox.setText ("Select second point");
            
            installSelectionFilter (new PointFilter());
            //myContinuousAddToggle.setEnabled (false);
            break;
         }
         case Complete: {
            uninstallLocationListener();
            uninstallSelectionFilter();
            myProgressBox.setText ("" + myPointA.distance (myPointB));
            myPointA = null;
            myPointB = null;
            myTmpList.clear();
            
            break;
         }
         default: {
            throw new InternalErrorException ("Unhandled state " + state);
         }
      }
      myState = state;
   }
   
   public MeasureAgent(Main main, VirtualHipFractureSurgery sim) {
      this(main, (ComponentList<FrameMarker>)sim.getMechMod ().frameMarkers (), sim.getMechMod ());
      myModel = sim.getMechMod ();
      this.sim = sim;  
   }

   public MeasureAgent (Main main, ComponentList<FrameMarker> list,
   CompositeComponent ancestor) {
      super (main, list, ancestor);
      myList = list;
      myAncestor = ancestor;
   }

   protected void createDisplay() {
      createDisplayFrame ("Measuring Tool");

      //addComponentType (AxialSpring.class, new String[] {"name", "restLength", "renderProps"} );
      addComponentType(FrameMarker.class,new String[] {"name", "renderProps", "position", "velocity", "targetPosition",
                                                       "targetVelocity", "targetActivity", "externalForce", "pointDamping",
                                                       "displacement", "refPos","location"});
      //addComponentType (Muscle.class, new String[] { "excitation", "name" });
      //addBasicProps (Muscle.class, new String[] { "renderProps", "muscleType"
      //   });

      //createSeparator();
      //createNameField();
      //createTypeSelector ("Point type");

      //createPropertyFrame ("Default TYPE propeties:");
      // createSeparator();
      createProgressBox();
      createInstructionBox();
      //createContinuousAddToggle();
      createOptionPanel ("Done");
      
   }
   

   private String getPointName (Point point) {
      return ComponentUtils.getPathName (myAncestor, point);
   }

   
   private void display (double distance) {
      myProgressBox.setText (distance + "");
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
      RenderProps.setPointRadius (marker, getDefaultPointRadius());
      //setProperties (marker, getPrototypeComponent (myComponentType));
      // update properties in the prototype as well ...
      //setProperties (myPrototype, myPrototype);
      if(myState == State.SelectingPointA) {
         myPointA = marker;
         setState(State.SelectingPointB);
      } else if (myState == State.SelectingPointB) {
         myPointB = marker;
         setState(State.Complete);
         setState(State.SelectingPointA);
      }
      
   }

   protected HashMap<Class,ModelComponent> getPrototypeMap() {
      RootModel root = myMain.getRootModel();
      if (root != null && root != myLastRootModel) {
         myPrototypeMap = new HashMap<Class,ModelComponent>();
         myLastRootModel = root;
      }
      return myPrototypeMap;
   }

   public void actionPerformed (ActionEvent e) {
      if (e.getActionCommand() == "OK") {
         this.dispose ();
         uninstallLocationListener();
      } else {
         super.actionPerformed (e);
      }
   }

   protected boolean isContextValid() {
      return (ComponentUtils.withinHierarchy (
                 myAncestor, myMain.getRootModel()));
   }

}
