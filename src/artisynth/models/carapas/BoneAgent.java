package artisynth.models.carapas;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import javax.swing.Box;

import maspack.geometry.PolygonalMesh;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.Shading;
import maspack.util.InternalErrorException;
import maspack.widgets.FileNameField;
import maspack.widgets.ValueChangeEvent;

import artisynth.core.driver.Main;
import artisynth.core.gui.editorManager.AddComponentAgent;
import artisynth.core.gui.editorManager.AddComponentsCommand;
import artisynth.core.gui.editorManager.EditorUtils;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.MutableCompositeComponent;
import artisynth.core.workspace.RootModel;

public class BoneAgent extends AddComponentAgent<RigidBody> {
   
   protected MechModel myModel;
   private VirtualHipFractureSurgery simulation;
   protected RigidBody body;
   
   private static RootModel myLastRootModel;
   private static HashMap<Class,ModelComponent> myPrototypeMap;
   
   private enum State {SelectingMesh, AddingBone};
   
   private State currentState;
   
   protected FileNameField fileNameField;
   protected boolean fileCheck = false;
   
   protected final Dimension DEFAULT_BUTTON_DIM = new Dimension(100, 26);
   
   public BoneAgent (Main main, MechModel sim) {
      super (main, (ComponentList<RigidBody>)sim.rigidBodies(), sim);
      myModel = sim;
      simulation = null;
   }

   public BoneAgent (Main main, VirtualHipFractureSurgery sim) {
      super (main, (ComponentList<RigidBody>)sim.getMechMod().rigidBodies(), sim.getMechMod());
      myModel = sim.getMechMod ();
      simulation = sim;
   }

   @Override
   protected void resetState () {
      currentState = State.SelectingMesh;
   }

   @Override
   protected void setInitialState () {
      currentState = State.SelectingMesh;
   }

   protected void initializePrototype (ModelComponent comp, Class type) {
      if (type == RigidBody.class) {
         RigidBody mkr = (RigidBody)comp;
         RenderProps.setPointRadius (mkr, getDefaultPointRadius());
      }
      else {
         throw new InternalErrorException ("unimplemented type " + type);
      }
   }

   protected HashMap<Class, ModelComponent> getPrototypeMap() {
      RootModel root = myMain.getRootModel();
      if (root != null && root != myLastRootModel) {
         myPrototypeMap = new HashMap<Class, ModelComponent>();
         myLastRootModel = root;
      }
      return myPrototypeMap;
   }

   @Override
   protected void createDisplay () {
      createDisplayFrame("Add Bone");

      this.addComponentType (RigidBody.class);
      addBasicProps (RigidBody.class, new String[] {"name", "dynamic" });
      
      createInstructionBox ();
      myInstructionBox.setText ("Select a mesh file");
      addWidget (Box.createHorizontalGlue());
      createPropertyFrame ("General Properties");
      addWidget (Box.createHorizontalGlue());
      fileNameField = new FileNameField("Mesh File Name","",20);
      fileNameField.addValueChangeListener (this);
      addWidget(fileNameField);
      
      createOptionPanel ("Add Done");
      myAddButton = myOptionPanel.getButton ("Add");
      myAddButton.setEnabled (false);
      
      myAddButton.setPreferredSize (DEFAULT_BUTTON_DIM);
      myOptionPanel.getButton ("Done").setPreferredSize (DEFAULT_BUTTON_DIM);
      
   }
   
   private void createBody() {
      body = new RigidBody();
      try {
         PolygonalMesh msh = new PolygonalMesh(fileNameField.getStringValue ()); 
         body.setMesh (msh, fileNameField.getStringValue ());
         simulation.scaleMesh (body, 0.001);
         body.setDensity ((double)simulation.getProperty ("boneDensity").get ());
         body.setInertiaFromDensity (body.getDensity ());
         RenderProps props = myModel.getRenderProps ();
         body.setRenderProps (props);
         body.setName (fileNameField.getStringValue ().substring(fileNameField.getStringValue ().lastIndexOf ("/") +1,fileNameField.getStringValue ().lastIndexOf (".")));
         RenderProps.setShading (body, Shading.SMOOTH);
         RenderProps.setShininess (body, 15.0);
         RenderProps.setFaceColor (body, Color.decode ("#E3DAC9"));
      }
      catch (IOException e) {
         e.printStackTrace();
      }
   }
   
   public void valueChange (ValueChangeEvent evt) {
      Object source = evt.getSource();
      
      if (source == fileNameField) {
         File file = new File (fileNameField.getStringValue());
         
         if (file.isFile()) {            
            fileCheck  = true;
            createBody();
            updateState();
         }
         else {
            fileCheck = false;
            
            EditorUtils.showError (
               myDisplay, "File does not exist:" + file.getAbsolutePath()); 
         }
      }
      else {
         super.valueChange (evt);
      }   
   
   }
   
   protected void updateState() {
      if (!fileCheck) {
         setState (State.SelectingMesh);
      }
      else {
         setState (State.AddingBone);
      }
   }
   
   private void setState (State state) {
      if (currentState != state) {
         switch (state) {
            case AddingBone: {
               myInstructionBox.setText (
                  "Click 'Add' to finish adding the rigid body");
               myAddButton.setEnabled (true);
               break;
            }
            case SelectingMesh: {
               myInstructionBox.setText ("Specify a valid mesh file");
               myAddButton.setEnabled (false);
               break;
            }
            default: {
               throw new InternalErrorException ("Unhandled state " + state);
            }
         }
         
         currentState = state;
      }
   }
   
   public void actionPerformed (ActionEvent evt) {
      String cmd = evt.getActionCommand();
      
      if (cmd.equals ("Add")) {
         
         //setProperties (body, getPrototypeComponent (myComponentType));
         //setProperties (myPrototype, myPrototype);
         
         body.setDynamic (((RigidBody)getPrototypeComponent(myComponentType)).isDynamic ());
         
         addComponent (new AddComponentsCommand (
               "add RigidBody", body, (MutableCompositeComponent<?>)myModel.rigidBodies()));

         myMain.getViewer ().autoFit ();
         myMain.rerender();
         
         //myDisplay.setVisible (false);
         //dispose();
         
         resetState();
      }
      else if (cmd.equals ("Done")) {
         myDisplay.setVisible (false);
         dispose();
      }
      else {
         super.actionPerformed (evt);
      }
   }

}
