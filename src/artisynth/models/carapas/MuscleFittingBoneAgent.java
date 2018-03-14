package artisynth.models.carapas;

import java.awt.Color;
import java.io.File;
import java.io.IOException;

import javax.swing.Box;

import maspack.geometry.PolygonalMesh;
import maspack.render.RenderProps;
import maspack.render.Renderer.Shading;
import maspack.util.InternalErrorException;
import maspack.widgets.FileNameField;
import maspack.widgets.ValueChangeEvent;
import artisynth.core.driver.Main;
import artisynth.core.gui.editorManager.EditorUtils;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ModelComponent;

public class MuscleFittingBoneAgent extends BoneAgent {

   public MuscleFittingBoneAgent (Main main, MechModel sim) {
      super (main, sim);
      // TODO Auto-generated constructor stub
   }
   
   protected void initializePrototype (ModelComponent comp, Class type) {
      if (type == RigidBody.class) {
         RigidBody mkr = (RigidBody)comp;
         RenderProps.setPointRadius (mkr, getDefaultPointRadius());
         mkr.setDynamic (false);
      }
      else {
         throw new InternalErrorException ("unimplemented type " + type);
      }
   }
   
   protected void createDisplay () {
      createDisplayFrame("Add Bone");

      this.addComponentType (RigidBody.class);
      addBasicProps (RigidBody.class, new String[] {"name" });
      
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
         body.getMesh ().scale (0.001);
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
   
}
