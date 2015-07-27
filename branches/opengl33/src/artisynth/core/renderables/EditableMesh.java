/**
 * Copyright (c) 2014, by the Authors: Antonio Sanchez (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.core.renderables;

import java.io.IOException;
import java.io.PrintWriter;
import java.util.*;

import maspack.geometry.MeshBase;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.properties.HierarchyNode;
import maspack.properties.PropertyUtils;
import maspack.render.Renderer;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.util.IndentingPrintWriter;
import maspack.util.InternalErrorException;
import maspack.util.NumberFormat;
import maspack.util.ReaderTokenizer;
import artisynth.core.mechmodels.*;
import artisynth.core.modelbase.*;
import artisynth.core.modelbase.CompositeComponent.NavpanelDisplay;
import artisynth.core.util.*;

public class EditableMesh extends MeshComponent
   implements CompositeComponent {

   MeshBase myMesh = null;
   VertexList<VertexComponent> myVertexList = null;

   public RenderProps createRenderProps() {
      return new RenderProps();
   }
   
   public EditableMesh(MeshBase mesh) {
      myMesh = mesh;
      myVertexList =
         new VertexList<VertexComponent>(VertexComponent.class, "vertices", "v");
      myComponents =
         new ComponentListImpl<ModelComponent>(ModelComponent.class, this);
      add(myVertexList);
      updateVertices();
   }
   
   public void updateVertices() {
      myVertexList.clear();
      if (myMesh == null) {
         return;
      }
      
      for (Vertex3d vtx : myMesh.getVertices()) {
         myVertexList.add(new VertexComponent(vtx));
      }
   }

   ///////////////////////////////////////////////////
   // Composite component stuff
   ///////////////////////////////////////////////////

   protected ComponentListImpl<ModelComponent> myComponents;
   private NavpanelDisplay myDisplayMode = NavpanelDisplay.NORMAL;

   public void updateNameMap (
      String newName, String oldName, ModelComponent comp) {
      myComponents.updateNameMap (newName, oldName, comp);
   }

   /**
    * {@inheritDoc}
    */
   public ModelComponent findComponent (String path) {
      return ComponentUtils.findComponent (this, path);
   }

   protected void add (ModelComponent comp) {
      myComponents.add (comp);
   }
   
   protected boolean remove (ModelComponent comp) {
      return myComponents.remove (comp);
   }

   /**
    * {@inheritDoc}
    */
   public ModelComponent get (String nameOrNumber) {
      return myComponents.get (nameOrNumber);
   }

   /**
    * {@inheritDoc}
    */
   public ModelComponent get (int idx) {
      return myComponents.get (idx);
   }

   /**
    * {@inheritDoc}
    */
   public ModelComponent getByNumber (int num) {
      return myComponents.getByNumber (num);
   }

   /**
    * {@inheritDoc}
    */
   public int getNumberLimit() {
      return myComponents.getNumberLimit();
   }

   /**
    * {@inheritDoc}
    */
   public int indexOf (ModelComponent comp) {
      return myComponents.indexOf (comp);
   }

   /**
    * {@inheritDoc}
    */
   public int numComponents() {
      return myComponents.size();
   }

   /**
    * {@inheritDoc}
    */
   public void componentChanged (ComponentChangeEvent e) {
      myComponents.componentChanged (e);
      notifyParentOfChange (e);
   }

   protected void notifyStructureChanged (Object comp) {
      if (comp instanceof CompositeComponent) {
         notifyParentOfChange (new StructureChangeEvent (
            (CompositeComponent)comp));
      }
      else {
         notifyParentOfChange (StructureChangeEvent.defaultEvent);
      }
   }

   protected boolean scanItem (ReaderTokenizer rtok, Deque<ScanToken> tokens)
      throws IOException {

      rtok.nextToken();
      if (ScanWriteUtils.scanProperty (rtok, this)) {
         return true;
      }
      else if (myComponents.scanAndStoreComponentByName (rtok, tokens)) {
         return true;
      }
      rtok.pushBack();
      return super.scanItem (rtok, tokens);
   }

   protected boolean postscanItem (
      Deque<ScanToken> tokens, CompositeComponent ancestor) throws IOException {
      
      if (myComponents.postscanComponent (tokens, ancestor)) {
         return true;
      }
      return super.postscanItem (tokens, ancestor);
   }

   @Override
      public void scan (
         ReaderTokenizer rtok, Object ref) throws IOException {

      myComponents.scanBegin();
      super.scan (rtok, ref);
   }

   @Override
   public void postscan (
   Deque<ScanToken> tokens, CompositeComponent ancestor) throws IOException {
      super.postscan (tokens, ancestor);
      myComponents.scanEnd();
   }

   protected void writeComponent (
      PrintWriter pw, ModelComponent comp, NumberFormat fmt, Object ref)
         throws IOException {

      pw.print (comp.getName() + "=");
      if (hierarchyContainsReferences()) {
         comp.write (pw, fmt, this);
      }
      else {
         comp.write (pw, fmt, ref);
      }  
   }

   protected void writeItems (
      PrintWriter pw, NumberFormat fmt, CompositeComponent ancestor)
      throws IOException {

      // properties
      super.writeItems(pw, fmt, ancestor);
      // components
      myComponents.writeComponentsByName (pw, fmt, ancestor);
   }

   public Iterator<? extends HierarchyNode> getChildren() {
      return myComponents.iterator();
   }

   public boolean hasChildren() {
      return myComponents != null && myComponents.size() > 0;
   }

   public void setSelected (boolean selected) {
      super.setSelected (selected);
   }

   /**
    * {@inheritDoc}
    */
   public NavpanelDisplay getNavpanelDisplay() {
      return myDisplayMode;
   }
   
   /**
    * Sets the display mode for this component. This controls
    * how the component is displayed in a navigation panel. The default
    * setting is <code>NORMAL</code>.
    *
    * @param mode new display mode
    */
   public void setDisplayMode (NavpanelDisplay mode) {
      myDisplayMode = mode;
   }

   /**
    * {@inheritDoc}
    */
   public boolean hierarchyContainsReferences() {
      return false;
   }

   public EditableMesh copy (
      int flags, Map<ModelComponent,ModelComponent> copyMap) {

      EditableMesh ccomp =
         (EditableMesh)super.copy (flags, copyMap);

      ccomp.myComponents =
         new ComponentListImpl<ModelComponent>(ModelComponent.class, this);
      ccomp.myDisplayMode = myDisplayMode;

      return ccomp;
   }

   public boolean hasState() {
      return true;
   }

   public void updateBounds (Point3d pmin, Point3d pmax) {
      myVertexList.updateBounds(pmin, pmax);
   }

   @Override
   public void prerender(RenderList list) {
      myMesh.saveRenderInfo();
      list.addIfVisible(myVertexList);
   }
   
   @Override
   public void updateSlavePos () {
      // nothing
   }

   @Override
   public void scaleDistance(double s) {
      myVertexList.scaleDistance(s);
   }

   @Override
   public MeshBase getMesh() {
      return myMesh;
   }

   @Override
   public void render(Renderer renderer, int flags) {      
   }

   public VertexList<VertexComponent> getVertexComponents() {
      return myVertexList;
   }

}
