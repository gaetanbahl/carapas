/**
 * Copyright (c) 2014, by the Authors: John E Lloyd (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package maspack.render;

import maspack.properties.PropertyList;

public class FaceRenderProps extends RenderProps {
   public static PropertyList myProps =
      new PropertyList (FaceRenderProps.class, RenderProps.class);

   static {
      myProps.remove ("lineStyle");
      myProps.remove ("lineColor");
      myProps.remove ("lineWidth");
      myProps.remove ("lineRadius");
      myProps.remove ("lineSlices");

      myProps.remove ("pointStyle");
      myProps.remove ("pointColor");
      myProps.remove ("pointSize");
      myProps.remove ("pointRadius");
      myProps.remove ("pointSlices");

      myProps.remove ("drawEdges");

      myProps.remove ("edgeWidth");
      myProps.remove ("edgeColor");
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

}
