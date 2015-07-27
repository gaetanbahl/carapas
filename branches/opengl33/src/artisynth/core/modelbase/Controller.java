/**
 * Copyright (c) 2014, by the Authors: John E Lloyd (UBC)
 *
 * This software is freely available under a 2-clause BSD license. Please see
 * the LICENSE file in the ArtiSynth distribution directory for details.
 */
package artisynth.core.modelbase;

public interface Controller extends ModelAgent, HasState {

   /**
    * Called at the beginning of a {@code RootModel}'s advance procedure
    */
   public void apply (double t0, double t1);

}
