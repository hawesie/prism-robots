//==============================================================================
//	
//	Copyright (c) 2002-
//	Authors:
//	* Dave Parker <david.parker@comlab.ox.ac.uk> (University of Oxford)
//	
//------------------------------------------------------------------------------
//	
//	This file is part of PRISM.
//	
//	PRISM is free software; you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation; either version 2 of the License, or
//	(at your option) any later version.
//	
//	PRISM is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.
//	
//	You should have received a copy of the GNU General Public License
//	along with PRISM; if not, write to the Free Software Foundation,
//	Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//	
//==============================================================================

package explicit;

import java.io.FileWriter;
import java.io.IOException;
import java.util.BitSet;
import java.util.Iterator;
import java.util.Map;
import java.util.TreeMap;

import explicit.rewards.MCRewards;

import prism.ModelType;
import prism.PrismException;
import prism.PrismLog;
import prism.PrismUtils;

/**
 * Base class for explicit-state representations of a DTMC.
 */
public abstract class DTMCExplicit extends ModelExplicit implements DTMC
{
	// Accessors (for Model)
	
	@Override
	public ModelType getModelType()
	{
		return ModelType.DTMC;
	}

	@Override
	public String infoString()
	{
		String s = "";
		s += numStates + " states (" + getNumInitialStates() + " initial)";
		s += ", " + getNumTransitions() + " transitions";
		return s;
	}

	@Override
	public String infoStringTable()
	{
		String s = "";
		s += "States:      " + numStates + " (" + getNumInitialStates() + " initial)\n";
		s += "Transitions: " + getNumTransitions() + "\n";
		return s;
	}
	
	@Override
	public void exportToPrismExplicitTra(PrismLog out)
	{
		int i;
		TreeMap<Integer, Double> sorted;
		// Output transitions to .tra file
		out.print(numStates + " " + getNumTransitions() + "\n");
		sorted = new TreeMap<Integer, Double>();
		for (i = 0; i < numStates; i++) {
			// Extract transitions and sort by destination state index (to match PRISM-exported files)
			Iterator<Map.Entry<Integer, Double>> iter = getTransitionsIterator(i);
			while (iter.hasNext()) {
				Map.Entry<Integer, Double> e = iter.next();
				sorted.put(e.getKey(), e.getValue());
			}
			// Print out (sorted) transitions
			for (Map.Entry<Integer, Double> e : sorted.entrySet()) {
				// Note use of PrismUtils.formatDouble to match PRISM-exported files
				out.print(i + " " + e.getKey() + " " + PrismUtils.formatDouble(e.getValue()) + "\n");
			}
			sorted.clear();
		}
	}

	@Override
	public void exportToDotFile(PrismLog out, BitSet mark)
	{
		int i;
		out.print("digraph " + getModelType() + " {\nsize=\"8,5\"\nnode [shape=box];\n");
		for (i = 0; i < numStates; i++) {
			if (mark != null && mark.get(i))
				out.print(i + " [style=filled  fillcolor=\"#cccccc\"]\n");
			Iterator<Map.Entry<Integer, Double>> iter = getTransitionsIterator(i);
			while (iter.hasNext()) {
				Map.Entry<Integer, Double> e = iter.next();
				out.print(i + " -> " + e.getKey() + " [ label=\"");
				out.print(e.getValue() + "\" ];\n");
			}
		}
		out.print("}\n");
	}

	@Override
	public void exportToPrismLanguage(String filename) throws PrismException
	{
		int i;
		boolean first;
		FileWriter out;
		TreeMap<Integer, Double> sorted;
		try {
			// Output transitions to PRISM language file
			out = new FileWriter(filename);
			out.write(getModelType().keyword() + "\n");
			out.write("module M\nx : [0.." + (numStates - 1) + "];\n");
			sorted = new TreeMap<Integer, Double>();
			for (i = 0; i < numStates; i++) {
				// Extract transitions and sort by destination state index (to match PRISM-exported files)
				Iterator<Map.Entry<Integer, Double>> iter = getTransitionsIterator(i);
				while (iter.hasNext()) {
					Map.Entry<Integer, Double> e = iter.next();
					sorted.put(e.getKey(), e.getValue());
				}
				// Print out (sorted) transitions
				out.write("[]x=" + i + "->");
				first = true;
				for (Map.Entry<Integer, Double> e : sorted.entrySet()) {
					if (first)
						first = false;
					else
						out.write("+");
					// Note use of PrismUtils.formatDouble to match PRISM-exported files
					out.write(PrismUtils.formatDouble(e.getValue()) + ":(x'=" + e.getKey() + ")");
				}
				out.write(";\n");
				sorted.clear();
			}
			out.write("endmodule\n");
			out.close();
		} catch (IOException e) {
			throw new PrismException("Could not export " + getModelType() + " to file \"" + filename + "\"" + e);
		}
	}
	
	// Accessors (for DTMC)
	
	@Override
	public void mvMult(double vect[], double result[], BitSet subset, boolean complement)
	{
		int s;
		// Loop depends on subset/complement arguments
		if (subset == null) {
			for (s = 0; s < numStates; s++)
				result[s] = mvMultSingle(s, vect);
		} else if (complement) {
			for (s = subset.nextClearBit(0); s < numStates; s = subset.nextClearBit(s + 1))
				result[s] = mvMultSingle(s, vect);
		} else {
			for (s = subset.nextSetBit(0); s >= 0; s = subset.nextSetBit(s + 1))
				result[s] = mvMultSingle(s, vect);
		}
	}

	@Override
	public double mvMultGS(double vect[], BitSet subset, boolean complement, boolean absolute)
	{
		int s;
		double d, diff, maxDiff = 0.0;
		// Loop depends on subset/complement arguments
		if (subset == null) {
			for (s = 0; s < numStates; s++) {
				d = mvMultJacSingle(s, vect);
				diff = absolute ? (Math.abs(d - vect[s])) : (Math.abs(d - vect[s]) / d);
				maxDiff = diff > maxDiff ? diff : maxDiff;
				vect[s] = d;
			}
		} else if (complement) {
			for (s = subset.nextClearBit(0); s < numStates; s = subset.nextClearBit(s + 1)) {
				d = mvMultJacSingle(s, vect);
				diff = absolute ? (Math.abs(d - vect[s])) : (Math.abs(d - vect[s]) / d);
				maxDiff = diff > maxDiff ? diff : maxDiff;
				vect[s] = d;
			}
		} else {
			for (s = subset.nextSetBit(0); s >= 0; s = subset.nextSetBit(s + 1)) {
				d = mvMultJacSingle(s, vect);
				diff = absolute ? (Math.abs(d - vect[s])) : (Math.abs(d - vect[s]) / d);
				maxDiff = diff > maxDiff ? diff : maxDiff;
				vect[s] = d;
			}
			// Use this code instead for backwards Gauss-Seidel
			/*for (s = numStates - 1; s >= 0; s--) {
				if (subset.get(s)) {
					d = mvMultJacSingle(s, vect);
					diff = absolute ? (Math.abs(d - vect[s])) : (Math.abs(d - vect[s]) / d);
					maxDiff = diff > maxDiff ? diff : maxDiff;
					vect[s] = d;
				}
			}*/
		}
		return maxDiff;
	}

	@Override
	public void mvMultRew(double vect[], MCRewards mcRewards, double result[], BitSet subset, boolean complement)
	{
		int s;
		// Loop depends on subset/complement arguments
		if (subset == null) {
			for (s = 0; s < numStates; s++)
				result[s] = mvMultRewSingle(s, vect, mcRewards);
		} else if (complement) {
			for (s = subset.nextClearBit(0); s < numStates; s = subset.nextClearBit(s + 1))
				result[s] = mvMultRewSingle(s, vect, mcRewards);
		} else {
			for (s = subset.nextSetBit(0); s >= 0; s = subset.nextSetBit(s + 1))
				result[s] = mvMultRewSingle(s, vect, mcRewards);
		}
	}
}
