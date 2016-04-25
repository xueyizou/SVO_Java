package avo;

import sim.util.Double2D;

public class Sector 
{
	
	/**
	 * \brief  The position of the apex of the hybrid reciprocal velocity obstacle.
	 */
	public Double2D apex;

	/**
	 * \brief  The direction of the first side of the hybrid reciprocal velocity obstacle.
	 */
	public Double2D side1;

	/**
	 * \brief  The direction of the second side of the hybrid reciprocal velocity obstacle.
	 */
	public Double2D side2;
	
	public double r;

	public Sector()
	{
		apex = new Double2D(0,0);
		side1 = new Double2D(0,0);
		side2 = new Double2D(0,0);
		r=0;
	}

}
