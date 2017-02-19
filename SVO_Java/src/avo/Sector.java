/* *************************************************************************************
 * Copyright (C) Xueyi Zou - All Rights Reserved
 * Written by Xueyi Zou <xz972@york.ac.uk>, 2015
 * You are free to use/modify/distribute this file for whatever purpose!
 -----------------------------------------------------------------------
 |THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
 |WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
 |AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
 |DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
 |MISUSING THIS SOFTWARE.
 ------------------------------------------------------------------------
 **************************************************************************************/

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
