package avo;

import sim.util.Double2D;

public class Goal
{

	protected int goalID;
	private Double2D position;
	
	public Goal( Double2D position)
	{
		this.position = position;
	}

	public Double2D getPosition() {
		return position;
	}

	public void setPosition(Double2D position) {
		this.position = position;
	}

}
