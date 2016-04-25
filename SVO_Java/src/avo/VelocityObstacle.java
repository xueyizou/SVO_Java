package avo;

import sim.util.Double2D;

public class VelocityObstacle 
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

	public VelocityObstacle()
	{
		apex = new Double2D(0,0);
		side1 = new Double2D(0,0);
		side2 = new Double2D(0,0);
	}

}


/**
 * \class  Candidate
 * \brief  A candidate point.
 */
class Candidate implements Comparable<Object>
{
	/**
	 * \brief  The position of the candidate point.
	 */
	public Double2D position;

	/**
	 * \brief  The ID of the first velocity obstacle.
	 */
	public int velocityObstacle1;

	/**
	 * \brief  The ID of the second velocity obstacle.
	 */
	public int velocityObstacle2;
	/**
	 * \brief  Constructor.
	 */
	public Candidate()
	{
		this.velocityObstacle1=0;
		this.velocityObstacle2=0;
	}
	
	@Override
	public int compareTo(Object arg0) 
	{
		return (this.hashCode()> arg0.hashCode()? 1 : -1);
	}   
	
}