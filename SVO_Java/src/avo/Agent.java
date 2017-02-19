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

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Iterator;
import com.google.common.collect.TreeMultimap;

import ec.util.MersenneTwisterFast;
import sim.util.Double2D;
import sim.util.MutableDouble2D;

public class Agent
{
	 protected int id = 0;
	 protected Double2D position = new Double2D();
	 protected int goalID;
	 protected double neighborDist = 0.0;
	 protected int maxNeighbors = 0;
	 protected double timeHorizon = 0.0;
	 protected double timeHorizonObst = 0.0;
     protected double radius = 0.0;
     protected double goalRadius = 0.0;
     protected double prefSpeed = 0.0;
     protected double maxSpeed = 0.0;
     protected double minSpeed = 0.0;
	 protected double alpha = 0.0;

     protected double maxAccel = Double.POSITIVE_INFINITY;
     protected double maxTurn = Double.POSITIVE_INFINITY;
     protected Double2D velocity = new Double2D();
     
	 protected Double2D prefVelocity= new Double2D();    
	 protected MutableDouble2D newVelocity = new MutableDouble2D(); 
	 protected boolean reachedGoal = false;
	 
     protected ArrayList<AbstractMap.SimpleEntry<Double, Agent>> agentNeighbors = new ArrayList<AbstractMap.SimpleEntry<Double, Agent>>();
	 protected ArrayList<Line> orcaLines = new ArrayList<Line>();
	 protected ArrayList<VelocityObstacle> velocityObstacles = new ArrayList<VelocityObstacle>();
	 protected TreeMultimap<Double,Candidate> candidates = TreeMultimap.create();//<Double,Candidate>. ;
	 
	 private AVOSimulator sim;
	public Agent(AVOSimulator orcaSimulator)
     {
         sim = orcaSimulator;
     }


     protected void computeNeighbors()
	 {            
	     agentNeighbors.clear();
	     MutableDouble rangeSq = new MutableDouble(AVOMath.sqr(timeHorizon * maxSpeed + radius));
	     
	     if (maxNeighbors > 0)
	     {
	         rangeSq.doubleValue = AVOMath.sqr(neighborDist);
	         sim.kdTree.queryAgentNeighbors(this, rangeSq);
	     }
	     
	    
	 }
     
     protected int computeNewVelocity()
     {
    	 int mode =-1;
    	 if(alpha >=0 && alpha<1.0)
    	 {//RVO
//    		 System.out.println("AVO-RVO IS RUNNING!");
    		 int sampleCount = 200;
    		 double penaltyCoef = 10;
    		 computeRVONewVelocity(sampleCount,penaltyCoef);
    	 }
    	 else if (Math.abs(alpha-1.0)<AVOMath.AVO_EPSILON)
    	 {//VO
//    		 System.out.println("AVO-VO IS RUNNING!");
    		 computeVONewVelocity();
    	 }
    	 else if (alpha >1.0 && alpha<=2.0)
    	 {//HRVO
//    		 System.out.println("AVO-HRVO IS RUNNING!");
    		 computeHRVONewVelocity();
    	 }
    	 else if (alpha >2.0 && alpha<=3.0)
    	 {//ORCA
    		 //System.out.println("AVO-ORCA IS RUNNING!");
    		 computeORCANewVelocity();
    	 }
    	 else
    	 {
    		// System.out.println("AVO-SVO IS RUNNING!");
    		 mode=computeSVONewVelocity();
    	 }
    	
    	 return mode;
     }
     
 	/**
 	 * \brief  Computes the preferred velocity of this agent.
 	 */
 	public void computePreferredVelocity()
 	{
 		
		final Double2D goalPosition = sim.goals.get(goalID).getPosition();
 		final double distSqToGoal = goalPosition.distanceSq(position);

 		if (Math.pow(prefSpeed * sim.getTimeStep(),2) > distSqToGoal)
 		{
 			prefVelocity = goalPosition.subtract(position).multiply(1.0/ sim.getTimeStep());
 			
 		}
 		else
 		{
//	 		prefVelocity = goalPosition.subtract(position).multiply(prefSpeed/Math.sqrt(distSqToGoal));
 			prefVelocity = goalPosition.subtract(position).normalize().multiply(prefSpeed); 
 			
 		}
 		
 		
 	}


	protected void update()
	{
		final double dv = new Double2D(newVelocity).length() - velocity.length(); 
		final double dTheta = velocity.rotateAngleToDouble2D(new Double2D(newVelocity));
		
		if (Math.abs(dv)<=maxAccel && Math.abs(dTheta)<= maxTurn )
		{
			velocity = new Double2D(newVelocity);
		} 
		else if(Math.abs(dv)>maxAccel && Math.abs(dTheta)<= maxTurn)
		{			
			velocity = velocity.add(velocity.normalize().multiply(maxAccel*dv/Math.abs(dv))).rotate(dTheta);
				
		}
		else if (Math.abs(dv)<=maxAccel && Math.abs(dTheta)>maxTurn)
		{
			velocity = velocity.normalize().multiply(new Double2D(newVelocity).length()).rotate(maxTurn*dTheta/Math.abs(dTheta));
		}
		else
		{			
			velocity =velocity.add(velocity.normalize().multiply(maxAccel*dv/Math.abs(dv))).rotate(maxTurn*dTheta/Math.abs(dTheta));			
		}


//		velocity = new Double2D(newVelocity);
		
		position = position.add(velocity.multiply(sim.timeStep) );

		if (sim.goals.get(goalID).getPosition().distanceSq(position) < AVOMath.sqr(radius + goalRadius))
		{
			reachedGoal = true;
		}
		else 
		{
			reachedGoal = false;
			sim.setAllReachedGoals(false);
		}
		
	}

	 protected void insertAgentNeighbor(Agent otherAgent, MutableDouble rangeSq)
	 {
	     if (this != otherAgent)
	     {
	         double distSq = AVOMath.absSq(position.subtract(otherAgent.position));
	
	         if (distSq < rangeSq.doubleValue)
	         {
	             if (agentNeighbors.size() < maxNeighbors)
	             {
	                 agentNeighbors.add(new AbstractMap.SimpleEntry<Double, Agent>(distSq, otherAgent));
	             }
	             
	             int i = agentNeighbors.size() - 1;
	             while (i != 0 && distSq < agentNeighbors.get(i - 1).getKey())
	             {
	                 agentNeighbors.set(i, agentNeighbors.get(i - 1)) ;
	                 --i;
	             }
	             agentNeighbors.set(i, new AbstractMap.SimpleEntry<Double, Agent>(distSq, otherAgent));
	
	             if (agentNeighbors.size() == maxNeighbors)
	             {
	                 rangeSq.doubleValue = agentNeighbors.get(agentNeighbors.size()-1).getKey();
	             }
	         }
	     }
	 }
	 
/*****************************************************************************************************************************/	 
	 
	 
	/* Search for the best new velocity according to RVO */
	 protected void computeRVONewVelocity(int sampleCount,double penaltyCoef)
	 {
		 double min_penalty = Double.POSITIVE_INFINITY;
	     Double2D vCand;
	     
	     MersenneTwisterFast rand = sim.rand;
	    	     		 
	     // Select sampleCount candidate velocities within the circle of radius maxSpeed
		 for (int n = 0; n < sampleCount; ++n)			
		 {
			 if (n == 0)
		     {
		        vCand = prefVelocity;
		     } 
		     else
		     {
		    	 int theta = rand.nextInt(360);
		    	 double speed = maxSpeed*(1-rand.nextDouble());		    	 
		    	 vCand = new Double2D(speed*Math.cos(theta),speed*Math.sin(theta));
		     }

		     double dV; // distance between candidate velocity and preferred velocity
		     dV = vCand.distance(prefVelocity);		     

		     // searching for smallest time to collision
		     double ct = Double.POSITIVE_INFINITY; // time to collision		      
			 for (int i=0; i< agentNeighbors.size(); ++i)
			 { 
				 Agent agenti = agentNeighbors.get(i).getValue();
		
			     double ct_i; // time to collision with agent i
			       
			     Double2D relV;		        
			     relV = vCand.multiply(1.0/alpha).add(velocity.multiply(1.0 - 1.0/alpha)).subtract(agenti.velocity);
			     ct_i = AVOMath.timeToCollision(position, relV, agenti.position, radius + agenti.radius, false);
			       			        	        
			     if (ct_i < ct)
			     {
		        	ct = ct_i;
			        // pruning search if no better penalty can be obtained anymore for this velocity
			        if ( penaltyCoef / ct + dV >= min_penalty)
			        {
			            break;
			        }
			     }
		      }

			 
		      double penalty = penaltyCoef / ct + dV;
		      if (penalty < min_penalty)
		      {
		    	  min_penalty = penalty;
		    	  newVelocity.setTo(vCand);
		      }
		   		      
		 }
		 
//		 System.out.println(newVelocity.x);
	 }

	 /* Search for the best new velocity according to original VO */
	 protected void computeVONewVelocity()
	 {
		velocityObstacles.clear();
		velocityObstacles.ensureCapacity(agentNeighbors.size());
		
		for (int i=0; i< agentNeighbors.size(); ++i)
		{//			
			Agent other = agentNeighbors.get(i).getValue();

			VelocityObstacle velocityObstacle = new VelocityObstacle();
			/* check for collision*/
			if (AVOMath.absSq(other.position.subtract(position) ) > AVOMath.sqr(other.radius + radius))
			{
				double angle = AVOMath.atan(other.position.subtract(position) );
				double openingAngle = Math.asin((other.radius + radius) / AVOMath.abs(other.position.subtract(position)));
				
				velocityObstacle.side1= new Double2D(Math.cos(angle - openingAngle), Math.sin(angle - openingAngle));
				velocityObstacle.side2= new Double2D(Math.cos(angle + openingAngle), Math.sin(angle + openingAngle));
				velocityObstacle.apex = other.velocity;
				velocityObstacles.add(velocityObstacle);
			}
			else 
			{
				//The else branch is rarely executed
				System.err.println("collision!out of my ability!");
			}
		}
				
		 double min_penalty = Double.POSITIVE_INFINITY;
	     Double2D vCand;
	     
	     MersenneTwisterFast rand = sim.rand;
	     int sampleCount=200;
	     double penaltyCoef =100;
	    	     		 
	     // Select sampleCount candidate velocities within the circle of radius maxSpeed
		 for (int n = 0; n < sampleCount; ++n)			
		 {
			 vCand = velocity.add(velocity.normalize().multiply(maxAccel*(2*rand.nextDouble()-1)));	
			 if(vCand.length()>maxSpeed)
			 {
				 vCand = vCand.normalize().multiply(maxSpeed);
			 }
			 
			 if(vCand.length()<minSpeed)
			 {
				 vCand = vCand.normalize().multiply(minSpeed);
			 }
			 
			 vCand = vCand.rotate(maxTurn*(2*rand.nextDouble()-1));

		     double dV; // distance between candidate velocity and preferred velocity
		     dV = vCand.distance(prefVelocity);		     

		     // searching for smallest time to collision
		     double ct = Double.POSITIVE_INFINITY; // time to collision		      
			 for (int i=0; i< agentNeighbors.size(); ++i)
			 { 
				 Agent agenti = agentNeighbors.get(i).getValue();
		
			     double ct_i; // time to collision with agent i
			       
			     Double2D relV;		        
			     relV = vCand.subtract(agenti.velocity);
			     ct_i = AVOMath.timeToCollision(position, relV, agenti.position, radius + agenti.radius, false);
			       			        	        
			     if (ct_i < ct)
			     {
		        	ct = ct_i;
			        // pruning search if no better penalty can be obtained anymore for this velocity
			        if ( penaltyCoef / ct + dV >= min_penalty)
			        {
			            break;
			        }
			     }
		      }

			 
		      double penalty = penaltyCoef / ct + dV;
		      if (penalty < min_penalty)
		      {
		    	  min_penalty = penalty;
		    	  newVelocity.setTo(vCand);
		      }
		   		      
		 }		
		
		
	 }

	 
	/* Search for the best new velocity according to HRVO */
	 protected void computeHRVONewVelocity()
	 {
		velocityObstacles.clear();
		velocityObstacles.ensureCapacity(agentNeighbors.size());

		
		for (int i=0; i< agentNeighbors.size(); ++i)
		{
			Agent other = agentNeighbors.get(i).getValue();

			VelocityObstacle velocityObstacle = new VelocityObstacle();
			/* check for collision*/
			if (AVOMath.absSq(other.position.subtract(position) ) > AVOMath.sqr(other.radius + radius))
			{
				double angle = AVOMath.atan(other.position.subtract(position) );
				double openingAngle = Math.asin((other.radius + radius) / AVOMath.abs(other.position.subtract(position)));
				
				velocityObstacle.side1= new Double2D(Math.cos(angle - openingAngle), Math.sin(angle - openingAngle));
				velocityObstacle.side2= new Double2D(Math.cos(angle + openingAngle), Math.sin(angle + openingAngle));

				double d = 2.0 * Math.sin(openingAngle) * Math.cos(openingAngle);
				/* check for which side of the centerline the preferred velocity lies*/
				if (AVOMath.det(other.position.subtract(position), velocity.subtract(other.velocity)) > 0.0)
				{
					//Double2D RVOApexVector = other.velocity.multiply(alpha-1.0).add( velocity.multiply(1.0 - (alpha-1.0)) );
					double s = 0.5* AVOMath.det(velocity.subtract(other.velocity), velocityObstacle.side2) / d;
					velocityObstacle.apex = other.velocity.add(velocityObstacle.side1.multiply(s));
				}
				else 
				{
					//Double2D RVOApexVector = other.velocity.multiply(alpha).add( velocity.multiply(1.0 - alpha) );
					double s = 0.5* AVOMath.det(velocity.subtract(other.velocity), velocityObstacle.side1) / d;
					velocityObstacle.apex = other.velocity.add( velocityObstacle.side2.multiply(s));	
				}	
				
				if(alpha==1.0)
				velocityObstacle.apex = other.velocity.multiply(alpha).add(velocity.multiply(1.0 - alpha));

				velocityObstacles.add(velocityObstacle);

			}
			else 
			{
				//The else branch is rarely executed
				System.out.println("collision!out of my ability!");
			}
		}

		candidates.clear();

		{
			Candidate candidate = new Candidate();
			candidate.velocityObstacle1 = Integer.MAX_VALUE;
			candidate.velocityObstacle2 = Integer.MAX_VALUE;

			if (AVOMath.absSq(prefVelocity) < maxSpeed * maxSpeed)
			{
				candidate.position = prefVelocity;
			}
			else 
			{
				System.out.println(prefVelocity.x);
				candidate.position = AVOMath.normalize(prefVelocity).multiply(maxSpeed);
			}

			candidates.put(AVOMath.absSq(prefVelocity.subtract(candidate.position)), candidate);
		}
		
		
		for (int i = 0; i < velocityObstacles.size(); ++i)
		{
			double dotProduct1 = prefVelocity.subtract(velocityObstacles.get(i).apex).dot(velocityObstacles.get(i).side1);
			double dotProduct2 = prefVelocity.subtract(velocityObstacles.get(i).apex).dot(velocityObstacles.get(i).side2);

			if (dotProduct1 > 0.0 && AVOMath.det(velocityObstacles.get(i).side1, prefVelocity.subtract(velocityObstacles.get(i).apex) ) > 0.0)
			{
				Candidate candidate = new Candidate();
				candidate.velocityObstacle1 = i;
				candidate.velocityObstacle2= i;
				candidate.position = velocityObstacles.get(i).apex.add(velocityObstacles.get(i).side1.multiply(dotProduct1));

				if (AVOMath.absSq(candidate.position) < maxSpeed * maxSpeed)
				{
					candidates.put(AVOMath.absSq(prefVelocity.subtract(candidate.position)), candidate);
				}
			}

			if (dotProduct2 > 0.0 && AVOMath.det(velocityObstacles.get(i).side2, prefVelocity.subtract(velocityObstacles.get(i).apex) ) < 0.0)
			{
				Candidate candidate = new Candidate();
				candidate.velocityObstacle1 = i;
				candidate.velocityObstacle2= i;
				candidate.position = velocityObstacles.get(i).apex.add(velocityObstacles.get(i).side2.multiply(dotProduct2));

				if (AVOMath.absSq(candidate.position) < maxSpeed * maxSpeed)
				{
					candidates.put(AVOMath.absSq(prefVelocity.subtract(candidate.position)), candidate);
				}
			}
		}

		for (int j = 0; j < velocityObstacles.size(); ++j)
		{
			double discriminant = maxSpeed * maxSpeed - AVOMath.sqr(AVOMath.det(velocityObstacles.get(j).apex, velocityObstacles.get(j).side1));

			if (discriminant > 0.0)
			{
				double t1 = -(velocityObstacles.get(j).apex.dot(velocityObstacles.get(j).side1)) + AVOMath.sqrt(discriminant);
				double t2 = -(velocityObstacles.get(j).apex.dot(velocityObstacles.get(j).side1)) - AVOMath.sqrt(discriminant);

				if (t1 >= 0.0)
				{
					Candidate candidate = new Candidate();
					candidate.velocityObstacle1 = Integer.MAX_VALUE;
					candidate.velocityObstacle2 = j;
					candidate.position = velocityObstacles.get(j).apex.add( velocityObstacles.get(j).side1.multiply(t1) );
					candidates.put(AVOMath.absSq(prefVelocity.subtract(candidate.position)), candidate);
				}

				if (t2 >= 0.0)
				{
					Candidate candidate = new Candidate();
					candidate.velocityObstacle1 = Integer.MAX_VALUE;
					candidate.velocityObstacle2 = j;
					candidate.position = velocityObstacles.get(j).apex.add( velocityObstacles.get(j).side1.multiply(t2) );
					candidates.put(AVOMath.absSq(prefVelocity.subtract(candidate.position)), candidate);
				}
			}

			discriminant = maxSpeed * maxSpeed - AVOMath.sqr(AVOMath.det(velocityObstacles.get(j).apex, velocityObstacles.get(j).side2));

			if (discriminant > 0.0)
			{
				double t1 = -(velocityObstacles.get(j).apex.dot( velocityObstacles.get(j).side2) ) + AVOMath.sqrt(discriminant);
				double t2 = -(velocityObstacles.get(j).apex.dot( velocityObstacles.get(j).side2) ) - AVOMath.sqrt(discriminant);

				if (t1 >= 0.0)
				{
					Candidate candidate = new Candidate();
					candidate.velocityObstacle1 = Integer.MAX_VALUE;
					candidate.velocityObstacle2 = j;
					candidate.position = velocityObstacles.get(j).apex.add( velocityObstacles.get(j).side2.multiply(t1) );
					candidates.put(AVOMath.absSq(prefVelocity.subtract(candidate.position)), candidate);
				}

				if (t2 >= 0.0)
				{
					Candidate candidate = new Candidate();
					candidate.velocityObstacle1 = Integer.MAX_VALUE;
					candidate.velocityObstacle2 = j;
					candidate.position = velocityObstacles.get(j).apex.add( velocityObstacles.get(j).side2.multiply(t2) );
					candidates.put(AVOMath.absSq(prefVelocity.subtract(candidate.position)), candidate);
				}
			}
		}

		for (int i = 0; i < velocityObstacles.size() - 1; ++i)
		{
			for (int j = i + 1; j < velocityObstacles.size(); ++j)
			{
				double d = AVOMath.det(velocityObstacles.get(i).side1, velocityObstacles.get(j).side1);

				if (d != 0.0)
				{
					double s = AVOMath.det(velocityObstacles.get(j).apex.subtract( velocityObstacles.get(i).apex ), velocityObstacles.get(j).side1) / d;
					double t = AVOMath.det(velocityObstacles.get(j).apex.subtract( velocityObstacles.get(i).apex ), velocityObstacles.get(i).side1) / d;

					if (s >= 0.0 && t >= 0.0)
					{
						Candidate candidate = new Candidate();
						candidate.velocityObstacle1 = i;
						candidate.velocityObstacle2= j;
						candidate.position = velocityObstacles.get(i).apex.add( velocityObstacles.get(i).side1.multiply(s) );

						if (AVOMath.absSq(candidate.position) < maxSpeed * maxSpeed)
						{
							candidates.put(AVOMath.absSq(prefVelocity.subtract(candidate.position)), candidate);
						}
					}
				}

				d =AVOMath.det(velocityObstacles.get(i).side2, velocityObstacles.get(j).side1);

				if (d != 0.0)
				{
					double s = AVOMath.det(velocityObstacles.get(j).apex.subtract( velocityObstacles.get(i).apex ), velocityObstacles.get(j).side1) / d;
					double t = AVOMath.det(velocityObstacles.get(j).apex.subtract( velocityObstacles.get(i).apex ), velocityObstacles.get(i).side2) / d;

					if (s >= 0.0 && t >= 0.0)
					{
						Candidate candidate = new Candidate();
						candidate.velocityObstacle1 = i;
						candidate.velocityObstacle2= j;
						candidate.position = velocityObstacles.get(i).apex.add( velocityObstacles.get(i).side2.multiply(s) );

						if (AVOMath.absSq(candidate.position) < maxSpeed * maxSpeed)
						{
							candidates.put(AVOMath.absSq(prefVelocity.subtract(candidate.position)), candidate);
						}
					}
				}

				d = AVOMath.det(velocityObstacles.get(i).side1, velocityObstacles.get(j).side2);

				if (d != 0.0)
				{
					double s = AVOMath.det(velocityObstacles.get(j).apex.subtract( velocityObstacles.get(i).apex ), velocityObstacles.get(j).side2) / d;
					double t = AVOMath.det(velocityObstacles.get(j).apex.subtract( velocityObstacles.get(i).apex ), velocityObstacles.get(i).side1) / d;

					if (s >= 0.0 && t >= 0.0)
					{
						Candidate candidate = new Candidate();
						candidate.velocityObstacle1 = i;
						candidate.velocityObstacle2= j;
						candidate.position = velocityObstacles.get(i).apex.add( velocityObstacles.get(i).side1.multiply(s) );

						if (AVOMath.absSq(candidate.position) < maxSpeed * maxSpeed)
						{
							candidates.put(AVOMath.absSq(prefVelocity.subtract(candidate.position)), candidate);
						}
					}
				}

				d = AVOMath.det(velocityObstacles.get(i).side2, velocityObstacles.get(j).side2);

				if (d != 0.0)
				{
					double s = AVOMath.det(velocityObstacles.get(j).apex.subtract( velocityObstacles.get(i).apex ), velocityObstacles.get(j).side2) / d;
					double t = AVOMath.det(velocityObstacles.get(j).apex.subtract( velocityObstacles.get(i).apex ), velocityObstacles.get(i).side2) / d;

					if (s >= 0.0 && t >= 0.0)
					{
						Candidate candidate = new Candidate();
						candidate.velocityObstacle1 = i;
						candidate.velocityObstacle2= j;
						candidate.position = velocityObstacles.get(i).apex.add( velocityObstacles.get(i).side2.multiply(s) );

						if (AVOMath.absSq(candidate.position) < maxSpeed * maxSpeed)
						{
							candidates.put(AVOMath.absSq(prefVelocity.subtract(candidate.position)), candidate);
						}
					}
				}
			}
		}

		//candidates.clear();
		
		
		int optimal = -1;

		outer:
		for (Double key: candidates.keySet())
		{
			Iterator<Candidate> candIt= candidates.get(key).iterator();
			while(candIt.hasNext())
			{
				Candidate candidate = candIt.next();
				boolean valid = true;
				
				for (int j = 0; j < velocityObstacles.size(); ++j)
				{
					if (j != candidate.velocityObstacle1 &&
						j != candidate.velocityObstacle2 &&
						AVOMath.det(velocityObstacles.get(j).side2, candidate.position.subtract(velocityObstacles.get(j).apex) ) < 0.0 &&
						AVOMath.det(velocityObstacles.get(j).side1, candidate.position.subtract(velocityObstacles.get(j).apex) ) > 0.0)
					{
						valid = false;
	
						if (j > optimal)
						{
							optimal = j;
							newVelocity.setTo(candidate.position);
						}
	
						break;
					}
				}
	
				if (valid)
				{
					newVelocity.setTo(candidate.position);
					break outer;
				}
			}
		}
	 }

	/* Search for the best new velocity according to ORCA */
	 protected void computeORCANewVelocity()
	 {
	     orcaLines.clear();
	    
	     int numObstLines = orcaLines.size();
	
	     double invTimeHorizon = 1.0 / timeHorizon;
	
	     /* Create agent ORCA lines. */
	     for (int i = 0; i < agentNeighbors.size(); ++i)
	     {
	         Agent other = agentNeighbors.get(i).getValue();
	
	         Double2D relativePosition = other.position.subtract( position);
	         Double2D relativeVelocity = velocity.subtract(other.velocity);
	         double distSq = AVOMath.absSq(relativePosition);
	         double combinedRadius = radius + other.radius;
	         double combinedRadiusSq = AVOMath.sqr(combinedRadius);
	
	         Line line = new Line();
	         Double2D u;
	
	         if (distSq > combinedRadiusSq)
	         {
	             /* No collision. */
	             Double2D w = relativeVelocity.subtract(relativePosition.multiply(invTimeHorizon));
	             /* Vector from cutoff center to relative velocity. */
	             double wLengthSq = AVOMath.absSq(w);
	
	             double dotProduct1 = w.dot(relativePosition);
	
	             if (dotProduct1 < 0.0 && AVOMath.sqr(dotProduct1) > combinedRadiusSq * wLengthSq)
	             {
	                 /* Project on cut-off circle. */
	                 double wLength = AVOMath.sqrt(wLengthSq);
	                 Double2D unitW = w.multiply(1.0 / wLength);
	
	                 line.direction = new Double2D(unitW.y, -unitW.x);
	                 u = unitW.multiply(combinedRadius * invTimeHorizon - wLength);
	             }
	             else
	             {
	                 /* Project on legs. */
	                 double leg = AVOMath.sqrt(distSq - combinedRadiusSq);
	
	                 if (AVOMath.det(relativePosition, w) > 0.0)
	                 {
	                     /* Project on left leg. */
	                     line.direction = new Double2D(relativePosition.x * leg - relativePosition.y * combinedRadius
	                    		 						, relativePosition.x * combinedRadius + relativePosition.y * leg).multiply(
	                    		 								1.0/ distSq);
	                 }
	                 else
	                 {
	                     /* Project on right leg. */
	                     line.direction = new Double2D(relativePosition.x * leg + relativePosition.y* combinedRadius
	                    		 						, -relativePosition.x * combinedRadius + relativePosition.y * leg).negate().multiply(
	                    		 								1.0/ distSq);
	                 }
	
	                 double dotProduct2 = relativeVelocity.dot(line.direction);
	
	                 u = line.direction.multiply(dotProduct2).subtract( relativeVelocity );
	             }
	         }
	         else
	         {
	             /* Collision. Project on cut-off circle of time timeStep. */
	             double invTimeStep = 1.0 / sim.timeStep;
	
	             /* Vector from cutoff center to relative velocity. */
	             Double2D w = relativeVelocity.subtract(relativePosition.multiply(invTimeStep) );
	
	             double wLength = AVOMath.abs(w);
	             Double2D unitW = w.multiply(1.0/ wLength);
	
	             line.direction = new Double2D(unitW.y, -unitW.x);
	             u = unitW.multiply(combinedRadius * invTimeStep - wLength);
	         }
	
	         line.point = velocity.add( u.multiply(0.5) );
	         orcaLines.add(line);
	     }
	
	     int lineFail = AVOMath.linearProgram2(orcaLines, maxSpeed, prefVelocity, false, newVelocity);
	
	     if (lineFail < orcaLines.size())
	     {
	    	 AVOMath.linearProgram3(orcaLines, numObstLines, lineFail, maxSpeed, newVelocity);
	     }
	 }
	 
	 
	 
		/* Search for the best new velocity according to SVO */
	 protected int computeSVONewVelocity()
	 {
		velocityObstacles.clear();
		velocityObstacles.ensureCapacity(agentNeighbors.size());
			
		int[] avoid = new int[agentNeighbors.size()];
		int[] maintain = new int[agentNeighbors.size()];
		int[] alert = new int[agentNeighbors.size()];
		double D_avo = 3889*0.29;
		double V_cat1 =100*0.29;
		
		for (int i=0; i< agentNeighbors.size(); ++i)
		{
			Agent other = agentNeighbors.get(i).getValue();

			VelocityObstacle velocityObstacle = new VelocityObstacle();
			/* check for collision*/

			double angle = AVOMath.atan(other.position.subtract(position) );
			double openingAngle = Math.asin((other.radius + radius) / AVOMath.abs(other.position.subtract(position)));
			
			velocityObstacle.side1= new Double2D(Math.cos(angle - openingAngle), Math.sin(angle - openingAngle));
			velocityObstacle.side2= new Double2D(Math.cos(angle + openingAngle), Math.sin(angle + openingAngle));
			velocityObstacle.apex = other.velocity;
			velocityObstacles.add(velocityObstacle);
			
			Double2D c_vo = velocity;
			Double2D c_vi = other.velocity;
			double turnAngle = c_vo.rotateAngleToDouble2D(c_vi);
			if(turnAngle<0)
			{
				turnAngle += 2*Math.PI;
			}
			
			double theta = c_vo.angleWithDouble2D(c_vi);
			
//			System.out.println(Math.toDegrees(turnAngle)+ "   "+ Math.toDegrees(theta));
			Double2D c_pi = other.position.subtract(position).add(c_vi);
			VelocityObstacle VO_oi = velocityObstacle;
			
			VelocityObstacle VO_div = new VelocityObstacle(); 
//			VO_div.side1= VO_oi.side1.negate();
//			VO_div.side2= VO_oi.side2.negate();
			VO_div.side1= other.velocity;
			VO_div.side2= other.velocity.negate();
			VO_div.apex= VO_oi.apex;
				
//			if (AVOMath.inCircle(new Double2D(), D_avo, c_pi))
//			{		
//				if (AVOMath.inCircle(new Double2D(), V_cat1, c_vi)&&
//						!AVOMath.inVelocityObstacle(VO_div, c_vo))
//				{
//					if(AVOMath.inVelocityObstacle(VO_oi, c_vo)&&
//							((turnAngle>45)&&(turnAngle<225)&&(c_vi.length()<V_cat1)||
//									(c_vi.length()<c_vo.length())&&(theta>=0)&&(theta<=45)))
//					{
//						avoid[i]=1;
//						maintain[i]=0;
//						alert[i]=0;
//					}
//					else
//					{
//						avoid[i]=0;
//						maintain[i]=1;
//						alert[i]=0;
//					}
//				}
//				else
//				{
//					avoid[i]=0;
//					maintain[i]=0;
//					alert[i]=1;
//				}
//				
//				
//			}
//			else
//			{
//				avoid[i]=0;
//				maintain[i]=0;
//				alert[i]=0;
//			}
//			System.out.println(AVOMath.inVelocityObstacle(VO_oi, c_vo));
//			System.out.println((turnAngle>0.25*Math.PI)&&(turnAngle<1.25*Math.PI)&&(c_vi.length()<V_cat1));
//			System.out.println((c_vi.length()<c_vo.length())&&(theta>=0)&&(theta<=0.25*Math.PI));
//			System.out.println("----------------------------"); 

			//!(AVOMath.leftOf(new Double2D(), c_vi, c_pi)^AVOMath.leftOf(new Double2D(), c_vi, c_vo))
			//AVOMath.inCircle(c_pi, other.position.subtract(position).length(), c_vo)		

			if (AVOMath.inCircle(new Double2D(), D_avo, c_pi) &&
					AVOMath.inCircle(new Double2D(), V_cat1, c_vi)&& 
					AVOMath.inCircle(c_pi, other.position.subtract(position).length(), c_vo))					
			{
				if(AVOMath.inVelocityObstacle(VO_oi, c_vo)&&
						((turnAngle>0.25*Math.PI)&&(turnAngle<1.25*Math.PI)&&(c_vi.length()<V_cat1)||
								(c_vi.length()<c_vo.length())&&(theta>=0)&&(theta<=0.25*Math.PI)))
				{
					avoid[i]=1;
					maintain[i]=0;					
				}
				else
				{
				
					avoid[i]=0;
					maintain[i]=1;					
				}
			}
			else
			{
				avoid[i]=0;
				maintain[i]=0;	
			}
			
		}
		
		
		int AVOID=0;
		int MAINTAIN=0;
		int ALERT=0;
		for(int i=0; i<avoid.length; i++)
		{
			AVOID += avoid[i];
			MAINTAIN += maintain[i];
			ALERT += alert[i];
		}
		
		int mode;
		if(AVOID>0)
		{
			 mode=1;//avoid
			 
			 
			 double min_penalty = Double.POSITIVE_INFINITY;
		     Double2D vCand;
		     
		     MersenneTwisterFast rand = sim.rand;
		     int sampleCount=1;
		     double penaltyCoef =100;
		    	     		 
		     // Select sampleCount candidate velocities
			 for (int n = 0; n < sampleCount; ++n)			
			 {			
				 vCand = velocity.add(velocity.normalize().multiply(maxAccel*(2*rand.nextDouble()-1)));	
				 if(vCand.length()>maxSpeed)
				 {
					 vCand = vCand.normalize().multiply(maxSpeed);
				 }
				 
				 if(vCand.length()<minSpeed)
				 {
					 vCand = vCand.normalize().multiply(minSpeed);
				 }
					
//				 vCand = vCand.rotate(-maxTurn*rand.nextDouble());
				 vCand = vCand.rotate(-maxTurn);

			     double dV; // distance between candidate velocity and preferred velocity
			     dV = vCand.distance(prefVelocity);		     

			     // searching for smallest time to collision
			     double ct = Double.POSITIVE_INFINITY; // time to collision		      
				 for (int i=0; i< agentNeighbors.size(); ++i)
				 { 
					 Agent agenti = agentNeighbors.get(i).getValue();
			
				     double ct_i; // time to collision with agent i
				       
				     Double2D relV;		        
				     relV = vCand.subtract(agenti.velocity);
				     ct_i = AVOMath.timeToCollision(position, relV, agenti.position, radius + agenti.radius, false);
				       			        	        
				     if (ct_i < ct)
				     {
			        	ct = ct_i;
				        // pruning search if no better penalty can be obtained anymore for this velocity
				        if ( penaltyCoef / ct + dV >= min_penalty)
				        {
				            break;
				        }
				     }
			      }

				 
			      double penalty = penaltyCoef / ct + dV;
			      if (penalty < min_penalty)
			      {
			    	  min_penalty = penalty;
			    	  newVelocity.setTo(vCand);
			      }
			   		      
			 }					
			
		}
		else if(MAINTAIN>0)
		{
			mode=2;//maintain
		}
		else if (ALERT>0)
		{
			mode=3;//alert
		}
		else
		{
			mode =4;//restore
		}

		return mode;
	 }

    
}

class MutableDouble
{
	public double doubleValue;
	
	public MutableDouble()
	{
		this.doubleValue = 0.0;
	}
	
	public MutableDouble(double value)
	{
		this.doubleValue = value;
	}
	
	public MutableDouble(Double value)
	{
		this.doubleValue = value;
	}
}

