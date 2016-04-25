package avo;

import java.util.ArrayList;

import ec.util.MersenneTwisterFast;
import sim.engine.SimState;
import sim.util.Double2D;

public class AVOSimulator 
{
	private Agent defaultAgent;
    protected double timeStep;
	protected double time;
	private boolean allReachedGoals;
    

	protected ArrayList<Agent> agents;
	public ArrayList<Goal> goals = new ArrayList<Goal>();
    protected KdTree kdTree;
    
    
    protected MersenneTwisterFast rand;

    public AVOSimulator()
    { 
    	this.rand = new MersenneTwisterFast(System.currentTimeMillis());
    	agents = new ArrayList<Agent>();
        time = 0;
        defaultAgent = null;
        kdTree = new KdTree(this);
        timeStep = 0.1;
        allReachedGoals=false;
	}
    
    public AVOSimulator(SimState state)
    { 
    	this.rand = state.random;
    	agents = new ArrayList<Agent>();
        time = 0;
        defaultAgent = null;
        kdTree = new KdTree(this);
        timeStep = 0.1;
        allReachedGoals=false;
	}

   

    /**
	 * \brief      Sets the default properties for any new agent that is
	 *             added.
	 * \param      neighborDist    The default maximum distance (center point
	 *                             to center point) to other agents a new agent
	 *                             takes into account in the navigation. The
	 *                             larger this number, the longer he running
	 *                             time of the simulation. If the number is too
	 *                             low, the simulation will not be safe.
	 *                             Must be non-negative.
	 * \param      maxNeighbors    The default maximum number of other agents a
	 *                             new agent takes into account in the
	 *                             navigation. The larger this number, the
	 *                             longer the running time of the simulation.
	 *                             If the number is too low, the simulation
	 *                             will not be safe.
	 * \param      timeHorizon     The default minimal amount of time for which
	 *                             a new agent's velocities that are computed
	 *                             by the simulation are safe with respect to
	 *                             other agents. The larger this number, the
	 *                             sooner an agent will respond to the presence
	 *                             of other agents, but the less freedom the
	 *                             agent has in choosing its velocities.
	 *                             Must be positive.
	 * \param      timeHorizonObst The default minimal amount of time for which
	 *                             a new agent's velocities that are computed
	 *                             by the simulation are safe with respect to
	 *                             obstacles. The larger this number, the
	 *                             sooner an agent will respond to the presence
	 *                             of obstacles, but the less freedom the agent
	 *                             has in choosing its velocities.
	 *                             Must be positive.
	 * \param      radius          The default radius of a new agent.
	 *                             Must be non-negative.
	 * \param      goalRadius      The default goal radius of a new agent.
	 *                             Must be non-negative.                             
	 * \param      prefSpeed       The default preferred speed of a new agent.
	 *                             Must be non-negative.
	 * \param      maxSpeed        The default maximum speed of a new agent.
	 *                             Must be non-negative.   
	 * \param      minSpeed        The default minimum speed of a new agent.
	 *                             Must be non-negative. 
	 * \param      alpha           The default alpha of a new agent. 
	 * \param      maxAccel        The default maximum acceleration of a new agent (optional).   
	 * \param      maxTurn         The default maximum turn rate of a new agent (optional).                           
	 * \param      velocity        The default initial two-dimensional linear velocity of a new agent (optional).
	 */
    public void setAgentDefaults(double neighborDist, int maxNeighbors, double timeHorizon, double timeHorizonObst, double radius, double goalRadius, double prefSpeed, double maxSpeed, double minSpeed, double alpha, double maxAccel,double maxTurn, Double2D velocity)
	{
	    if (defaultAgent == null)
	    {
	        defaultAgent = new Agent(this);
	    }
	
	    defaultAgent.neighborDist = neighborDist;
	    defaultAgent.maxNeighbors = maxNeighbors;
	    defaultAgent.timeHorizon = timeHorizon;
	    defaultAgent.timeHorizonObst = timeHorizonObst;
	    defaultAgent.radius = radius;
	    defaultAgent.goalRadius = goalRadius;
	    defaultAgent.prefSpeed = prefSpeed;
	    defaultAgent.maxSpeed = maxSpeed;
	    defaultAgent.minSpeed = minSpeed;
	    defaultAgent.maxTurn = maxTurn;
	    defaultAgent.alpha = alpha;
	    
	    defaultAgent.maxAccel = maxAccel;
	    defaultAgent.velocity = velocity;
	}
    
    
    /**
	 * \brief      Sets the default properties for any new agent that is
	 *             added.
	 * \param      neighborDist    The default maximum distance (center point
	 *                             to center point) to other agents a new agent
	 *                             takes into account in the navigation. The
	 *                             larger this number, the longer he running
	 *                             time of the simulation. If the number is too
	 *                             low, the simulation will not be safe.
	 *                             Must be non-negative.
	 * \param      maxNeighbors    The default maximum number of other agents a
	 *                             new agent takes into account in the
	 *                             navigation. The larger this number, the
	 *                             longer the running time of the simulation.
	 *                             If the number is too low, the simulation
	 *                             will not be safe.
	 * \param      timeHorizon     The default minimal amount of time for which
	 *                             a new agent's velocities that are computed
	 *                             by the simulation are safe with respect to
	 *                             other agents. The larger this number, the
	 *                             sooner an agent will respond to the presence
	 *                             of other agents, but the less freedom the
	 *                             agent has in choosing its velocities.
	 *                             Must be positive.
	 * \param      timeHorizonObst The default minimal amount of time for which
	 *                             a new agent's velocities that are computed
	 *                             by the simulation are safe with respect to
	 *                             obstacles. The larger this number, the
	 *                             sooner an agent will respond to the presence
	 *                             of obstacles, but the less freedom the agent
	 *                             has in choosing its velocities.
	 *                             Must be positive.
	 * \param      radius          The default radius of a new agent.
	 *                             Must be non-negative.
	 * \param      goalRadius      The default goal radius of a new agent.
	 *                             Must be non-negative.                             
	 * \param      prefSpeed       The default preferred speed of a new agent.
	 *                             Must be non-negative.
	 * \param      maxSpeed        The default maximum speed of a new agent.
	 *                             Must be non-negative.  
	 * \param      minSpeed        The default minimum speed of a new agent.
	 *                             Must be non-negative. 
	 * \param      alpha           The default alpha of a new agent. 
	 */
    public void setAgentDefaults(double neighborDist, int maxNeighbors, double timeHorizon, double timeHorizonObst, double radius, double goalRadius, double prefSpeed, double maxSpeed, double minSpeed,double alpha)
	{
	    if (defaultAgent == null)
	    {
	        defaultAgent = new Agent(this);
	    }
	
	    defaultAgent.neighborDist = neighborDist;
	    defaultAgent.maxNeighbors = maxNeighbors;
	    defaultAgent.timeHorizon = timeHorizon;
	    defaultAgent.timeHorizonObst = timeHorizonObst;
	    defaultAgent.radius = radius;
	    defaultAgent.goalRadius = goalRadius;
	    defaultAgent.prefSpeed = prefSpeed;
	    defaultAgent.maxSpeed = maxSpeed;
	    defaultAgent.minSpeed = minSpeed;
	    defaultAgent.alpha = alpha;
	    
	    defaultAgent.maxAccel = Double.POSITIVE_INFINITY;
	    defaultAgent.maxTurn = Double.POSITIVE_INFINITY;
	    defaultAgent.velocity = new Double2D();
	}

	public int addAgent(Double2D position, int goalID)
	{
	    if (defaultAgent == null)
	    {
	    	System.out.println("defaultAgent has not been set!");
	        return -1;
	    }
	
	    Agent agent = new Agent(this);
	
	    agent.id = agents.size();
	    agent.position = position;
	    agent.goalID = goalID;
	    agent.neighborDist = defaultAgent.neighborDist;
	    agent.maxNeighbors = defaultAgent.maxNeighbors;
	    agent.timeHorizon = defaultAgent.timeHorizon;
	    agent.timeHorizonObst = defaultAgent.timeHorizonObst;
	    agent.radius = defaultAgent.radius;
	    agent.goalRadius = defaultAgent.goalRadius;
	    agent.prefSpeed = defaultAgent.prefSpeed;
	    agent.maxSpeed = defaultAgent.maxSpeed;  
	    agent.minSpeed = defaultAgent.minSpeed; 
	    agent.alpha = defaultAgent.alpha;
	    
	    agent.maxAccel = defaultAgent.maxAccel;
	    agent.maxTurn = defaultAgent.maxTurn;
	    agent.velocity = defaultAgent.velocity;    
	
	    agents.add(agent);
	
	    return agent.id;
	
	}
	
	public int addAgent(Double2D position, int goalID, double neighborDist, int maxNeighbors, 
			double timeHorizon, double timeHorizonObst, double radius, double goalRadius, double prefSpeed, 
			double maxSpeed, double minSpeed,double alpha, double maxAccel,double maxTurn,Double2D velocity)
	{
		Agent agent = new Agent(this);
		
		agent.id = agents.size();
	    agent.position = position;
	    agent.goalID = goalID;
	    agent.neighborDist = neighborDist;
	    agent.maxNeighbors = maxNeighbors;
	    agent.timeHorizon = timeHorizon;
	    agent.timeHorizonObst = timeHorizonObst;
	    agent.radius = radius;
	    agent.goalRadius = goalRadius;
	    agent.prefSpeed = prefSpeed;
	    agent.maxSpeed = maxSpeed;  
	    agent.minSpeed = minSpeed;
	    agent.alpha = alpha;
	    
	    agent.maxAccel = maxAccel;
	    agent.maxAccel = maxTurn;
	    agent.velocity = velocity;    
	
	    agents.add(agent);
	
	    return agent.id;
	}
	
	public int addAgent(Double2D position, int goalID, double neighborDist, int maxNeighbors, 
			double timeHorizon, double timeHorizonObst, double radius, double goalRadius, double prefSpeed, 
			double maxSpeed, double minSpeed, double alpha)
	{
		Agent agent = new Agent(this);
		
		agent.id = agents.size();
	    agent.position = position;
	    agent.goalID = goalID;
	    agent.neighborDist = neighborDist;
	    agent.maxNeighbors = maxNeighbors;
	    agent.timeHorizon = timeHorizon;
	    agent.timeHorizonObst = timeHorizonObst;
	    agent.radius = radius;
	    agent.goalRadius = goalRadius;
	    agent.prefSpeed = prefSpeed;
	    agent.maxSpeed = maxSpeed;  
	    agent.minSpeed = minSpeed;
	    agent.alpha = alpha;
	    
	    agent.maxAccel = defaultAgent.maxAccel;
	    agent.maxTurn = defaultAgent.maxTurn;
	    agent.velocity = defaultAgent.velocity;     
	
	    agents.add(agent);
	
	    return agent.id;
	}
	
	/**
	 * \brief      Adds a new goal to the simulation.
	 * \param[in]  position  The position of this goal.
	 * \return     The ID of the goal.
	 */
	public int addGoal(Double2D position)
	{
		Goal goal = new Goal(position);
		goal.goalID = goals.size();
		goals.add(goal);
		
		return goal.goalID ;
	}
	

	public double doStep()
	{
		if (kdTree == null) 
		{
			System.err.println("Simulation not initialized when attempting to do step.");
		}

		if (timeStep == 0.0) 
		{
			System.err.println("Time step not set when attempting to do step.");
		}

		allReachedGoals = true;
		
	    kdTree.buildAgentTree();
	    for (int i = 0; i < getNumAgents(); ++i)
	    {
	    	agents.get(i).computePreferredVelocity();
	        agents.get(i).computeNeighbors();
	        agents.get(i).computeNewVelocity();
	    }
	    	
	    for (int i = 0; i < getNumAgents(); ++i)
	    {
	    	agents.get(i).update();
	    }
	
	    time += timeStep;
	    return time;
	}
	
	public int agentDoStep(int agentID)
	{
		if (kdTree == null) 
		{
			System.err.println("Simulation not initialized when attempting to do step.");
		}

		if (timeStep == 0.0) 
		{
			System.err.println("Time step not set when attempting to do step.");
		}
	    kdTree.buildAgentTree();
     	agents.get(agentID).computePreferredVelocity();
        agents.get(agentID).computeNeighbors();
        int mode= agents.get(agentID).computeNewVelocity(); 
    	agents.get(agentID).update();
    	return mode;
	  
	}

	public double getGlobalTime() { return time; }
    public int getNumAgents() { return agents.size(); }
    public double getTimeStep() { return timeStep; }

    public void setTimeStep(double timeStep)
    {
        this.timeStep = timeStep;
    }
    public Double2D getAgentPosition(int i)
    {
        return agents.get(i).position;
    }
    public void setAgentPosition(int i, Double2D position)
    {
        agents.get(i).position = position;
    }
    
    public Double2D getAgentPrefVelocity(int i)
    {
        return agents.get(i).prefVelocity;
    }
    public void setAgentPrefVelocity(int i, Double2D velocity)
	{
	    agents.get(i).prefVelocity = velocity;
	}

	public Double2D getAgentVelocity(int i)
    {
        return agents.get(i).velocity;
    }
    public void setAgentVelocity(int i, Double2D velocity)
    {
    	 agents.get(i).velocity = velocity;
    }
    
    public double getAgentRadius(int i)
    {
        return agents.get(i).radius;
    }
    
    public void setAgentRadius(int i, double radius)
    {
        agents.get(i).radius = radius;
    }
    
    
    public double getAgentAlpha(int i)
    {
        return agents.get(i).alpha;
    }
    public void setAgentAlpha(int i, double alpha)
    {
        agents.get(i).alpha = alpha;
    }
    
    public boolean haveAllReachedGoals()
	{ 
		return allReachedGoals; 
		
	}
    
	public void setAllReachedGoals(boolean allReachedGoals) 
	{
		this.allReachedGoals = allReachedGoals;
	}
	
    public ArrayList<Line> getAgentOrcaLines(int i)
    {
        return agents.get(i).orcaLines;
    }
    
    public ArrayList<VelocityObstacle> getAgentVelocityObstacles(int i)
    {
        return agents.get(i).velocityObstacles;
    }

}
