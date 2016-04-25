package avo;


public class KdTree
{

     private final int MAX_LEAF_SIZE = 10;
     private AVOSimulator sim;
     
     private Agent[] kdTreeAgents;
     private AgentTreeNode[] agentTree;


     private class AgentTreeNode
     {
         protected int begin;
         protected int end;
         protected int left;
         protected int right;
         protected double maxX;
         protected double maxY;
         protected double minX;
         protected double minY;
         
     }
    

     public KdTree(AVOSimulator orcaSimulator)
     {
         sim = orcaSimulator;
     }

     protected void buildAgentTree()
     {
         if (kdTreeAgents==null || kdTreeAgents.length != sim.agents.size())
         {
             kdTreeAgents = new Agent[sim.agents.size()];
             for (int i = 0; i < kdTreeAgents.length; ++i)
             {
                 kdTreeAgents[i] = sim.agents.get(i);
             }

             agentTree = new AgentTreeNode[2 * kdTreeAgents.length];
             for (int i = 0; i < agentTree.length; ++i)
             {
                 agentTree[i] = new AgentTreeNode();
             }
         }

         if (kdTreeAgents.length != 0)
         {
             buildAgentTreeRecursive(0, kdTreeAgents.length, 0);
         }
     }

     void buildAgentTreeRecursive(int begin, int end, int node)
     {
    	 try
    	 {
    		 agentTree[node].begin = begin;
    	 }
    	 catch(Exception e)
    	 {
    		 System.out.print(node);
    	 }
         
         agentTree[node].end = end;
         agentTree[node].minX = agentTree[node].maxX = kdTreeAgents[begin].position.x;
         agentTree[node].minY = agentTree[node].maxY = kdTreeAgents[begin].position.y;

         for (int i = begin + 1; i < end; ++i)
         {
             agentTree[node].maxX = Math.max(agentTree[node].maxX, kdTreeAgents[i].position.x);
             agentTree[node].minX = Math.min(agentTree[node].minX, kdTreeAgents[i].position.x);
             agentTree[node].maxY = Math.max(agentTree[node].maxY, kdTreeAgents[i].position.y);
             agentTree[node].minY = Math.min(agentTree[node].minY, kdTreeAgents[i].position.y);
         }

         if (end - begin > MAX_LEAF_SIZE)
         {
             /* No leaf node. */
             boolean isVertical = (agentTree[node].maxX - agentTree[node].minX > agentTree[node].maxY - agentTree[node].minY);
             double splitValue = (isVertical ? 0.5 * (agentTree[node].maxX + agentTree[node].minX) : 0.5 * (agentTree[node].maxY + agentTree[node].minY));

             int left = begin;
             int right = end;

             while (left < right)
             {
                 while (left < right && (isVertical ? kdTreeAgents[left].position.x: kdTreeAgents[left].position.y) < splitValue)
                 {
                     ++left;
                 }

                 while (right > left && (isVertical ? kdTreeAgents[right - 1].position.x: kdTreeAgents[right - 1].position.y) >= splitValue)
                 {
                     --right;
                 }

                 if (left < right)
                 {
                     Agent tmp = kdTreeAgents[left];
                     kdTreeAgents[left] = kdTreeAgents[right - 1];
                     kdTreeAgents[right - 1] = tmp;
                     ++left;
                     --right;
                 }
             }

             int leftSize = left - begin;

             if (leftSize == 0)
             {
                 ++leftSize;
                 ++left;
                 ++right;
             }

             agentTree[node].left = node + 1;
             agentTree[node].right = node + 1 + (2 * leftSize - 1);

             buildAgentTreeRecursive(begin, left, agentTree[node].left);
             buildAgentTreeRecursive(left, end, agentTree[node].right);
         }
     }

     protected void queryAgentNeighbors(Agent agent, MutableDouble rangeSq)
     {
         queryAgentTreeRecursive(agent,rangeSq, 0);
     }


     void queryAgentTreeRecursive(Agent agent, MutableDouble rangeSq, int node)
     {
         if (agentTree[node].end - agentTree[node].begin <= MAX_LEAF_SIZE)
         {
             for (int i = agentTree[node].begin; i < agentTree[node].end; ++i)
             {
                 agent.insertAgentNeighbor(kdTreeAgents[i], rangeSq);
             }
         }
         else
         {
             double distSqLeft = AVOMath.sqr(Math.max(0.0f, agentTree[agentTree[node].left].minX - agent.position.x)) + AVOMath.sqr(Math.max(0.0f, agent.position.x- agentTree[agentTree[node].left].maxX)) + AVOMath.sqr(Math.max(0.0f, agentTree[agentTree[node].left].minY - agent.position.y)) + AVOMath.sqr(Math.max(0.0f, agent.position.y- agentTree[agentTree[node].left].maxY));

             double distSqRight = AVOMath.sqr(Math.max(0.0f, agentTree[agentTree[node].right].minX - agent.position.x)) + AVOMath.sqr(Math.max(0.0f, agent.position.x- agentTree[agentTree[node].right].maxX)) + AVOMath.sqr(Math.max(0.0f, agentTree[agentTree[node].right].minY - agent.position.y)) + AVOMath.sqr(Math.max(0.0f, agent.position.y- agentTree[agentTree[node].right].maxY));

             if (distSqLeft < distSqRight)
             {
                 if (distSqLeft < rangeSq.doubleValue)
                 {
                     queryAgentTreeRecursive(agent, rangeSq, agentTree[node].left);

                     if (distSqRight < rangeSq.doubleValue)
                     {
                         queryAgentTreeRecursive(agent, rangeSq, agentTree[node].right);
                     }
                 }
             }
             else
             {
                 if (distSqRight < rangeSq.doubleValue)
                 {
                     queryAgentTreeRecursive(agent, rangeSq, agentTree[node].right);

                     if (distSqLeft < rangeSq.doubleValue)
                     {
                         queryAgentTreeRecursive(agent, rangeSq, agentTree[node].left);
                     }
                 }
             }

         }
     }
 
}
