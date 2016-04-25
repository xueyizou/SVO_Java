package avo;

import sim.util.Double2D;

public class Test {

	public Test() {
		// TODO Auto-generated constructor stub
	}

	/**
	 * @param args
	 */
	public static void main(String[] args)
	{
		Double2D v1= new Double2D(1,1);
		Double2D v2= new Double2D(-1,1);
		Double2D v3= new Double2D(-1,-1);
		Double2D v4= new Double2D(1,-1);
		
		Double2D v5= new Double2D(1,0);
		Double2D v6= new Double2D(0,1);
		Double2D v7= new Double2D(-1,0);
		Double2D v8= new Double2D(0,-1);
		
		Double2D v= v2;
	
//		System.out.println(Math.toDegrees(v7.angle()));
//		System.out.println(v3.rotate(-Math.PI/4));
//		System.out.println(Math.toDegrees(v3.rotate(-Math.PI/4).angle()));
		
//		Double2D[22.388000000000034,-0.0]Double2D[0.6718572305274516,0.7406806746419017]Double2D[-0.9521857666739805,0.3055196650683619]Double2D[19.388576739926012,11.19399999999999]
		
		VelocityObstacle VO_div = new VelocityObstacle(); 
		VO_div.side1= new Double2D(0.6718572305274516,0.7406806746419017);
		VO_div.side2= new Double2D(-0.9521857666739805,0.3055196650683619);
		VO_div.apex= new Double2D(22.388000000000034,-0.0);
		Double2D c_vo = new Double2D(19.388576739926012,11.19399999999999);
		
//		VelocityObstacle VO_div = new VelocityObstacle(); 
//		VO_div.side1= new Double2D(1,-1);
//		VO_div.side2= new Double2D(1,1);
//		VO_div.apex= new Double2D(0,0);
//		Double2D c_vo = new Double2D(0,-2);
	 
	 System.out.println(AVOMath.leftOf(VO_div.apex, VO_div.apex.add(VO_div.side1), c_vo));
	 System.out.println(AVOMath.rightOf(VO_div.apex,VO_div.apex.add(VO_div.side2), c_vo));
		
		System.out.println(AVOMath.inVelocityObstacle(VO_div, c_vo));

	}

}
