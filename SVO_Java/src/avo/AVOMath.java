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

import java.util.ArrayList;

import sim.util.Double2D;
import sim.util.MutableDouble2D;

public class AVOMath 
{

	private AVOMath() 
	{
		
	}
	
	protected static final double AVO_EPSILON = 1.0e-6;

	protected static double sqr(double p)
	{
	    return p * p;
	}

	protected static double sqrt(double a)
    {
        return Math.sqrt(a);
    }
    
    protected static double fabs(double a)
    {
        return Math.abs(a);
    }

    public static double absSq(Double2D v)
	{
	    return v.lengthSq();
	}

	protected static double abs(Double2D v)
	{
	    return (double)Math.sqrt(absSq(v));
	}

	public static Double2D normalize(Double2D v)
	{
	    return v.normalize();
	}
	
	  /*! \param q A vector
    \returns The angle the vector makes with the positive x-axis. Is in the range [-PI, PI]. */
	public static double atan(Double2D v)
	{
		  return Math.atan2(v.y, v.x);
	}
	
	 /*! \param p A point
    \param q A point
    \returns The normal vector to the line segment pq. */
	protected static Double2D normal(Double2D p, Double2D q)
	{
		  return normalize(new Double2D(q.y - p.y, -(q.x - p.x)));
	}

	protected static double det(Double2D v1, Double2D v2)
    {
        return v1.x * v2.y - v1.y * v2.x;
    }
	
    
    protected static double distSqPointLineSegment(Double2D a, Double2D b, Double2D c)
	{
	    double r = c.subtract(a).dot( b.subtract(a)) / absSq(b.subtract(a));
	
	    if (r < 0.0)
	    {
	        return absSq(c.subtract(a));
	    }
	    else if (r > 1.0)
	    {
	        return absSq(c.subtract(b));
	    }
	    else
	    {
	        return absSq(c.subtract(a.add(b.subtract(a).multiply(r)))) ;
	    }
	}
    
    /*
     * a first point of a line
     * b second point of a line
     * c testing point
     */
	public static boolean leftOf(Double2D a, Double2D b, Double2D c)
    {
		
		  double acx, bcx, acy, bcy;
	
		  acx = a.x - c.x;
		  bcx = b.x - c.x;
		  acy = a.y - c.y;
		  bcy = b.y - c.y;
		  return (acx * bcy - acy * bcx>0);	
    }
	
	public static boolean rightOf(Double2D a, Double2D b, Double2D c)
    {
		  double acx, bcx, acy, bcy;
			
		  acx = a.x - c.x;
		  bcx = b.x - c.x;
		  acy = a.y - c.y;
		  bcy = b.y - c.y;
		  return (acx * bcy - acy * bcx<0);	
    }
	
	public static boolean inCircle(Double2D o, double r, Double2D c)
    {
        return (c.distanceSq(o)<=r*r);
    }
	
	public static boolean inVelocityObstacle(VelocityObstacle vo, Double2D c)
	{
		return (!rightOf(vo.apex, vo.apex.add(vo.side1), c)&& !leftOf(vo.apex, vo.apex.add(vo.side2), c));
	}
	
	public static boolean inSector(Sector s, Double2D c)
	{
		return (!rightOf(s.apex, s.apex.add(s.side1), c)&& !leftOf(s.apex, s.apex.add(s.side2), c)&& inCircle(s.apex,s.r,c));
	}
	
	

	/* Time to collision of a ray to a disc.
    \param p The start position of the ray
    \param v The velocity vector of the ray
    \param p2 The center position of the disc
    \param radius The radius of the disc
    \param collision Specifies whether the time to collision is computed (false), or the time from collision (true).
    \returns Returns the time to collision of ray p + tv to disc D(p2, radius), and #RVO_INFTY when the disc is not hit. If collision is true, the value is negative.
   */
	protected static double timeToCollision(Double2D p, Double2D v, Double2D p2, double radius, boolean collision) 
	{
		 Double2D ba = p2.subtract(p);
		 double sq_diam = radius * radius;
		 double time;

		 double discr = -sqr(det(v, ba)) + sq_diam * absSq(v);
		 if (discr > 0)
		 {
		     if (collision)
		     {
		       time = (v.dot(ba) + Math.sqrt(discr)) / absSq(v);
		       if (time < 0) 
		       {
		         time = -Double.POSITIVE_INFINITY;
		       }
		     } 
		     else 
		     {
		       time = (v.dot(ba) - Math.sqrt(discr)) / absSq(v);
		       if (time < 0) 
		       {
		         time = Double.POSITIVE_INFINITY;
		       }
		     }
		  }
		 else
		 {
		     if (collision) 
		     {
		       time = -Double.POSITIVE_INFINITY;
		     } 
		     else 
		     {
		       time = Double.POSITIVE_INFINITY;
		     }
		 }
		 
		 return time;
	}
  
 
    
    protected static boolean linearProgram1(ArrayList<Line> lines, int lineNo, double radius, Double2D optVelocity, boolean directionOpt, MutableDouble2D result)
    {
        double dotProduct = lines.get(lineNo).point.dot( lines.get(lineNo).direction ); 
        double discriminant = AVOMath.sqr(dotProduct) + AVOMath.sqr(radius) - AVOMath.absSq(lines.get(lineNo).point);

        if (discriminant < 0.0f)
        {
            /* Max speed circle fully invalidates line lineNo. */
            return false;
        }

        double sqrtDiscriminant = AVOMath.sqrt(discriminant);
        double tLeft = -dotProduct - sqrtDiscriminant;
        double tRight = -dotProduct + sqrtDiscriminant;

        for (int i = 0; i < lineNo; ++i)
        {
            double denominator = AVOMath.det(lines.get(lineNo).direction, lines.get(i).direction);
            double numerator = AVOMath.det(lines.get(i).direction, lines.get(lineNo).point.subtract( lines.get(i).point) );

            if (AVOMath.fabs(denominator) <= AVOMath.AVO_EPSILON)
            {
                /* Lines lineNo and i are (almost) parallel. */
                if (numerator < 0.0f)
                {
                    return false;
                }
                else
                {
                    continue;
                }
            }

            double t = numerator / denominator;

            if (denominator >= 0.0f)
            {
                /* Line i bounds line lineNo on the right. */
                tRight = Math.min(tRight, t);
            }
            else
            {
                /* Line i bounds line lineNo on the left. */
                tLeft = Math.max(tLeft, t);
            }

            if (tLeft > tRight)
            {
                return false;
            }
        }

        if (directionOpt)
        {
            /* Optimize direction. */
            if (optVelocity.dot(lines.get(lineNo).direction) > 0.0)
            {
                /* Take right extreme. */
                result.setTo(lines.get(lineNo).point.add( lines.get(lineNo).direction.multiply(tRight) ) );
            }
            else
            {
                /* Take left extreme. */
                result.setTo( lines.get(lineNo).point.add( lines.get(lineNo).direction.multiply(tLeft) ) );
            }
        }
        else
        {
            /* Optimize closest point. */
            double t = lines.get(lineNo).direction.dot( optVelocity.subtract(lines.get(lineNo).point) );

            if (t < tLeft)
            {
                result.setTo( lines.get(lineNo).point.add( lines.get(lineNo).direction.multiply(tLeft) ) );
            }
            else if (t > tRight)
            {
                result.setTo( lines.get(lineNo).point.add( lines.get(lineNo).direction.multiply(tRight) ) );
            }
            else
            {
                result.setTo(lines.get(lineNo).point.add( lines.get(lineNo).direction.multiply(t) ) );
            }
        }

        return true;
    }

    protected static int linearProgram2(ArrayList<Line> lines, double radius, Double2D optVelocity, boolean directionOpt, MutableDouble2D result)
    {
        if (directionOpt)
        {
            /*
             * Optimize direction. Note that the optimization velocity is of unit
             * length in this case.
             */
            result.setTo(optVelocity.multiply(radius) );
        }
        else if (AVOMath.absSq(optVelocity) > AVOMath.sqr(radius))
        {
            /* Optimize closest point and outside circle. */
            result.setTo(  AVOMath.normalize(optVelocity).multiply(radius) );
        }
        else
        {
            /* Optimize closest point and inside circle. */
            result.setTo( optVelocity);
        }

        for (int i = 0; i < lines.size(); ++i)
        {
            if (AVOMath.det(lines.get(i).direction, lines.get(i).point.subtract(new Double2D(result) )) > 0.0)
            {
                /* Result does not satisfy constraint i. Compute new optimal result. */
                Double2D tempResult = new Double2D(result);
                if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, result))
                {
                    result.setTo(tempResult);
                    return i;
                }
            }
        }

        return lines.size();
    }

    protected static void linearProgram3(ArrayList<Line> lines, int numObstLines, int beginLine, double radius, MutableDouble2D result)
    {
        double distance = 0.0f;

        for (int i = beginLine; i < lines.size(); ++i)
        {
            if (AVOMath.det(lines.get(i).direction, lines.get(i).point.subtract(new Double2D(result))) > distance)
            {
                /* Result does not satisfy constraint of line i. */
                //std::vector<Line> projLines(lines.begin(), lines.begin() + numObstLines);
                ArrayList<Line> projLines = new ArrayList<Line>();
                for (int ii = 0; ii < numObstLines; ++ii)
                {
                    projLines.add(lines.get(ii));
                }

                for (int j = numObstLines; j < i; ++j)
                {
                    Line line = new Line();

                    double determinant = AVOMath.det(lines.get(i).direction, lines.get(j).direction);

                    if (AVOMath.fabs(determinant) <= AVOMath.AVO_EPSILON)
                    {
                        /* Line i and line j are parallel. */
                        if (lines.get(i).direction.dot(lines.get(j).direction) > 0.0)
                        {
                            /* Line i and line j point in the same direction. */
                            continue;
                        }
                        else
                        {
                            /* Line i and line j point in opposite direction. */
                            line.point = (lines.get(i).point.add(lines.get(j).point)).multiply(0.5);
                        }
                    }
                    else
                    {
                        line.point = lines.get(i).point.add(
                       		 lines.get(i).direction.multiply(
                       				 AVOMath.det(lines.get(j).direction , lines.get(i).point.subtract( lines.get(j).point) )/ determinant ) );
                       		 

                    line.direction = AVOMath.normalize(lines.get(j).direction.subtract(lines.get(i).direction));
                    projLines.add(line);
                }

                Double2D tempResult = new Double2D(result);
                if (linearProgram2(projLines, radius, new Double2D(-lines.get(i).direction.y, lines.get(i).direction.x), true, result) < projLines.size())
                {
                    /* This should in principle not happen.  The result is by definition
                     * already in the feasible region of this linear program. If it fails,
                     * it is due to small floating point error, and the current result is
                     * kept.
                     */
                    result.setTo(tempResult );
                }

                distance = AVOMath.det(lines.get(i).direction, lines.get(i).point.subtract(new Double2D(result)));
            }
        }
    }
 }

}
