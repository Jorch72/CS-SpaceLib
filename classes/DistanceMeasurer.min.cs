// See DistanceMeasurer.cs

public delegate void DistanceListener(MyDetectedEntityInfo result, double distance);
public class DistanceMeasurer{
private readonly Program p;private readonly String c;private readonly double m;private readonly DistanceListener l;
private int t=0;public IMyCameraBlock camera;private double s;
public DistanceMeasurer(Program p_,String c_,DistanceListener l_,double m_){p=p_;c=c_;m=m_;s=m_;l=l_;}
public void Tick(){if((camera==null||!camera.IsWorking)&&(t%60)==0){camera=p.GridTerminalSystem.GetBlockWithName(c)as IMyCameraBlock;if(camera!=null)camera.EnableRaycast=true;}t++;if((t%6)!=0)return;if(camera==null||!camera.IsWorking)return;if(!camera.CanScan(s))return;var a=camera.Raycast(s);double b=-1;if(a.HitPosition.HasValue)b=(camera.GetPosition()-a.HitPosition.Value).Length();l(a, b);if(b<0)s+=100;else s=b+100;}
}