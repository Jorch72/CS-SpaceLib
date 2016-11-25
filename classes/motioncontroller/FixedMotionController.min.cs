// See FixedMotionController.cs
public class FixedMotionController:MotionController{
private readonly MotionDriver.MotionTarget T;
public FixedMotionController(MotionDriver.MotionTarget t){T=t;}
public FixedMotionController(string s){T=new MotionDriver.MotionTarget(Serializer.StripPrefix(s,"Fixed:"));}
public MotionDriver.MotionTarget Tick(MotionDriver.MotionState s,double d){return T;}
public void OnArrived(MotionDriver.MotionState s){}
public string Serialize(){return "Fixed:" + T.Serialize();}
}