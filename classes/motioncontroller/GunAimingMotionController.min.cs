// See GunAimingMotionController.cs
public class GunAimingMotionController:EntityTrackingMotionController{
private double V;private Vector3D P;
public GunAimingMotionController(MyDetectedEntityInfo e,IMyCameraBlock c,double v):base(e,c){V=v;var p=e.HitPosition.Value-e.Position;var i=MatrixD.Invert(e.Orientation);P=Vector3D.TransformNormal(p,i);}
public override MotionDriver.MotionTarget Tick(MotionDriver.MotionState s,MyDetectedEntityInfo e,double d){var v=new Vector3D(e.Velocity);var a=Vector3D.Distance(s.Position,e.HitPosition.Value);var b=a/V;var t=e.Position+Vector3D.TransformNormal(P,e.Orientation)+v*(s.Time-lastScanTime+d+b);var f=-(s.Position-t);f.Normalize();return MotionDriver.MotionTarget.Orientation(Quaternion.CreateFromForwardUp(f,s.WorldMatrix.Up));}
public override void OnArrived(MotionDriver.MotionState s){}
public override string Serialize(){return String.Format("GunAiming:{0:0.00}:{1}",V,Serializer.SerializeDetectedEntity(entity));}}
