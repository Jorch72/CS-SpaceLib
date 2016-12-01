// See EntityTrackingMotionController.cs
public abstract class EntityTrackingMotionController:MotionController{
protected MyDetectedEntityInfo entity;protected IMyCameraBlock camera;protected double lastScanTime;
public EntityTrackingMotionController(MyDetectedEntityInfo e,IMyCameraBlock c){entity=e;camera=c;}
public MotionDriver.MotionTarget Tick(MotionDriver.MotionState s,double d){if(entity.IsEmpty())return Tick(s,entity,d);var v=new Vector3D(entity.Velocity);var p=entity.HitPosition.Value+v*(s.Time-lastScanTime);var f=p-camera.GetPosition();var t=f.Length()+50;if(camera.CanScan(t+5)){f.Normalize();var x=camera.Raycast(p+50*f);if(!x.IsEmpty()){entity=x;lastScanTime=s.Time;}}return Tick(s,entity,d);}
public virtual void OnArrived(MotionDriver.MotionState state){}
public abstract MotionDriver.MotionTarget Tick(MotionDriver.MotionState s,MyDetectedEntityInfo e,double d);
public abstract string Serialize();
}