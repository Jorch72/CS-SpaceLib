// See MotionDriver.cs
public interface MotionController{MotionDriver.MotionTarget Tick(MotionDriver.MotionState s,double d);void OnArrived(MotionDriver.MotionState s);string Serialize();}
public delegate MotionController MotionControllerDeserializer(string s);
public class MotionDriver{
public class MotionState{
public readonly double Time,BaseMass,TotalMass,PowerPosX,PowerPosY,PowerPosZ,PowerNegX,PowerNegY,PowerNegZ;
public readonly Vector3D Position,VelocityLocal,VelocityWorld,AngularVelocityWorldYPR,AngularVelocityLocalYPR,GravityWorld,GravityLocal;
public readonly Quaternion Orientation,AngularVelocityWorld,AngularVelocityLocal;
public readonly Matrix WorldMatrix,WorldMatrixInverse;
public MotionState(MotionDriver d){Time=d.TM;Position=d.SC.GetPosition();var o=MatrixD.Invert(d.SC.WorldMatrix.GetOrientation());Orientation=Quaternion.CreateFromRotationMatrix(o);BaseMass=d.B;TotalMass=d.M;WorldMatrix=d.SC.WorldMatrix;WorldMatrixInverse=MatrixD.Invert(this.WorldMatrix);PowerPosX=d.PX;PowerPosY=d.PY;PowerPosZ=d.PZ;PowerNegX=d.NX;PowerNegY=d.NY;PowerNegZ=d.NZ;var v=d.SC.GetShipVelocities();VelocityWorld=v.LinearVelocity;VelocityLocal=Vector3D.TransformNormal(VelocityWorld,WorldMatrixInverse);AngularVelocityWorldYPR=v.AngularVelocity;Quaternion.CreateFromYawPitchRoll((float)-AngularVelocityWorldYPR.Y,(float)AngularVelocityWorldYPR.X,(float)-AngularVelocityWorldYPR.Z,out AngularVelocityWorld);AngularVelocityLocalYPR=Vector3D.TransformNormal(AngularVelocityLocalYPR,WorldMatrixInverse);Quaternion.CreateFromYawPitchRoll((float)-AngularVelocityLocalYPR.Y,(float)AngularVelocityLocalYPR.X,(float)-AngularVelocityLocalYPR.Z,out AngularVelocityLocal);GravityWorld=d.SC.GetNaturalGravity();GravityLocal=Vector3D.TransformNormal(GravityWorld,WorldMatrixInverse);}
}

public struct MotionTarget{
public static MotionTarget ToPosition(Vector3D p){return new MotionTarget(p,null,null);}
public static MotionTarget Linear(Vector3D p,Vector3D s){return new MotionTarget(p,s,null);}
public static MotionTarget Orientation(Quaternion o){return new MotionTarget(null,null,o);}
public readonly Vector3D? Position;public readonly Vector3D Speed;public readonly Quaternion? Rotation;
public MotionTarget(string s){string[] i=s.Split(':');Position=i[0].Length==0?null:(Vector3D?)Serializer.ParseVector(i[0]);Speed=i[1].Length==0?Vector3D.Zero:Serializer.ParseVector(i[1]);Rotation = i[2].Length == 0 ? null : (Quaternion?)Serializer.ParseQuaternion(i[2]);}
public MotionTarget(Vector3D? p,Vector3D? s,Quaternion? r){Position=p;Speed=s??Vector3D.Zero;Rotation=r;}
public string Serialize(){var r=new StringBuilder();if(Position.HasValue)r.Append(Serializer.SerializeVector(Position.Value));r.Append(":");if(Speed!=Vector3D.Zero)r.Append(Serializer.SerializeVector(Speed));r.Append(":");if(Rotation.HasValue)r.Append(Serializer.SerializeQuaternion(Rotation.Value));return r.ToString();}
}

public double positionPrecision = 0.02;
public double velocityPrecision = 0.1;
public double angularPrecision = 0.001;
public Vector3D rotationDampening = new Vector3D(0.5, 0.5, 0.5);
public Vector3D angularCorrectionFactor = new Vector3D(5, 5, 5);

private void InitThrusterPowers(){
register("SmallBlockSmallThrust",12000,0,1,1,.3);
register("SmallBlockLargeThrust",144000,0,1,1,.3);
register("LargeBlockSmallThrust",288000,0,1,1,.3);
register("LargeBlockLargeThrust",3600000,0,1,1,.3);
register("SmallBlockLargeHydrogenThrust",400000,0,1,1,1);
register("SmallBlockSmallHydrogenThrust",82000,0,1,1,1);
register("LargeBlockLargeHydrogenThrust",6000000,0,1,1,1);
register("LargeBlockSmallHydrogenThrust", 900000,0,1,1,1);
register("SmallBlockLargeAtmosphericThrust",408000,.3,1,0,1,true);
register("SmallBlockSmallAtmosphericThrust",80000,.3,1,0,1,true);
register("LargeBlockLargeAtmosphericThrust",5400000,.3,1,0,1,true);
register("LargeBlockSmallAtmosphericThrust",420000,.3,1,0,1,true);
}

private const int TR=6;private const double TT=1.0/60.0,D=TT*TR,D2=D*D/2;private readonly Dictionary<string,ThrusterInfo>TH=new Dictionary<string,ThrusterInfo>();private readonly IMyShipController SC;private readonly Program P;private readonly double MS;public readonly List<IMyGyro>G=new List<IMyGyro>();public readonly List<IMyThrust>T=new List<IMyThrust>();private int TI=0;private MotionController MC;private bool GO=false,GH=false,TS=false,ED=false;private double B,M,AD=1.0,AA=8000,TM=0,PX=0,NX=0,PY=0,NY=0,PZ=0,NZ=0;
public MotionDriver(IMyShipController s,Program p,double m){SC=s;P=p;MS=m;InitThrusterPowers();}
public MotionDriver(IMyShipController s,Program p,double m,string t,MotionControllerDeserializer d):this(s,p,m){if(t==null||t.Length==0)return;var e=t.Split(":".ToCharArray(),2);SetController(d(e[1]),double.Parse(e[0]));}
private void register(string s,double f,double a,double b,double c,double d,bool n=false){TH[s]=new ThrusterInfo(f,a,b,c,d,n);}
public MotionState GetState(){return new MotionState(this);}
public void SetPlanetAtmosphere(double d,double a){this.AD=d;this.AA=a;}
public void SetController(MotionController c,double t=0){this.TM=t;this.MC=c;}
public string Serialize(){return String.Format("{0:0.00}:{1}",TM,MC.Serialize());}
public void Tick(){TI++;if((TI%60)==1)UP();if((TI%60)==2)UM();if((TI%60)==3)UT();if((TI%60)==4)UG();if((TI%6)!=0)return;if(MC==null){CT();SG(false);return;}var s=GetState();var t=MC.Tick(s, D);var a=true;if(t.Position.HasValue||t.Speed.LengthSquared()>0)a&=T1(s, t);else CT();if(t.Rotation.HasValue)a&=G1(s,t);else SG(false);if(a)MC.OnArrived(s);TM+=D;}
private bool T1(MotionState s,MotionTarget t){var w=MatrixD.Invert(SC.WorldMatrix);var v=s.VelocityLocal;var p=Vector3D.Transform(s.Position,w);var g=s.GravityLocal;var u=t.Speed;var x=Vector3D.Transform(t.Position??(s.Position+(u+v)*D*.66),w);var a=u-v;var b=x-p;if(a.LengthSquared()<velocityPrecision&&b.LengthSquared()<positionPrecision){CT();SD(true);return true;}var c=GA(a)+g;var d=a/c;var e=p+v*d+c*d*d/2;var f=x-e;var h=GA(f);var i=h+g;var j=(f-v*D)/D2-g;var o=Vector3D.Max(-Vector3D.One,Vector3D.Min(Vector3D.One,.5*j/h))*100;if(f.X<0)o.X=-o.X;if(f.Y<0)o.Y=-o.Y;if(f.Z<0)o.Z=-o.Z;if(v.LengthSquared()>=MS*MS&&(o*v).Min()>=-.01){CT();return false;}SD(d.Max()<D);TO(o);return false;}
private Vector3D GA(Vector3D d){return new Vector3D((d.X>0?PX:-NX)/M,(d.Y>0?PY:-NY)/M,(d.Z>0?PZ:-NZ)/M);}
private bool G1(MotionState s,MotionTarget t){var a=Quaternion.Inverse(t.Rotation.Value);var b=s.Orientation;var d=Math.Abs(b.X-a.X)+Math.Abs(b.Y-a.Y)+Math.Abs(b.Z-a.Z)+Math.Abs(b.W-a.W);SG(true);bool h=d<angularPrecision&&s.AngularVelocityLocalYPR.LengthSquared()<angularPrecision;HG(h);if(h)return true;foreach(var g in G){var c=CG(g,a);var w=MatrixD.Invert(g.WorldMatrix);var x=Vector3D.TransformNormal(s.AngularVelocityWorldYPR,w);var y=c*angularCorrectionFactor;
var z=y-s.AngularVelocityLocalYPR*rotationDampening;g.SetValueFloat("Yaw",-(float)z.Y);g.SetValueFloat("Pitch",(float)z.X);g.SetValueFloat("Roll",-(float)z.Z);}return false;}
private void UT(){T.Clear();P.GridTerminalSystem.GetBlocksOfType(T,t=>t.IsWorking&&t.CubeGrid==SC.CubeGrid);}
private void UG(){G.Clear();P.GridTerminalSystem.GetBlocksOfType(G,g=>g.IsWorking&&g.CubeGrid==SC.CubeGrid);}
private void SG(bool g){if(g==GO)return;GO=g;foreach(var y in G)y.SetValueBool("Override",g);}
private void HG(bool h){if(h==GH)return;GH=h;foreach(var y in G){y.SetValueFloat("Yaw",0);y.SetValueFloat("Pitch",0);y.SetValueFloat("Roll",0);}}
private void CT(){if(TS)return;TS=true;foreach(var t in T)t.SetValueFloat("Override",0);}
private void TO(Vector3D o){TS=false;var a=SC.Orientation;foreach(var t in T){var d=a.TransformDirectionInverse(Base6Directions.GetFlippedDirection(t.Orientation.Forward));switch(d){case Base6Directions.Direction.Right:t.SetValueFloat("Override",Math.Max(0,(float)o.X));break;case Base6Directions.Direction.Left:t.SetValueFloat("Override",Math.Max(0,(float)-o.X));break;case Base6Directions.Direction.Up:t.SetValueFloat("Override", Math.Max(0,(float)o.Y));break;case Base6Directions.Direction.Down:t.SetValueFloat("Override", Math.Max(0,(float)-o.Y));break;case Base6Directions.Direction.Backward:t.SetValueFloat("Override", Math.Max(0,(float)o.Z));break;case Base6Directions.Direction.Forward:t.SetValueFloat("Override",Math.Max(0,(float)-o.Z));break;}}}
private void UP(){PX=0;NX=0;PY=0;NY=0;PZ=0;NZ=0;bool a = false;var b=.0;double d;if(SC.TryGetPlanetElevation(MyPlanetElevation.Sealevel,out d)){a=d<AA;if(a)b=PA(d);}var c=SC.Orientation;foreach(var t in T){var e=c.TransformDirectionInverse(Base6Directions.GetFlippedDirection(t.Orientation.Forward));var f=TH.GetValueOrDefault(t.BlockDefinition.SubtypeId);if(f==null){P.Echo("Unknown thruster type: "+t.BlockDefinition.SubtypeId);continue;}var g=f.GetPower(a,b);switch(e){case Base6Directions.Direction.Right:PX+=g;break;case Base6Directions.Direction.Left:NX+=g;break;case Base6Directions.Direction.Up:PY+=g;break;case Base6Directions.Direction.Down:NY+=g;break;case Base6Directions.Direction.Backward:PZ+=g;break;case Base6Directions.Direction.Forward:NZ+=g;break;}}}
private void UM(){var m=SC.CalculateShipMass();B=m.BaseMass;M=m.TotalMass;}
private Vector3D CG(IMyGyro g,Quaternion o){Quaternion a;g.Orientation.GetQuaternion(out a);return QY(Quaternion.CreateFromRotationMatrix(MatrixD.Transform(g.WorldMatrix,a*o)));}
private Vector3D QY(Quaternion r){var t=MatrixD.CreateFromQuaternion(r);var x=new Vector3D();MatrixD.GetEulerAnglesXYZ(ref t,out x);return x;}
private double CR(double r){if(r>Math.PI)return r-2*Math.PI;if(r<-Math.PI)return r+2*Math.PI;return r;}
private void SD(bool e){if(e==ED)return;ED=e;SC.SetValueBool("DampenersOverride",e);}
private double PA(double a){return MathHelper.Clamp(1.0-a/AA,.0,1.0)*AD;}
}

public class ThrusterInfo{
public readonly double Force,MinPlanetaryInfluence,MaxPlanetaryInfluence,EffectivenessAtMinInfluence,EffectivenessAtMaxInfluence;public readonly bool NeedsAtmosphereForInfluence;
public ThrusterInfo(double f,double a,double b,double c,double d,bool n){Force=f;MinPlanetaryInfluence=a;MaxPlanetaryInfluence=b;EffectivenessAtMinInfluence=c;EffectivenessAtMaxInfluence=d;NeedsAtmosphereForInfluence=n;}
public double GetPower(bool a,double b){if(!a)return NeedsAtmosphereForInfluence?0:Force*EffectivenessAtMinInfluence;if(EffectivenessAtMinInfluence==EffectivenessAtMaxInfluence)return Force*EffectivenessAtMaxInfluence;var x=MathHelper.Clamp((b-MinPlanetaryInfluence)/(MaxPlanetaryInfluence - MinPlanetaryInfluence),0f,1f);return Force*MathHelper.Lerp(EffectivenessAtMinInfluence,EffectivenessAtMaxInfluence,x);}
}

public static class Serializer{
public static string SerializeVector(Vector3D v){return String.Format("{0:0.000} {1:0.000} {2:0.000}",v.X,v.Y,v.Z);}
public static string SerializeQuaternion(Quaternion q){return String.Format("{0:0.000} {1:0.000} {2:0.000} {3:0.000}",q.X,q.Y,q.Z,q.W);}
public static string SerializeMatrix(MatrixD m){return String.Format("{0:0.000} {1:0.000} {2:0.000} {3:0.000} {4:0.000} {5:0.000} {6:0.000} {7:0.000} {8:0.000} {9:0.000} {10:0.000} {11:0.000} {12:0.000} {13:0.000} {14:0.000} {15:0.000}",m.M11,m.M12,m.M13,m.M14,m.M21,m.M22,m.M23,m.M24,m.M31,m.M32,m.M33,m.M34,m.M41,m.M42,m.M43,m.M44);}
public static string SerializeDetectedEntity(MyDetectedEntityInfo e){return String.Format("{0}:{1}:{2}:{3}:{4}:{5}:{6}:{7}:{8}:{9}",SerializeVector(e.BoundingBox.Min),SerializeVector(e.BoundingBox.Max),e.EntityId,e.HitPosition.HasValue ? SerializeVector(e.HitPosition.Value) : "",e.Name.Replace(':', ' '),SerializeMatrix(e.Orientation),(int)e.Relationship,e.TimeStamp,(int)e.Type,SerializeVector(e.Velocity));}
public static Vector3D ParseVector(string s){var c=s.Split(' ');return new Vector3D(double.Parse(c[0]),double.Parse(c[1]),double.Parse(c[2]));}
public static Quaternion ParseQuaternion(string s){var c=s.Split(' ');return new Quaternion(float.Parse(c[0]),float.Parse(c[1]),float.Parse(c[2]),float.Parse(c[3]));}
public static MatrixD ParseMatrix(string s){var c=s.Split(' ');return new MatrixD(double.Parse(c[0]),double.Parse(c[1]),double.Parse(c[2]),double.Parse(c[3]),double.Parse(c[4]),double.Parse(c[5]),double.Parse(c[6]),double.Parse(c[7]),double.Parse(c[8]),double.Parse(c[9]),double.Parse(c[10]),double.Parse(c[11]),double.Parse(c[12]),double.Parse(c[13]), double.Parse(c[14]), double.Parse(c[15]));}
public static MyDetectedEntityInfo ParseDetectedEntity(string s){var p=s.Split(':');return new MyDetectedEntityInfo(long.Parse(p[2]),p[4],(MyDetectedEntityType)int.Parse(p[8]),p[3].Length==0?(Vector3D?)null:ParseVector(p[3]),ParseMatrix(p[5]),ParseVector(p[9]), (MyRelationsBetweenPlayerAndBlock)int.Parse(p[6]),new BoundingBoxD(ParseVector(p[0]),ParseVector(p[1])),long.Parse(p[7]));}
public static string StripPrefix(string v,string p){if (!v.StartsWith(p))throw new InvalidOperationException("Value doesn't start with the given prefix");return v.Substring(p.Length);}
}