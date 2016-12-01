// Aims a gun at a target. This motion controller takes an entity to aim at, a camera to keep it in track,
// and the velocity of the bullets you intend to present it with. This motion controller will rotate the
// nose of the ship to aim at its target, taking into account the speed of the target and the travel time
// of the bullets. It also keeps track of the block it was aimed at and will take into account rotations of
// the target ship. Make sure the provided entity has a hit point, so it must be detected with a camera.
// Velocity of the gatling gun is 400 and for missiles it is 200.
//
// This thing is pretty accurate and makes manual aiming a stone age thing.
//
// Requires EntityTrackingMotionController.cs

public class GunAimingMotionController : EntityTrackingMotionController
{
	private double bulletVelocity;
	private Vector3D blockPosition;

	public GunAimingMotionController(MyDetectedEntityInfo entity, IMyCameraBlock camera, double bulletVelocity)
		: base(entity, camera)
	{
		this.bulletVelocity = bulletVelocity;

		Vector3D blockPositionGlobal = entity.HitPosition.Value - entity.Position;
		MatrixD entityWorldInv = MatrixD.Invert(entity.Orientation);
		blockPosition = Vector3D.TransformNormal(blockPositionGlobal, entityWorldInv);
	}

	public override MotionDriver.MotionTarget Tick(MotionDriver.MotionState state, MyDetectedEntityInfo entity, double delta)
	{
		Vector3D entityVelocity = new Vector3D(entity.Velocity);
		double entityDistance = Vector3D.Distance(state.Position, entity.HitPosition.Value);
		double bulletTime = entityDistance / bulletVelocity;
		Vector3D targetPosition = entity.Position + Vector3D.TransformNormal(blockPosition, entity.Orientation) + entityVelocity * (state.Time - lastScanTime + delta + bulletTime);

		Vector3D forward = -(state.Position - targetPosition);
		forward.Normalize();
		Quaternion orientation = Quaternion.CreateFromForwardUp(forward, state.WorldMatrix.Up);

		return MotionDriver.MotionTarget.Orientation(orientation);
	}

	public override void OnArrived(MotionDriver.MotionState state)
	{

	}

	public override string Serialize()
	{
		return String.Format("GunAiming:{0:0.00}:{1}", bulletVelocity, Serializer.SerializeDetectedEntity(entity));
	}
}