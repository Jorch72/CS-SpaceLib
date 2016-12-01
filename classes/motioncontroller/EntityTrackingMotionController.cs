// Base class intended for motion controllers that need to keep track of a target.
// This motion controller is provided with an entity to track (for instance, one which was
// detected earlier) and attempts to keep tracking it even if it moves or rotates.
//
// Note that it doesn't always manage to do so. In such case, the entity will not be updated
// but remain at its last detected value. Use lastScanTime to know when the entity was last
// seen.

public abstract class EntityTrackingMotionController : MotionController
{
	protected MyDetectedEntityInfo entity;
	protected IMyCameraBlock camera;
	protected double lastScanTime;

	public EntityTrackingMotionController(MyDetectedEntityInfo entity, IMyCameraBlock camera)
	{
		this.entity = entity;
		this.camera = camera;
	}

	public MotionDriver.MotionTarget Tick(MotionDriver.MotionState state, double delta)
	{
		if (entity.IsEmpty())
			return Tick(state, entity, delta);

		Vector3D entityVelocity = new Vector3D(entity.Velocity);
		Vector3D scanPosition = entity.HitPosition.Value + entityVelocity * (state.Time - lastScanTime);
		Vector3D difference = scanPosition - camera.GetPosition();
		double scanDistance = difference.Length() + 50;
		if (camera.CanScan(scanDistance + 5))
		{
			difference.Normalize();
			MyDetectedEntityInfo detected = camera.Raycast(scanPosition + 50 * difference);
			if (!detected.IsEmpty())
			{
				entity = detected;
				lastScanTime = state.Time;
			}
		}

		return Tick(state, entity, delta);
	}

	public virtual void OnArrived(MotionDriver.MotionState state)
	{

	}

	public abstract MotionDriver.MotionTarget Tick(MotionDriver.MotionState state, MyDetectedEntityInfo entity, double delta);

	public abstract string Serialize();
}