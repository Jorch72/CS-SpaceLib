// Goes to a fixed point, rotation and/or velocity.

public class FixedMotionController : MotionController
{
	private MotionDriver.MotionTarget target;

	public FixedMotionController(MotionDriver.MotionTarget target)
	{
		this.target = target;
	}

	public FixedMotionController(string serialized)
	{
		target = new MotionDriver.MotionTarget(Serializer.StripPrefix(serialized, "Fixed:"));
	}

	public MotionDriver.MotionTarget Tick(MotionDriver.MotionState state, double delta)
	{
		return target;
	}

	public void OnArrived(MotionDriver.MotionState state)
	{
	}

	public void OnArrivedPosition(MotionDriver.MotionState state)
	{
		target.Position = null;
	}

	public void OnArrivedOrientation(MotionDriver.MotionState state)
	{
		target.Rotation = null;
	}

	public void MoveTo(Vector3D vector)
	{
		target.Position = vector;
	}

	public void SetOrientation(IMyTerminalBlock relativeTo, Quaternion orientation)
	{
		target.SetOrientation(relativeTo, orientation);
	}

	public string Serialize()
	{
		return "Fixed:" + target.Serialize();
	}
}
