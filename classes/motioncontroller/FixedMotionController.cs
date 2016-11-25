// Goes to a fixed point, rotation and/or velocity.

public class FixedMotionController : MotionController
{
	private readonly MotionDriver.MotionTarget target;

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

	public string Serialize()
	{
		return "Fixed:" + target.Serialize();
	}
}
