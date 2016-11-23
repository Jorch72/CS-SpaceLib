// The distance measurer automatically measures the distance to the entity in front of it.
// It scans intelligently by using the most recent scan as basis for the next.
// It loads the camera by name, refreshing it only when necessary.
//
// Usage:
// - In the constructor:
//   var measurer = new DistanceMeasurer(this, "Camera", myListener, 5000);
// - In the Main method, called every tick:
//   measurer.Tick();

public delegate void DistanceListener(MyDetectedEntityInfo result, double distance);

public class DistanceMeasurer
{
	private readonly Program program;
	private readonly String cameraName;
	private readonly double maxDistance;
	private readonly DistanceListener listener;

	private int tickCounter = 0;
	public IMyCameraBlock camera;
	private double scanDistance;

	public DistanceMeasurer(Program program, String cameraName, DistanceListener listener, double maxDistance)
	{
		this.program = program;
		this.cameraName = cameraName;
		this.maxDistance = maxDistance;
		this.scanDistance = maxDistance;
		this.listener = listener;
	}

	public void Tick()
	{
		if ((camera == null || !camera.IsWorking) && (tickCounter % 60) == 0)
		{
			camera = program.GridTerminalSystem.GetBlockWithName(cameraName) as IMyCameraBlock;
			if (camera != null)
				camera.EnableRaycast = true;
		}
		tickCounter++;

		if ((tickCounter % 6) != 0)
			return;
		if (camera == null || !camera.IsWorking)
			return;
		if (!camera.CanScan(scanDistance))
			return;

		MyDetectedEntityInfo result = camera.Raycast(scanDistance);
		double distanceToObject = -1;
		if (result.HitPosition.HasValue)
			distanceToObject = (camera.GetPosition() - result.HitPosition.Value).Length();

		listener(result, distanceToObject);

		if (distanceToObject < 0)
			scanDistance += 100;
		else
			scanDistance = distanceToObject + 100;
	}
}
