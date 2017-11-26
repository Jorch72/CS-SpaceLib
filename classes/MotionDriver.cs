// The MotionDriver is an advanced motion control system making it much easier to make accurately
// controlled drones. Motion drivers work both in space and in natural gravity (in the latter case,
// gravity is automatically taken into account).
//
// To use the MotionDriver, you make an instance for your script (and provide it with the ship controller
// it should be using - it doesn't even have to be a remote control block).
// 
// The MotionDriver will also need a MotionController to control its behaviour. You can change the controller
// at any moment, or set it to null to disable the MotionDriver.

// Interface for motion controllers. Motion controllers define where, how fast and in which orientation the
// MotionDriver should set the given ship. (in fact, it determines the position it should be after delta seconds).
// 
// Implementation highly depends on the purpose. Sample implementations are provided in the motioncontroller directory.
public interface MotionController
{
	MotionDriver.MotionTarget Tick(MotionDriver.MotionState state, double delta);

	void OnArrived(MotionDriver.MotionState state);
	void OnArrivedPosition(MotionDriver.MotionState state);
	void OnArrivedOrientation(MotionDriver.MotionState state);

	string Serialize();
}

// Used to deserialize motioncontrollers
public delegate MotionController MotionControllerDeserializer(string serialized);

public class MotionDriver
{
	// Defines the current state of the controlled ship. It is passed to the motion controller to help it
	// calculate the target position, velocity and orientation.
	public class MotionState
	{
		public readonly double Time;
		public readonly Vector3D Position;
		public readonly Quaternion Orientation;

		public readonly double BaseMass;
		public readonly double TotalMass;

		public readonly Matrix WorldMatrix;
		public readonly Matrix WorldMatrixInverse;

		public readonly Vector3D VelocityLocal;
		public readonly Vector3D VelocityWorld;

		public readonly Vector3D AngularVelocityWorldYPR;
		public readonly Quaternion AngularVelocityWorld;
		public readonly Vector3D AngularVelocityLocalYPR;
		public readonly Quaternion AngularVelocityLocal;

		public readonly Vector3D GravityWorld;
		public readonly Vector3D GravityLocal;

		public readonly double PowerPosX; // right
		public readonly double PowerNegX; // left
		public readonly double PowerPosY; // up
		public readonly double PowerNegY; // down
		public readonly double PowerPosZ; // backward
		public readonly double PowerNegZ; // forward

		public MotionState(MotionDriver driver)
		{
			Time = driver.time;
			Position = driver.shipController.GetPosition();
			Orientation = Quaternion.CreateFromRotationMatrix(driver.shipController.CubeGrid.WorldMatrix.GetOrientation());

			BaseMass = driver.baseMass;
			TotalMass = driver.totalMass;

			WorldMatrix = driver.shipController.CubeGrid.WorldMatrix;
			WorldMatrixInverse = MatrixD.Invert(WorldMatrix);

			PowerPosX = driver.powerPosX;
			PowerPosY = driver.powerPosY;
			PowerPosZ = driver.powerPosZ;
			PowerNegX = driver.powerNegX;
			PowerNegY = driver.powerNegY;
			PowerNegZ = driver.powerNegZ;

			var velocities = driver.shipController.GetShipVelocities();
			VelocityWorld = velocities.LinearVelocity;
			VelocityLocal = Vector3D.TransformNormal(VelocityWorld, WorldMatrixInverse);

			AngularVelocityWorldYPR = velocities.AngularVelocity;
			Quaternion.CreateFromYawPitchRoll((float)-AngularVelocityWorldYPR.Y, (float)AngularVelocityWorldYPR.X, (float)-AngularVelocityWorldYPR.Z, out AngularVelocityWorld);
			AngularVelocityLocalYPR = Vector3D.TransformNormal(AngularVelocityLocalYPR, WorldMatrixInverse);
			Quaternion.CreateFromYawPitchRoll((float)-AngularVelocityLocalYPR.Y, (float)AngularVelocityLocalYPR.X, (float)-AngularVelocityLocalYPR.Z, out AngularVelocityLocal);

			// TODO: double-check gravity compensation
			GravityWorld = driver.shipController.GetNaturalGravity();
			GravityLocal = Vector3D.TransformNormal(GravityWorld, WorldMatrixInverse);
		}
	}

	// Motion targets define where the ship should go. They are constructed by the motion controller tick function.
	public struct MotionTarget
	{
		public static MotionTarget ToPosition(Vector3D position)
		{
			return new MotionTarget(position, null, null);
		}

		public static MotionTarget LinearLocal(IMyTerminalBlock relativeTo, Vector3D position, Vector3D velocity)
		{
			return new MotionTarget(position, Vector3D.TransformNormal(velocity, relativeTo.WorldMatrix), null);
		}

		public static MotionTarget LinearWorld(Vector3D position, Vector3D velocity)
		{
			return new MotionTarget(position, velocity, null);
		}

		public static MotionTarget Orientation(IMyTerminalBlock relativeTo, Quaternion orientation)
		{
			Quaternion blockOrientation;
			relativeTo.Orientation.GetQuaternion(out blockOrientation);
			return new MotionTarget(null, null, Quaternion.Inverse(blockOrientation) * orientation);
		}

		public Vector3D? Position;
		public Vector3D Speed;
		public Quaternion? Rotation;

		public MotionTarget(string serialized)
		{
			string[] items = serialized.Split(':');
			Position = items[0].Length == 0 ? null : (Vector3D?)Serializer.ParseVector(items[0]);
			Speed = items[1].Length == 0 ? Vector3D.Zero : Serializer.ParseVector(items[1]);
			Rotation = items[2].Length == 0 ? null : (Quaternion?)Serializer.ParseQuaternion(items[2]);
		}

		public MotionTarget(Vector3D? position, Vector3D? speed, Quaternion? rotation)
		{
			Position = position;
			Speed = speed ?? Vector3D.Zero;
			Rotation = rotation;
		}

		public void SetOrientation(IMyTerminalBlock relativeTo, Quaternion orientation)
		{
			Quaternion blockOrientation;
			relativeTo.Orientation.GetQuaternion(out blockOrientation);
			Rotation = Quaternion.Inverse(blockOrientation) * orientation;
		}

		public string Serialize()
		{
			StringBuilder result = new StringBuilder();
			if (Position.HasValue)
				result.Append(Serializer.SerializeVector(Position.Value));
			result.Append(":");
			if (Speed != Vector3D.Zero)
				result.Append(Serializer.SerializeVector(Speed));
			result.Append(":");
			if (Rotation.HasValue)
				result.Append(Serializer.SerializeQuaternion(Rotation.Value));
			return result.ToString();
		}
	}

	// tweakable variables
	public double positionPrecision = 0.04; // square of tolerated position difference
	public double velocityPrecision = 0.1; // square of tolerated velocity difference
	public double angularPrecision = 0.001;
	// !! tweak this according to your ship properties
	// - without proper tweaking your ship will rotate either slowly or overshoot its target rotation
	public Vector3D rotationDampening = new Vector3D(0.5, 0.5, 0.5);
	// you could tweak this too, determines how fast it should rotate
	// low values will decrease rotation speed, but too high values will destabilize the rotation
	public Vector3D angularCorrectionFactor = new Vector3D(10, 10, 10);

	private void InitThrusterPowers()
	{
		// If modded thrusters are used, add their subblock IDs here
		register("SmallBlockSmallThrust", 12000, 0, 1, 1.0, 0.3);
		register("SmallBlockLargeThrust", 144000, 0, 1, 1.0, 0.3);
		register("LargeBlockSmallThrust", 288000, 0, 1, 1.0, 0.3);
		register("LargeBlockLargeThrust", 3600000, 0, 1, 1.0, 0.3);
		register("SmallBlockLargeHydrogenThrust", 400000, 0, 1, 1, 1);
		register("SmallBlockSmallHydrogenThrust", 82000, 0, 1, 1, 1);
		register("LargeBlockLargeHydrogenThrust", 6000000, 0, 1, 1, 1);
		register("LargeBlockSmallHydrogenThrust", 900000, 0, 1, 1, 1);
		register("SmallBlockLargeAtmosphericThrust", 408000, 0.3, 1.0, 0, 1, true);
		register("SmallBlockSmallAtmosphericThrust", 80000, 0.3, 1.0, 0, 1, true);
		register("LargeBlockLargeAtmosphericThrust", 5400000, 0.3, 1.0, 0, 1, true);
		register("LargeBlockSmallAtmosphericThrust", 420000, 0.3, 1.0, 0, 1, true);
		// Armored thrusters
		register("SmallBlockArmorThrust", 12000, 0, 1, 1, .3);
		register("SmallBlockArmorSlopedThrust", 12000, 0, 1, 1, .3);
		register("SBLArmorThrust", 144000, 0, 1, 1, .3);
		register("SBLArmorThrustSloped", 144000, 0, 1, 1, .3);
		register("LargeBlockArmorThrust", 288000, 0, 1, 1, .3);
		register("LargeBlockArmorSlopedThrust", 288000, 0, 1, 1, .3);
		register("LBLArmorThrust", 3600000, 0, 1, 1, .3);
		register("LBLArmorThrustSloped", 3600000, 0, 1, 1, .3);

	}

	private const double TICK_TIME = 1.0 / 60.0;
	private const int TICKS_PER_RUN = 6;
	private const double DELTA = TICK_TIME * TICKS_PER_RUN;
	private const double DELTA_SQ_2 = DELTA * DELTA / 2;

	private readonly Dictionary<String, ThrusterInfo> thrusterTypes = new Dictionary<String, ThrusterInfo>();

	private readonly IMyShipController shipController;
	private readonly Program program;
	private readonly double maxSpeed;

	public readonly List<IMyGyro> gyros = new List<IMyGyro>();
	public readonly List<IMyThrust> thrusters = new List<IMyThrust>();

	private double baseMass;
	private double totalMass;

	private int ticks = 0;
	private double time = 0;
	private MotionController controller;

	private double atmosphereDensity = 1.0;
	private double atmosphereAltitude = 8000;

	private bool gyroOverride = false;
	private bool gyroHold = false;
	private bool thrustersStopped = false;
	private bool enableDampeners = false;

	private double powerPosX = 0; // right
	private double powerNegX = 0; // left
	private double powerPosY = 0; // up
	private double powerNegY = 0; // down
	private double powerPosZ = 0; // backward
	private double powerNegZ = 0; // forward

	public string MotionDebug = "";
	public MotionState LastState = null;
	public string MotionStateDebug = "";

	/// <summary>
	/// Initializes a motion controller. Only blocks on the grid of the given ship controller will be taken into account.
	/// </summary>
	/// <param name="shipController">Ship controller to be used to determine ship.</param>
	/// <param name="terminalSystem">Terminal system of this ship</param>
	/// <param name="dampening">Rotation dampening factor. Increase this value if the ship overshoots its target orientation when rotating (and wiggles before stabilizing). Decrease it when the ship rotates slowly.</param>
	public MotionDriver(IMyShipController shipController, Program program, double maxSpeed)
	{
		this.shipController = shipController;
		this.program = program;
		this.maxSpeed = maxSpeed;

		InitThrusterPowers();
	}

	// Used to construct a motion driver from a saved state
	public MotionDriver(IMyShipController shipController, Program program, double maxSpeed, string serialized, MotionControllerDeserializer deserializer)
		:this(shipController, program, maxSpeed)
	{
		if (serialized == null || serialized.Length == 0)
			return;

		string[] elements = serialized.Split(":".ToCharArray(), 2);
		SetController(deserializer(elements[1]), double.Parse(elements[0]));
	}

	private void register(
		string subId,
		double force,
		double minPlanetaryInfluence,
		double maxPlanetaryInfluence,
		double effectivenessAtMinInfluence,
		double effectivenessAtMaxInfluence,
		bool needsAtmosphereForInfluence = false)
	{
		thrusterTypes[subId] = new ThrusterInfo(
			force,
			minPlanetaryInfluence,
			maxPlanetaryInfluence,
			effectivenessAtMinInfluence,
			effectivenessAtMaxInfluence,
			needsAtmosphereForInfluence);
	}

	public MotionState GetState()
	{
		return new MotionState(this);
	}

	// Probably 1.0 & 8000 for earth
	public void SetPlanetAtmosphere(double density, double altitude)
	{
		this.atmosphereDensity = density;
		this.atmosphereAltitude = altitude;
	}

	public void SetController(MotionController controller, double time = 0)
	{
		this.time = time;
		this.controller = controller;
	}

	public string Serialize()
	{
		return String.Format("{0:0.00}:{1}", time, controller.Serialize());
	}

	public void Tick()
	{
		ticks++;

		if ((ticks % 60) == 1)
			UpdateThrusters();
		if ((ticks % 60) == 2)
			UpdateMass();
		if ((ticks % 60) == 3)
			UpdateThrusterPower();
		if ((ticks % 60) == 4)
			UpdateGyros();

		if ((ticks % 6) != 0)
			return;

		if (controller == null)
		{
			ClearThrustersOverride();
			return;
		}

		var state = GetState();
		var target = controller.Tick(state, DELTA);
		LastState = state;

		var arrivedPosition = false;
		var arrived = true;
		if (target.Position.HasValue || target.Speed.LengthSquared() > 0)
		{
			arrivedPosition = TickThrusters(state, target);
			arrived &= arrivedPosition;
		}
		else
		{
			ClearThrustersOverride();
		}

		var arrivedOrientation = false;
		if (target.Rotation.HasValue)
		{
			arrivedOrientation = TickGyros(state, target);
			arrived &= arrivedOrientation;
		}
		else
		{
			SetGyrosOverride(false);
		}

		if (arrived)
			controller.OnArrived(state);
		if (arrivedPosition)
			controller.OnArrivedPosition(state);
		if (arrivedOrientation)
			controller.OnArrivedOrientation(state);

		time += DELTA;
	}

	private bool TickThrusters(MotionState state, MotionTarget target)
	{
		Vector3D currentVelocity = state.VelocityLocal;
		Vector3D currentPosition = Vector3D.Transform(state.Position, state.WorldMatrixInverse);
		Vector3D currentGravity = state.GravityLocal;

		Vector3D targetSpeed = target.Speed;
		Vector3D targetPosition = Vector3D.Transform(target.Position ?? (state.Position + (targetSpeed + currentVelocity) * DELTA * .66), state.WorldMatrixInverse);

		Vector3D speedDifference = targetSpeed - currentVelocity;
		Vector3D positionDifference = targetPosition - currentPosition;
		if (speedDifference.LengthSquared() < velocityPrecision && positionDifference.LengthSquared() < positionPrecision)
		{
			ClearThrustersOverride();
			SetDampeners(true);
			return true;
		}


		Vector3D deceleration = GetAccelerations(speedDifference) + currentGravity;
		Vector3D decelerationTimes = speedDifference / deceleration;
		Vector3D positionAtFullStop = currentPosition + currentVelocity * decelerationTimes + deceleration * decelerationTimes * decelerationTimes / 2;

		Vector3D maxAccelerationDelta = targetPosition - positionAtFullStop;
		Vector3D shipAcceleration = GetAccelerations(maxAccelerationDelta);
		Vector3D acceleration = shipAcceleration + currentGravity;
		Vector3D accelerationForTick = (maxAccelerationDelta - currentVelocity * DELTA) / DELTA_SQ_2 - currentGravity;
		Vector3D overrides = Vector3D.Max(-Vector3D.One, Vector3D.Min(Vector3D.One, 0.5 * accelerationForTick / shipAcceleration)) * 100;
		if (maxAccelerationDelta.X < 0)
			overrides.X = -overrides.X;
		if (maxAccelerationDelta.Y < 0)
			overrides.Y = -overrides.Y;
		if (maxAccelerationDelta.Z < 0)
			overrides.Z = -overrides.Z;

		if (currentVelocity.LengthSquared() >= maxSpeed * maxSpeed && (overrides * currentVelocity).Min() >= -0.01)
		{
			ClearThrustersOverride();
			return false;
		}

		SetDampeners(decelerationTimes.Max() < DELTA);
		SetThrustersOverride(overrides);
		return false;
	}

	private Vector3D GetAccelerations(Vector3D directions)
	{
		return new Vector3D(
			(directions.X > 0 ? powerPosX : -powerNegX) / totalMass,
			(directions.Y > 0 ? powerPosY : -powerNegY) / totalMass,
			(directions.Z > 0 ? powerPosZ : -powerNegZ) / totalMass);
	}

	private Vector3D GetMaxAcceleration(Vector3D deltaV)
	{
		double powerX = deltaV.X < 0 ? powerNegX : powerPosX;
		double powerY = deltaV.Y < 0 ? powerNegY : powerPosY;
		double powerZ = deltaV.Z < 0 ? powerNegZ : powerPosZ;
		return new Vector3D(powerX / totalMass, powerY / totalMass, powerZ / totalMass);
	}

	private bool TickGyros(MotionState state, MotionTarget target)
	{
		Quaternion targetOrientation = Quaternion.Inverse(target.Rotation.Value);
		Quaternion orientation = state.Orientation;
		double delta = Math.Abs(orientation.X - targetOrientation.X)
			+ Math.Abs(orientation.Y - targetOrientation.Y)
			+ Math.Abs(orientation.Z - targetOrientation.Z)
			+ Math.Abs(orientation.W - targetOrientation.W);

		SetGyrosOverride(true);
		bool hold = delta < angularPrecision && state.AngularVelocityLocalYPR.LengthSquared() < angularPrecision;
		SetHoldGyros(hold);
		if (hold)
			return true;

		foreach (IMyGyro gyro in gyros)
		{
			Vector3D rotationAngles = calculateGyroRotation(gyro, targetOrientation);
			Vector3D rotationSpeedRad = rotationAngles * angularCorrectionFactor;

			Matrix gyroOrientation;
			gyro.Orientation.GetMatrix(out gyroOrientation);
			Vector3D relativeAngularVelocity = Vector3D.TransformNormal(state.AngularVelocityLocalYPR, gyroOrientation);
			Vector3D rotationSpeedDampened = rotationSpeedRad - relativeAngularVelocity * rotationDampening;

			gyro.SetValueFloat("Yaw", (float)rotationSpeedDampened.Y);
			gyro.SetValueFloat("Pitch", -(float)rotationSpeedDampened.X);
			gyro.SetValueFloat("Roll", (float)rotationSpeedDampened.Z);
		}

		return false;
	}

	private void UpdateThrusters()
	{
		thrusters.Clear();
		program.GridTerminalSystem.GetBlocksOfType<IMyThrust>(thrusters, thruster => thruster.IsWorking && thruster.CubeGrid == shipController.CubeGrid);
	}

	private void UpdateGyros()
	{
		gyros.Clear();
		program.GridTerminalSystem.GetBlocksOfType<IMyGyro>(gyros, gyro => gyro.IsWorking && gyro.CubeGrid == shipController.CubeGrid);
	}

	private void SetGyrosOverride(bool gyroOverride)
	{
		if (gyroOverride == this.gyroOverride)
			return;

		this.gyroOverride = gyroOverride;
		foreach (IMyGyro gyro in gyros)
			gyro.SetValueBool("Override", gyroOverride);
	}

	private void SetHoldGyros(bool hold)
	{
		if (hold == this.gyroHold)
			return;

		this.gyroHold = hold;
		foreach (IMyGyro gyro in gyros)
		{
			gyro.SetValueFloat("Yaw", 0);
			gyro.SetValueFloat("Pitch", 0);
			gyro.SetValueFloat("Roll", 0);
		}
	}

	private void ClearThrustersOverride()
	{
		if (thrustersStopped)
			return;

		thrustersStopped = true;
		foreach (IMyThrust thruster in thrusters)
			thruster.SetValueFloat("Override", 0);
	}

	private void SetThrustersOverride(Vector3D overrides)
	{
		thrustersStopped = false;

		MyBlockOrientation orientation = shipController.Orientation;
		foreach (IMyThrust thruster in thrusters)
		{
			Base6Directions.Direction direction = Base6Directions.GetFlippedDirection(thruster.Orientation.Forward);
			switch (direction)
			{
				case Base6Directions.Direction.Right:
					thruster.SetValueFloat("Override", Math.Max(0, (float)overrides.X));
					break;
				case Base6Directions.Direction.Left:
					thruster.SetValueFloat("Override", Math.Max(0, (float)-overrides.X));
					break;
				case Base6Directions.Direction.Up:
					thruster.SetValueFloat("Override", Math.Max(0, (float)overrides.Y));
					break;
				case Base6Directions.Direction.Down:
					thruster.SetValueFloat("Override", Math.Max(0, (float)-overrides.Y));
					break;
				case Base6Directions.Direction.Backward:
					thruster.SetValueFloat("Override", Math.Max(0, (float)overrides.Z));
					break;
				case Base6Directions.Direction.Forward:
					thruster.SetValueFloat("Override", Math.Max(0, (float)-overrides.Z));
					break;
			}
		}
	}

	private void UpdateThrusterPower()
	{
		MotionDebug = "";

		powerPosX = 0;
		powerNegX = 0;
		powerPosY = 0;
		powerNegY = 0;
		powerPosZ = 0;
		powerNegZ = 0;

		bool inAtmosphere = false;
		double airDensity = 0;

		double altitude;
		if (shipController.TryGetPlanetElevation(MyPlanetElevation.Sealevel, out altitude))
		{
			inAtmosphere = altitude < atmosphereAltitude;
			if (inAtmosphere)
				airDensity = getAirDensityAtPlanet(altitude);
		}

		foreach (IMyThrust thruster in thrusters)
		{
			Base6Directions.Direction direction = Base6Directions.GetFlippedDirection(thruster.Orientation.Forward);
			ThrusterInfo info = thrusterTypes.GetValueOrDefault(thruster.BlockDefinition.SubtypeId);
			if (info == null)
			{
				program.Echo("Unknown thruster type: " + thruster.BlockDefinition.SubtypeId);
				continue;
			}

			double power = info.GetPower(inAtmosphere, airDensity);
			switch (direction)
			{
				case Base6Directions.Direction.Right:
					powerPosX += power;
					break;
				case Base6Directions.Direction.Left:
					powerNegX += power;
					break;
				case Base6Directions.Direction.Up:
					powerPosY += power;
					break;
				case Base6Directions.Direction.Down:
					powerNegY += power;
					break;
				case Base6Directions.Direction.Backward:
					powerPosZ += power;
					break;
				case Base6Directions.Direction.Forward:
					powerNegZ += power;
					break;
			}
		}
	}

	private void UpdateMass()
	{
		var mass = shipController.CalculateShipMass();
		baseMass = mass.BaseMass;
		totalMass = mass.TotalMass;
	}

	private Vector3D calculateGyroRotation(IMyGyro gyro, Quaternion desiredOrientation)
	{
		Quaternion gyroOrientation;
		gyro.Orientation.GetQuaternion(out gyroOrientation);
		Quaternion relativeOrientation = Quaternion.Inverse(Quaternion.CreateFromRotationMatrix(gyro.WorldMatrix)) * Quaternion.Inverse(gyroOrientation) * desiredOrientation;
		return quaternionToYPR(relativeOrientation);
	}

	public Vector3D quaternionToYPR(Quaternion rotation)
	{
		MatrixD transform = MatrixD.CreateFromQuaternion(rotation);
		Vector3D result = new Vector3D();
		MatrixD.GetEulerAnglesXYZ(ref transform, out result);
		return result;
	}

	private double ClampRotation(double rotation)
	{
		if (rotation > Math.PI)
			return rotation - 2 * Math.PI;
		if (rotation < -Math.PI)
			return rotation + 2 * Math.PI;
		return rotation;
	}

	private void SetDampeners(bool enable)
	{
		if (enable == enableDampeners)
			return;

		enableDampeners = enable;
		shipController.SetValueBool("DampenersOverride", enable);
	}

	private double getAirDensityAtPlanet(double altitude)
	{
		double relativeAltitude = MathHelper.Clamp(1.0 - altitude / atmosphereAltitude, 0.0, 1.0);
		return relativeAltitude * atmosphereDensity;
	}
}

public class ThrusterInfo
{
	public readonly double Force;
	public readonly double MinPlanetaryInfluence;
	public readonly double MaxPlanetaryInfluence;
	public readonly double EffectivenessAtMinInfluence;
	public readonly double EffectivenessAtMaxInfluence;
	public readonly bool NeedsAtmosphereForInfluence;

	public ThrusterInfo(
		double force,
		double minPlanetoryInfluence,
		double maxPlanetaryInfluence,
		double effectivenessAtMinInfluence,
		double effectivenessAtMaxInfluence,
		bool needsAtmosphere)
	{
		Force = force;
		MinPlanetaryInfluence = minPlanetoryInfluence;
		MaxPlanetaryInfluence = maxPlanetaryInfluence;
		EffectivenessAtMinInfluence = effectivenessAtMinInfluence;
		EffectivenessAtMaxInfluence = effectivenessAtMaxInfluence;
		NeedsAtmosphereForInfluence = needsAtmosphere;
	}

	public double GetPower(bool inAtmosphere, double airDensity)
	{
		if (!inAtmosphere)
			return NeedsAtmosphereForInfluence ? 0 : Force * EffectivenessAtMinInfluence;

		if (EffectivenessAtMinInfluence == EffectivenessAtMaxInfluence)
			return Force * EffectivenessAtMaxInfluence;

		double influence = MathHelper.Clamp((airDensity - MinPlanetaryInfluence) / (MaxPlanetaryInfluence - MinPlanetaryInfluence), 0f, 1f);
		return Force * MathHelper.Lerp(EffectivenessAtMinInfluence, EffectivenessAtMaxInfluence, influence);
	}
}

public static class Serializer
{
	public static string SerializeVector(Vector3D vector)
	{
		return String.Format("{0:0.000} {1:0.000} {2:0.000}", vector.X, vector.Y, vector.Z);
	}

	public static string SerializeQuaternion(Quaternion quaternion)
	{
		return String.Format("{0:0.000} {1:0.000} {2:0.000} {3:0.000}", quaternion.X, quaternion.Y, quaternion.Z, quaternion.W);
	}

	public static string SerializeMatrix(MatrixD matrix)
	{
		return String.Format(
			"{0:0.000} {1:0.000} {2:0.000} {3:0.000} "
			+ "{4:0.000} {5:0.000} {6:0.000} {7:0.000} "
			+ "{8:0.000} {9:0.000} {10:0.000} {11:0.000} "
			+ "{12:0.000} {13:0.000} {14:0.000} {15:0.000}",
			matrix.M11, matrix.M12, matrix.M13, matrix.M14,
			matrix.M21, matrix.M22, matrix.M23, matrix.M24,
			matrix.M31, matrix.M32, matrix.M33, matrix.M34,
			matrix.M41, matrix.M42, matrix.M43, matrix.M44);
	}

	public static string SerializeDetectedEntity(MyDetectedEntityInfo entity)
	{
		Vector3D boundingBoxMin = entity.BoundingBox.Min;
		Vector3D boundingBoxMax = entity.BoundingBox.Max;

		return String.Format("{0}:{1}:{2}:{3}:{4}:{5}:{6}:{7}:{8}:{9}",
			SerializeVector(entity.BoundingBox.Min),
			SerializeVector(entity.BoundingBox.Max),
			entity.EntityId,
			entity.HitPosition.HasValue ? SerializeVector(entity.HitPosition.Value) : "",
			entity.Name.Replace(':', ' '),
			SerializeMatrix(entity.Orientation),
			(int) entity.Relationship,
			entity.TimeStamp,
			(int) entity.Type,
			SerializeVector(entity.Velocity));
	}

	public static Vector3D ParseVector(string serialized)
	{
		string[] components = serialized.Split(' ');
		return new Vector3D(double.Parse(components[0]), double.Parse(components[1]), double.Parse(components[2]));
	}

	public static Quaternion ParseQuaternion(string serialized)
	{
		string[] components = serialized.Split(' ');
		return new Quaternion(float.Parse(components[0]), float.Parse(components[1]), float.Parse(components[2]), float.Parse(components[3]));
	}

	public static MatrixD ParseMatrix(string serialized)
	{
		string[] components = serialized.Split(' ');
		return new MatrixD(
			double.Parse(components[0]), double.Parse(components[1]), double.Parse(components[2]), double.Parse(components[3]),
			double.Parse(components[4]), double.Parse(components[5]), double.Parse(components[6]), double.Parse(components[7]),
			double.Parse(components[8]), double.Parse(components[9]), double.Parse(components[10]), double.Parse(components[11]),
			double.Parse(components[12]), double.Parse(components[13]), double.Parse(components[14]), double.Parse(components[15]));
	}

	public static MyDetectedEntityInfo ParseDetectedEntity(string serialized)
	{
		string[] parts = serialized.Split(':');
		Vector3D boundingBoxMin = ParseVector(parts[0]);
		Vector3D boundingBoxMax = ParseVector(parts[1]);
		long entityId = long.Parse(parts[2]);
		Vector3D? hitPosition = parts[3].Length == 0 ? (Vector3D?)null : ParseVector(parts[3]);
		string name = parts[4];
		MatrixD orientation = ParseMatrix(parts[5]);
		MyRelationsBetweenPlayerAndBlock relationship = (MyRelationsBetweenPlayerAndBlock) int.Parse(parts[6]);
		long timestamp = long.Parse(parts[7]);
		MyDetectedEntityType type = (MyDetectedEntityType) int.Parse(parts[8]);
		Vector3D velocity = ParseVector(parts[9]);
		return new MyDetectedEntityInfo(entityId, name, type, hitPosition, orientation, velocity, relationship, new BoundingBoxD(boundingBoxMin, boundingBoxMax), timestamp);
	}

	public static string StripPrefix(string value, string prefix)
	{
		if (!value.StartsWith(prefix))
			throw new InvalidOperationException("Value doesn't start with the given prefix");
		return value.Substring(prefix.Length);
	}
}
