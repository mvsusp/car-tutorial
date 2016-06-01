using UnityEngine;
using System.Collections;
using System.Threading;

public class CarEngine : MonoBehaviour
{
    public float SuspensionRange = .1f;
    public float SuspensionDamper = 50f;
    public float SuspensionSpringFront = 18500f;
    public float SuspensionSpringRear = 9000f;
    public Material BrakeLights;

    private float _wheelRadius = .4f;
    public Vector3 _dragMultiplier = new Vector3(2,5,1);
    public float _throttle = 0;
    private float _steer = 0;
    private bool _handbrake = false;
    public Transform _centerOfMass;
    public Transform[] _frontWheels;
    public Transform[] _rearWheels;
    private Wheel[] _wheels;
    private WheelFrictionCurve _wheelFrictionCurve;
    public float _topSpeed = 160f;
    public int _numberOfGears = 5;
    public int maximumTurn = 15;
    public int minimumTurn = 10;
    public float _resetTime = 5f;
    private float _resetTimer = 0.0f;
    private float[] engineForceValues;
    private float[] gearSpeeds;
    private int currentGear;
    private float currentEnginePower = 0f;
    private float handbrakeXDragFactor = .5f;
    private float initialDragMultiplierX = 10f;
    private float handbrakeTime = 0f;
    private float handbrakeTimer = 1f;
    private Skidmarks _skidmarks;
    public float[] _skidmarktime;
    private ParticleEmitter skidSmoke;
    private float[] skidmarktime;
    private SoundController _soundController;
    private bool canSteer;
    private bool canDrive;
    private float accelerationTimer;
    public int wheelCount;


    void Awake()
    {
        _wheels = new Wheel[_frontWheels.Length + _rearWheels.Length];
        _soundController = GetComponent<SoundController>();
    }

    void Start()
    {
        accelerationTimer = Time.time;

        SetupWheelColliders();

        SetupCenterOfMass();

        _topSpeed = Convert_Miles_Per_Hour_To_Meters_Per_Second(_topSpeed);

        SetupGears();

        SetUpSkidmarks();

        initialDragMultiplierX = _dragMultiplier.x;
    }

    void Update()
    {
        var relativeVelocity = transform.InverseTransformDirection(GetComponent<Rigidbody>().velocity);

        GetInput();

        Check_If_Car_Is_Flipped();

        UpdateWheelGraphics(relativeVelocity);

        UpdateGear(relativeVelocity);
    }

    void FixedUpdate()
    {
//        // The rigidbody velocity is always given in world space, but in order to work in local space of the car model we need to transform it first.
        var relativeVelocity = transform.InverseTransformDirection(GetComponent< Rigidbody > ().velocity);

        CalculateState();

        UpdateFriction(relativeVelocity);

        UpdateDrag(relativeVelocity);

        CalculateEnginePower(relativeVelocity);

        ApplyThrottle(canDrive, relativeVelocity);

        ApplySteering(canSteer, relativeVelocity);
    }

    /**************************************************/
    /* Functions called from Start()                  */
    /**************************************************/
    void SetupCenterOfMass()
    {
        if (_centerOfMass != null)
            GetComponent<Rigidbody>().centerOfMass = _centerOfMass.localPosition;
    }

    void SetupWheelColliders()
    {
        SetupWheelFrictionCurve();

        var wheelCount = 0;

        for (int i = 0; i < _frontWheels.Length; i++)
        {
            _wheels[wheelCount] = SetupWheel(_frontWheels[i], true);
            wheelCount++;
        }

        for (int i = 0; i < _rearWheels.Length; i++)
        {
            _wheels[wheelCount] = SetupWheel(_rearWheels[i], true);
            wheelCount++;
        }
    }

    void SetupWheelFrictionCurve()
    {
        _wheelFrictionCurve = new WheelFrictionCurve
        {
            extremumSlip = 1,
            extremumValue = 50,
            asymptoteSlip = 2,
            asymptoteValue = 25,
            stiffness = 1
        };
    }

    Wheel SetupWheel(Transform wheelTransform, bool isFrontWheel)
    {
        var go = new GameObject(wheelTransform.name + " Collider");
        go.transform.position = wheelTransform.position;
        go.transform.parent = transform;
        go.transform.rotation = wheelTransform.rotation;

        var wc = go.AddComponent(typeof(WheelCollider)) as WheelCollider;
        wc.suspensionDistance = SuspensionRange;
        var js = wc.suspensionSpring;

        js.spring = isFrontWheel ? SuspensionSpringFront : SuspensionSpringRear;

        js.damper = SuspensionDamper;
        wc.suspensionSpring = js;

        var wheel = new Wheel {Collider = wc};
        wc.sidewaysFriction = _wheelFrictionCurve;
        wheel.wheelGraphic = wheelTransform;
        wheel.tireGraphic = wheelTransform.GetComponentsInChildren<Transform>()[1];

        var wheelRadius = wheel.tireGraphic.GetComponent<Renderer>().bounds.size.y / 2;
        wheel.Collider.radius = wheelRadius;

        if (isFrontWheel)
        {
            wheel.steerWheel = true;

            go = new GameObject(wheelTransform.name + " Steer Column");
            go.transform.position = wheelTransform.position;
            go.transform.rotation = wheelTransform.rotation;
            go.transform.parent = transform;
            wheelTransform.parent = go.transform;
        }
        else
            wheel.driveWheel = true;

        return wheel;
    }


    void SetupGears()
    {
        engineForceValues = new float[_numberOfGears];
        gearSpeeds = new float[_numberOfGears];

        var tempTopSpeed = _topSpeed;

        for (var i = 0; i < _numberOfGears; i++)
        {
            if (i > 0)
                gearSpeeds[i] = tempTopSpeed / 4 + gearSpeeds[i - 1];
            else
                gearSpeeds[i] = tempTopSpeed / 4;

            tempTopSpeed -= tempTopSpeed / 4;
        }

        var engineFactor = _topSpeed / gearSpeeds[gearSpeeds.Length - 1];

        for (var i = 0; i < _numberOfGears; i++)
        {
            var maxLinearDrag = gearSpeeds[i] * gearSpeeds[i];// * dragMultiplier.z;
            engineForceValues[i] = maxLinearDrag * engineFactor;
        }
    }

    void SetUpSkidmarks()
    {
        if (FindObjectOfType<Skidmarks>())
        {
            _skidmarks = FindObjectOfType<Skidmarks>();
            skidSmoke = _skidmarks.GetComponentInChildren<ParticleEmitter>();
        }
        else
            Debug.Log("No skidmarks object found. Skidmarks will not be drawn");

        skidmarktime = new[] {0f,0f,0f,0f};
    }

    /**************************************************/
    /* Functions called from Update()                 */
    /**************************************************/

    void GetInput()
    {
        _throttle = Input.GetAxis("Vertical");
        _steer = Input.GetAxis("Horizontal");

        if (_throttle < 0.0f)
            BrakeLights.SetFloat("_Intensity", Mathf.Abs(_throttle));
        else
            BrakeLights.SetFloat("_Intensity", 0.0f);

        CheckHandbrake();
    }

    void CheckHandbrake()
    {
        if (Input.GetKey("space"))
        {
            if (!_handbrake)
            {
                _handbrake = true;
                handbrakeTime = Time.time;
                _dragMultiplier.x = initialDragMultiplierX * handbrakeXDragFactor;
            }
        }
        else if (_handbrake)
        {
            _handbrake = false;
            StartCoroutine(StopHandbraking(Mathf.Min(5, Time.time - handbrakeTime)));
        }
    }

   IEnumerator StopHandbraking(float seconds)
    {
        var diff = initialDragMultiplierX - _dragMultiplier.x;
        handbrakeTimer = 1;

        // Get the x value of the dragMultiplier back to its initial value in the specified time.
        while (_dragMultiplier.x < initialDragMultiplierX && !_handbrake)
        {
            _dragMultiplier.x += diff * (Time.deltaTime / seconds);
            handbrakeTimer -= Time.deltaTime / seconds;
            yield return new WaitForSeconds(1);
        }

        _dragMultiplier.x = initialDragMultiplierX;
        handbrakeTimer = 0;
    }

    void Check_If_Car_Is_Flipped()
    {
        if (transform.localEulerAngles.z > 80 && transform.localEulerAngles.z < 280)
            _resetTimer += Time.deltaTime;
        else
            _resetTimer = 0;

        if (_resetTimer > _resetTime)
            FlipCar();
    }

    void FlipCar()
    {
        transform.rotation = Quaternion.LookRotation(transform.forward);
        transform.position += Vector3.up * 0.5f;
        GetComponent<Rigidbody>().velocity = Vector3.zero;
        GetComponent<Rigidbody>().angularVelocity = Vector3.zero;
        _resetTimer = 0;
        currentEnginePower = 0;
    }

    void UpdateWheelGraphics(Vector3 relativeVelocity)
    {
        wheelCount = -1;

        foreach (var w in _wheels)
        {
            wheelCount++;
            var wheel = w.Collider;
            var wh = new WheelHit();

            // First we get the velocity at the point where the wheel meets the ground, if the wheel is touching the ground
            if (wheel.GetGroundHit(out wh))
            {
                w.wheelGraphic.localPosition = wheel.transform.up * (_wheelRadius + wheel.transform.InverseTransformPoint(wh.point).y);
                w.wheelVelo = GetComponent<Rigidbody>().GetPointVelocity(wh.point);
                w.groundSpeed = w.wheelGraphic.InverseTransformDirection(w.wheelVelo);

                // Code to handle skidmark drawing. Not covered in the tutorial
                if (_skidmarks)
                {
                    if (_skidmarktime[wheelCount] < 0.02 && w.lastSkidmark != -1)
                    {
                        _skidmarktime[wheelCount] += Time.deltaTime;
                    }
                    else
                    {
                        var dt = _skidmarktime[wheelCount] == 0.0 ? Time.deltaTime : _skidmarktime[wheelCount];
                        _skidmarktime[wheelCount] = 0.0f;

                        var handbrakeSkidding = _handbrake && w.driveWheel ? w.wheelVelo.magnitude * 0.3f : 0f;
                        var skidGroundSpeed = Mathf.Abs(w.groundSpeed.x) - 15;
                        if (skidGroundSpeed > 0 || handbrakeSkidding > 0)
                        {
                            var staticVel = transform.TransformDirection(skidSmoke.localVelocity) + skidSmoke.worldVelocity;
                            if (w.lastSkidmark != -1)
                            {
                                var emission = UnityEngine.Random.Range(skidSmoke.minEmission, skidSmoke.maxEmission);
                                var lastParticleCount = w.lastEmitTime * emission;
                                var currentParticleCount = Time.time * emission;
                                var noOfParticles = Mathf.CeilToInt(currentParticleCount) - Mathf.CeilToInt(lastParticleCount);
                                var lastParticle = Mathf.CeilToInt(lastParticleCount);

                                for (var i = 0; i <= noOfParticles; i++)
                                {
                                    var particleTime = Mathf.InverseLerp(lastParticleCount, currentParticleCount, lastParticle + i);
                                    skidSmoke.Emit(Vector3.Lerp(w.lastEmitPosition, wh.point, particleTime) + new Vector3(Random.Range(-0.1f, 0.1f), Random.Range(-0.1f, 0.1f), Random.Range(-0.1f, 0.1f)), staticVel + (w.wheelVelo * 0.05f), Random.Range(skidSmoke.minSize, skidSmoke.maxSize) * Mathf.Clamp(skidGroundSpeed * 0.1f, 0.5f, 1f), Random.Range(skidSmoke.minEnergy, skidSmoke.maxEnergy), Color.white);
                                }
                            }
                            else
                            {
                                skidSmoke.Emit(wh.point + new Vector3(Random.Range(-0.1f, 0.1f), Random.Range(-0.1f, 0.1f), Random.Range(-0.1f, 0.1f)), staticVel + (w.wheelVelo * 0.05f), Random.Range(skidSmoke.minSize, skidSmoke.maxSize) * Mathf.Clamp(skidGroundSpeed * 0.1f, 0.5f, 1f), Random.Range(skidSmoke.minEnergy, skidSmoke.maxEnergy), Color.white);
                            }

                            w.lastEmitPosition = wh.point;
                            w.lastEmitTime = Time.time;

                            w.lastSkidmark = _skidmarks.AddSkidMark(wh.point + GetComponent< Rigidbody > ().velocity * dt, wh.normal, (skidGroundSpeed * 0.1f + handbrakeSkidding) * Mathf.Clamp01(wh.force / wheel.suspensionSpring.spring), w.lastSkidmark);
//                            _soundController.Skid Skid(true, Mathf.Clamp01(skidGroundSpeed * 0.1));
                        }
                        else
                        {
                            w.lastSkidmark = -1;
//                            _soundController.Skid(false, 0);
                        }
                    }
                }
            }
            else
            {
                // If the wheel is not touching the ground we set the position of the wheel graphics to
                // the wheel's transform position + the range of the suspension.
                w.wheelGraphic.position = wheel.transform.position + (-wheel.transform.up * SuspensionRange);
                if (w.steerWheel)
                    w.wheelVelo *= 0.9f;
                else
                    w.wheelVelo *= 0.9f * (1 - _throttle);

                if (_skidmarks)
                {
                    w.lastSkidmark = -1;
//                    _soundController.Skid(false, 0);
                }
            }
            // If the wheel is a steer wheel we apply two rotations:
            // *Rotation around the Steer Column (visualizes the steer direction)
            // *Rotation that visualizes the speed
            if (w.steerWheel)
            {
                var ea = w.wheelGraphic.parent.localEulerAngles;
                ea.y = _steer * maximumTurn;
                w.wheelGraphic.parent.localEulerAngles = ea;
                w.tireGraphic.transform.Rotate(Vector3.right * (w.groundSpeed.z / _wheelRadius) * Time.deltaTime * Mathf.Rad2Deg);
            }
            else if (!_handbrake && w.driveWheel)
            {
                // If the wheel is a drive wheel it only gets the rotation that visualizes speed.
                // If we are hand braking we don't rotate it.
                w.tireGraphic.transform.Rotate(Vector3.right * (w.groundSpeed.z / _wheelRadius) * Time.deltaTime * Mathf.Rad2Deg);
            }
        }
    }

    void UpdateGear(Vector3 relativeVelocity)
    {
        currentGear = 0;
        for (var i = 0; i < _numberOfGears - 1; i++)
        {
            if (relativeVelocity.z > gearSpeeds[i])
                currentGear = i + 1;
        }
    }

    /**************************************************/
    /* Functions called from FixedUpdate()            */
    /**************************************************/

    void UpdateDrag(Vector3 relativeVelocity)
    {
        var relativeDrag = new Vector3(-relativeVelocity.x * Mathf.Abs(relativeVelocity.x),
                                                    -relativeVelocity.y * Mathf.Abs(relativeVelocity.y),
                                                    -relativeVelocity.z * Mathf.Abs(relativeVelocity.z));

        var drag = Vector3.Scale(_dragMultiplier, relativeDrag);

        if (initialDragMultiplierX > _dragMultiplier.x) // Handbrake code
        {
            drag.x /= (relativeVelocity.magnitude / (_topSpeed / (1 + 2 * handbrakeXDragFactor)));
            drag.z *= (1 + Mathf.Abs(Vector3.Dot(GetComponent<Rigidbody> ().velocity.normalized, transform.forward)));
            drag += GetComponent< Rigidbody > ().velocity * Mathf.Clamp01(GetComponent< Rigidbody > ().velocity.magnitude / _topSpeed);
        }
        else // No handbrake
        {
            drag.x *= _topSpeed / relativeVelocity.magnitude;
        }

        if (Mathf.Abs(relativeVelocity.x) < 5 && !_handbrake)
            drag.x = -relativeVelocity.x * _dragMultiplier.x;


        GetComponent< Rigidbody > ().AddForce(transform.TransformDirection(drag) * GetComponent< Rigidbody > ().mass * Time.deltaTime);
    }

    void UpdateFriction(Vector3 relativeVelocity)
    {
        var sqrVel = relativeVelocity.x * relativeVelocity.x;

        // Add extra sideways friction based on the car's turning velocity to avoid slipping
        _wheelFrictionCurve.extremumValue = Mathf.Clamp(300 - sqrVel, 0, 300);
        _wheelFrictionCurve.asymptoteValue = Mathf.Clamp(150 - (sqrVel / 2), 0, 150);

        foreach  (var w in _wheels)
        {
            w.Collider.sidewaysFriction = _wheelFrictionCurve;
            w.Collider.forwardFriction = _wheelFrictionCurve;
        }
    }

    void CalculateEnginePower(Vector3 relativeVelocity)
    {
        if (_throttle == 0f)
        {
            currentEnginePower -= Time.deltaTime * 200;
        }
        else if (HaveTheSameSign(relativeVelocity.z, _throttle))
        {
            var normPower = (currentEnginePower / engineForceValues[engineForceValues.Length - 1]) * 2f;
            currentEnginePower += Time.deltaTime * 200 * EvaluateNormPower(normPower);
        }
        else
        {
            currentEnginePower -= Time.deltaTime * 300;
        }

        if (currentGear == 0)
            currentEnginePower = Mathf.Clamp(currentEnginePower, 0, engineForceValues[0]);
        else
            currentEnginePower = Mathf.Clamp(currentEnginePower, engineForceValues[currentGear - 1], engineForceValues[currentGear]);
    }

    void CalculateState()
    {
        canDrive = false;
        canSteer = false;

        foreach (var w in _wheels)
        {
            if (w.Collider.isGrounded)
            {
                if (w.steerWheel)
                    canSteer = true;
                if (w.driveWheel)
                    canDrive = true;
            }
        }
    }

    void ApplyThrottle(bool canDrive, Vector3 relativeVelocity)
    {
        if (canDrive)
        {
            var throttleForce = 0f;
            var brakeForce = 0f;

            if (HaveTheSameSign(relativeVelocity.z, _throttle))
            {
                if (!_handbrake)
                    throttleForce = Mathf.Sign(_throttle) * currentEnginePower * GetComponent< Rigidbody > ().mass;
            }
            else
                brakeForce = Mathf.Sign(_throttle) * engineForceValues[0] * GetComponent< Rigidbody > ().mass;

            GetComponent< Rigidbody > ().AddForce(transform.forward * Time.deltaTime * (throttleForce + brakeForce));
        }
    }

    void ApplySteering(bool canSteer, Vector3 relativeVelocity)
    {
        if (canSteer)
        {
            var turnRadius = 3.0f / Mathf.Sin((90 - (_steer * 30)) * Mathf.Deg2Rad);
            var minMaxTurn = EvaluateSpeedToTurn(GetComponent< Rigidbody > ().velocity.magnitude);
            var turnSpeed = Mathf.Clamp(relativeVelocity.z / turnRadius, -minMaxTurn / 10f, minMaxTurn / 10f);

            transform.RotateAround(transform.position + transform.right * turnRadius * _steer,
                                    transform.up,
                                    turnSpeed * Mathf.Rad2Deg * Time.deltaTime * _steer);

            var debugStartPoint = transform.position + transform.right * turnRadius * _steer;
            var debugEndPoint = debugStartPoint + Vector3.up * 5;

            Debug.DrawLine(debugStartPoint, debugEndPoint, Color.red);

            if (initialDragMultiplierX > _dragMultiplier.x) // Handbrake
            {
                var rotationDirection = Mathf.Sign(_steer); // rotationDirection is -1 or 1 by default, depending on steering
                if (_steer == 0)
                {
                    if (GetComponent< Rigidbody > ().angularVelocity.y < 1) // If we are not steering and we are handbraking and not rotating fast, we apply a random rotationDirection
                        rotationDirection = Random.Range(-1.0f, 1.0f);
                    else
                        rotationDirection = GetComponent< Rigidbody > ().angularVelocity.y; // If we are rotating fast we are applying that rotation to the car
                }
                // -- Finally we apply this rotation around a point between the cars front wheels.
                transform.RotateAround(transform.TransformPoint((_frontWheels[0].localPosition + _frontWheels[1].localPosition) * 0.5f),
                                                                    transform.up,
                                                                    GetComponent< Rigidbody > ().velocity.magnitude * Mathf.Clamp01(1 - GetComponent< Rigidbody > ().velocity.magnitude / _topSpeed) * rotationDirection * Time.deltaTime * 2);
            }
        }
    }
    /**************************************************/
    /*               Utility Functions                */
    /**************************************************/

    float Convert_Miles_Per_Hour_To_Meters_Per_Second(float value)
{
	return value* 0.44704f;
}

float Convert_Meters_Per_Second_To_Miles_Per_Hour(float value)
{
	return value* 2.23693629f;	
}

bool HaveTheSameSign(float first, float second)
{
    return Mathf.Sign(first) == Mathf.Sign(second);
}

    float EvaluateSpeedToTurn(float speed)
{
    if (speed > _topSpeed / 2)
        return minimumTurn;

    var speedIndex = 1f - (speed / (_topSpeed / 2));
    return minimumTurn + speedIndex * (maximumTurn - minimumTurn);
}

float EvaluateNormPower(float normPower)
{
    if (normPower < 1)
        return 10f - normPower * 9f;
    else
        return 1.9f - normPower * 0.9f;
}

    float GetGearState()
{
    var relativeVelocity = transform.InverseTransformDirection(GetComponent<Rigidbody>().velocity);
    var lowLimit = (currentGear == 0 ? 0 : gearSpeeds[currentGear - 1]);
    return (relativeVelocity.z - lowLimit) / (gearSpeeds[(int) (currentGear - lowLimit)]) * (1 - currentGear * 0.1f) + currentGear * 0.1f;
}
}
