using System;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using UnityEngine;
using System.Collections;
using UnityEngine.Animations;
using UnityEditor;


public class OPRSHandTracker : MonoBehaviour
{
#if UNITY_EDITOR
    
    // Handle to the C++ DLL
    public IntPtr libraryHandle;

    //#########################################################
    //##### One Delegate combo per DLL-function please ######## (excluding the dummy function!)
    //#########################################################
    public delegate int Init_Delegate();   // func 1
    public Init_Delegate Init;

    public delegate float MonoUpdate_Delegate(); // func 2
    public MonoUpdate_Delegate MonoUpdate;

    public delegate IntPtr ImageUpdate_Delegate(); // func 3
    public ImageUpdate_Delegate ImageUpdate;

    public delegate void Shutdown_Delegate(); // func 4
    public Shutdown_Delegate Shutdown;



#endif

#if UNITY_EDITOR_OSX || UNITY_EDITOR_LINUX
 
	[DllImport("__Internal")]
	public static extern IntPtr dlopen(
		string path,
		int flag);
 
	[DllImport("__Internal")]
	public static extern IntPtr dlsym(
		IntPtr handle,
		string symbolName);
 
	[DllImport("__Internal")]
	public static extern int dlclose(
		IntPtr handle);
 
	public static IntPtr OpenLibrary(string path)
	{
		IntPtr handle = dlopen(path, 0);
		if (handle == IntPtr.Zero)
		{
			throw new Exception("Couldn't open native library: " + path);
		}
		return handle;
	}
 
	public static void CloseLibrary(IntPtr libraryHandle)
	{
		dlclose(libraryHandle);
	}
 
	public static T GetDelegate<T>(
		IntPtr libraryHandle,
		string functionName) where T : class
	{
		IntPtr symbol = dlsym(libraryHandle, functionName);
		if (symbol == IntPtr.Zero)
		{
			throw new Exception("Couldn't get function: " + functionName);
		}
		return Marshal.GetDelegateForFunctionPointer(
			symbol,
			typeof(T)) as T;
	}
 
 


#elif UNITY_EDITOR_WIN // THIS WE WANT TO USE ;^)

    // Import dll code, no need to change

    [DllImport("kernel32")]
    public static extern IntPtr LoadLibrary(string path);

    [DllImport("kernel32")]
    public static extern IntPtr GetProcAddress(IntPtr libraryHandle, string symbolName);


    [DllImport("kernel32")]
    public static extern bool FreeLibrary(IntPtr libraryHandle);

    public static IntPtr OpenLibrary(string path)       // <---- Right now it fails here!
    {
        IntPtr handle = LoadLibrary(path);
        if (handle == IntPtr.Zero)
        {
            throw new Exception("Couldn't open native library: " + path);
        }
        return handle;
    }

    public static void CloseLibrary(IntPtr libraryHandle)
    {
        FreeLibrary(libraryHandle);
    }

    public static T GetDelegate<T>(IntPtr libraryHandle, string functionName) where T : class
    {
        IntPtr symbol = GetProcAddress(libraryHandle, functionName);
        if (symbol == IntPtr.Zero)
        {
            throw new Exception("Couldn't get function: " + functionName);
        }
        return Marshal.GetDelegateForFunctionPointer(symbol, typeof(T)) as T;
    }

    //[DllImport("DLLTESTER", EntryPoint = "rs2_test_function")]
    //public static extern int rs2_test_function(int h);

    [DllImport("beta", EntryPoint = "dummy_function")]
    public static extern int dummy_function(int h);


#else
    // when EXE put all functions here.
#endif


#if UNITY_EDITOR_OSX
	const string LIB_PATH = "/NativeScript.bundle/Contents/MacOS/NativeScript";
#elif UNITY_EDITOR_LINUX
	const string LIB_PATH = "/NativeScript.so";
#elif UNITY_EDITOR_WIN
    const string LIB_PATH = "/Plugins/beta.dll";  // path to the dll in Assets
#endif

    /// <summary>
    /// GLOBAL VARIABLES
    /// </summary>
    /// 
    public List<GameObject> fingerBallsInScene = new List<GameObject>();
    public Vector3 dll_xyz, dll_xyz_tumme; //-new (2/7)
    public Vector3 dll_xyz_mid;
    float update;
    // C++ Finger Positions
    protected float[] FINGER_POINTS = new float[15];//[6];
    protected Vector3 P_sum, P_centroid, myNormal;
    private GameObject myPlane;
    protected GameObject collidedObj;
    ConstraintSource constraintSource;
    Vector3 CF_pos, LF_pos, CF_normal, LF_normal;
    GameObject dummyGO, legoman;
    Matrix4x4 CF_pointSet = default;
    Matrix4x4 LF_pointSet = default;

    Vector3 LF_relativePos, fromFingerToObject;
    Vector3[] LF_StartEnd = new Vector3[2];

    [SerializeField]
    [Range(0.001f, 0.4f)]
    private float UnstickThreashold = 0.05f;

    public bool youMayRead = false;

    //################ UNITY START() ################################
    void Start()
    {
        // Yet another neccessary step in the linking process ^^ ------------------------------------|
#if UNITY_EDITOR

        //==============
        // ====================
        // Let him do this. UNITY FAIL OTHERWISE
        dummy_function(0);//====================
                          // ====================
                          //==============

        // ## Open native library
        libraryHandle = OpenLibrary(Application.dataPath + LIB_PATH);
        Debug.Log("lib handle?? ->" + libraryHandle);

        // ## Assign the delegate pointer whatever from libraryHandel to the dll-func name. ##

        Init = GetDelegate<Init_Delegate>(libraryHandle, "Init");   // func 1

        MonoUpdate = GetDelegate<MonoUpdate_Delegate>(libraryHandle, "MonoUpdate"); // func 2

        ImageUpdate = GetDelegate<ImageUpdate_Delegate>(libraryHandle, "ImageUpdate");  // func 3

        Shutdown = GetDelegate<Shutdown_Delegate>(libraryHandle, "Shutdown");    // func 4

#endif
        // -------------------------------------------------------------------------------------------|
        // Go ahead! Play with your C++ functions.
   
        Debug.Log("init: " + Init());



        // C# CITY
        P_sum = Vector3.zero;
        CF_pos = Vector3.zero;
        LF_pos = CF_pos;
        CF_normal = Vector3.zero;
        LF_normal = Vector3.zero;
        LF_pointSet = Matrix4x4.identity;
        CF_pointSet = Matrix4x4.identity;
        dummyGO = new GameObject("dummy");
        LF_StartEnd[0] = Vector3.zero;
        LF_StartEnd[1] = Vector3.zero;


        foreach (GameObject go in Resources.FindObjectsOfTypeAll(typeof(GameObject)) as GameObject[])
        {
            if (!EditorUtility.IsPersistent(go.transform.root.gameObject) && !(go.hideFlags == HideFlags.NotEditable || go.hideFlags == HideFlags.HideAndDontSave))
            {
                if (go.tag.Equals("fingerBall"))
                {
                    fingerBallsInScene.Add(go);
                    Debug.Log("Interactable gameObject Added to List!: " + go.name);
                }

            }

        }

    }


    void Update()
    {
        update = MonoUpdate();
        Marshal.Copy(ImageUpdate(), FINGER_POINTS, 0, FINGER_POINTS.Length);
        // Pekfinger
        dll_xyz.x = FINGER_POINTS[0];
        dll_xyz.y = FINGER_POINTS[1];
        dll_xyz.z = FINGER_POINTS[2];
        dll_xyz.z *= -1;

        //print("X:" + dll_xyz.x + "| Y:" + dll_xyz.y + "| Z:" + dll_xyz.z);

        // Tumme
        dll_xyz_tumme.x = FINGER_POINTS[3];
        dll_xyz_tumme.y = FINGER_POINTS[4];
        dll_xyz_tumme.z = FINGER_POINTS[5];
        dll_xyz_tumme.z *= -1;

        // ====[][] Part of the TriPlane Magic [][]========================================== v (6/7)
        dll_xyz_mid.x = FINGER_POINTS[6];
        dll_xyz_mid.y = FINGER_POINTS[7];
        dll_xyz_mid.z = FINGER_POINTS[8];
        dll_xyz_mid.z *= -1;

        //print("PEKxyz:"+ dll_xyz + "TUMxyz:" + dll_xyz_tumme + "MIDxyz:" + dll_xyz_mid );

        /// Vart ska bollarna enligt beta.dll?? 
        /// ---------------------------------------------------------------------
        // Pekfinger
        fingerBallsInScene[1].transform.position = dll_xyz;
        // Tumme 1 & 2
        fingerBallsInScene[2].transform.position = dll_xyz_tumme;
        fingerBallsInScene[0].transform.position = dll_xyz_mid;
        /// ---------------------------------------------------------------------
        /// 

        CF_pos = transform.position;
        if (Vector3.Distance(CF_pos, LF_pos) > UnstickThreashold)
        {
            NoMoreParent();
        }
        LF_pos = CF_pos;

        foreach (GameObject fb in fingerBallsInScene)
        {
            //print("fb:" + fb.transform.position);
            P_sum += fb.transform.position;

        }

        Plane myTriangle = new Plane(fingerBallsInScene[0].transform.position, fingerBallsInScene[1].transform.position, fingerBallsInScene[2].transform.position);
        P_centroid = P_sum / 3;

        CF_normal = myTriangle.normal.normalized;

        Debug.DrawLine(fingerBallsInScene[0].transform.position, fingerBallsInScene[1].transform.position, Color.yellow);
        Debug.DrawLine(fingerBallsInScene[1].transform.position, fingerBallsInScene[2].transform.position, Color.yellow);
        Debug.DrawLine(fingerBallsInScene[2].transform.position, fingerBallsInScene[0].transform.position, Color.yellow);
        Debug.DrawLine(P_centroid, P_centroid + myTriangle.normal.normalized * 0.1f, Color.blue);

        var theNormVector = CalcNormalVector(fingerBallsInScene[1].transform.position, fingerBallsInScene[2].transform.position,
                                                fingerBallsInScene[0].transform.position);
        Vector3 CF_relativePos = fingerBallsInScene[1].transform.position - P_centroid;

        Vector3 baseNormal = LF_normal;
        Vector3 activeNormal = CalcNormalVector(LF_StartEnd[0], LF_StartEnd[1], fingerBallsInScene[1].transform.position);

        var LeftSide_or_RightSide = Vector3.Dot(activeNormal, baseNormal);//If Ans >= 0 -> Left | Else -> Right
        var Better_CF_relativePos = Vector3.ProjectOnPlane(CF_relativePos,LF_normal);

        var angleAroundNormal = Vector3.Angle(LF_relativePos, Better_CF_relativePos);

        if (LeftSide_or_RightSide < 0)
        {
            angleAroundNormal *= -1f;
        }

        LF_StartEnd[0] = P_centroid;


        CF_normal = myTriangle.normal.normalized;

        Quaternion rot1 = Quaternion.FromToRotation(LF_normal, CF_normal);
        Quaternion rot2 = Quaternion.AngleAxis(angleAroundNormal, myTriangle.normal.normalized);


        ///[[[ Transform the Object of Interest ]]]
        if (collidedObj)
        {
            //[Translate The Object]-------------
            collidedObj.transform.position = CF_pos + (fromFingerToObject);//P_centroid + myTriangle.normal.normalized * 0.06f;//CF_pos + ( fromFingerToObject );// + myTriangle.normal.normalized * 0.06f;//CF_pos;//CF_pos + (CF_pos - collidedObj.transform.position);//myTriangle.normal.normalized * 0.06f;
            //[xxxxxxxxxxxxxxxxxxxx]-------------

            if (fingerBallsInScene[1].transform.position != LF_StartEnd[1])
            {
                collidedObj.transform.Rotate(rot1.eulerAngles, Space.World);
                var orthoOrNot = Vector3.Dot(LF_normal, (fingerBallsInScene[1].transform.position - P_centroid));
                print("DOT: " + orthoOrNot);
                collidedObj.transform.Rotate(rot2.eulerAngles, Space.World);

                if (System.Math.Round(orthoOrNot,2) == 0.00f) // If normals between frames are paralell within Range[] Then apply 2nd rotation.
                {
                   
                }
            }

        }
        ///[[[ Update what's OLD ]]]
        LF_normal = CF_normal;
        LF_relativePos = CF_relativePos;
        LF_StartEnd[1] = fingerBallsInScene[1].transform.position;
        P_sum = Vector3.zero;
        LF_pointSet = CF_pointSet;
    }

    void OnCollisionEnter(Collision _col)
    {
        collidedObj = _col.gameObject;
        collidedObj.GetComponent<Rigidbody>().isKinematic = true;
        fromFingerToObject = collidedObj.transform.position - CF_pos;
    }

    //                 _________TT_
    //                /____________\    
    // FUNCTION-VILLE  |   |-|  0 |
    private Vector3 GetMeanVector(List<Vector3> positions)
    {
        if (positions.Count == 0)
        {
            return Vector3.zero;
        }

        Vector3 meanVector = Vector3.zero;

        foreach (Vector3 pos in positions)
        {
            meanVector += pos;
        }

        return (meanVector / positions.Count);
    }

    public Vector3 CalcNormalVector(Vector3 a, Vector3 b, Vector3 c)
    {
        var side1 = b - a;
        var side2 = c - a;

        var perp = Vector3.Cross(side1, side2);
        var perpLength = perp.magnitude;
        perp /= perpLength;

        return perp;
    }

    private void NoMoreParent()
    {
        Debug.Log("NoMoreParent HAS BEEN CALLED!");
        if (collidedObj != null)
        {
            collidedObj.GetComponent<Rigidbody>().isKinematic = false;
            collidedObj = null;
        }
    }


    void OnApplicationQuit()
    {
#if UNITY_EDITOR
        Shutdown();
        CloseLibrary(libraryHandle);
        libraryHandle = IntPtr.Zero;
#endif

    }



}
