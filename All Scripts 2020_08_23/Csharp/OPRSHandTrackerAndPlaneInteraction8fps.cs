using System;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using UnityEngine;
using System.Collections;
using UnityEngine.Animations;
using UnityEditor;

/// <summary>
///  Btw, rs2::alignment reduces the framerate. ex. We put 60fps but got 29fps.
/// </summary>

public class OPRSHandTrackerAndPlaneInteraction8fps : MonoBehaviour
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
    const string DLL_NAME = "beta_CAM1";
    [DllImport(DLL_NAME, EntryPoint = "dummy_function")]
    public static extern int dummy_function(int h);


#else
    // when EXE put all functions here.
#endif


#if UNITY_EDITOR_OSX
	const string LIB_PATH = "/NativeScript.bundle/Contents/MacOS/NativeScript";
#elif UNITY_EDITOR_LINUX
	const string LIB_PATH = "/NativeScript.so";
#elif UNITY_EDITOR_WIN
    const string LIB_PATH = "/Plugins/" + DLL_NAME + ".dll";  // path to the dll in Assets
#endif

    /// <summary>
    /// GLOBAL VARIABLES
    /// </summary>
    /// 

    //For threading tasks
    Vector3 dll_xyz_copy;

    protected List<GameObject> fingerBallsInScene = new List<GameObject>();
    protected List<GameObject> objectsInScene = new List<GameObject>();
    protected List<Vector3> vector3List_PEK = new List<Vector3>();
    protected List<Vector3> vector3List_TUM = new List<Vector3>();
    protected List<Vector3> vector3List_MID = new List<Vector3>();
    protected Vector3 dll_xyz, dll_xyz_tumme; //-new (2/7)
    protected Vector3 dll_xyz_mid;
    float update;
    // C++ Finger Positions
    protected float[] FINGER_POINTS = new float[16];//[6];
    protected Vector3 P_sum, P_centroid, myNormal;
    private GameObject myPlane;
    protected GameObject collidedObj;
    ConstraintSource constraintSource;
    Vector3 CF_pos, LF_pos, CF_normal, LF_normal;
    GameObject dummyGO, legoman, _index, _thumb, _middle;
    Matrix4x4 CF_pointSet = default;
    Matrix4x4 LF_pointSet = default;
    GameObject canvasLeft, canvasRight;
    int counter;


    Vector3 LF_relativePos, fromFingerToObject;
    Vector3[] LF_StartEnd = new Vector3[2];

    [SerializeField]
    [Range(0.001f, 0.4f)]
    protected float UnstickThreashold = 0.05f;

    [SerializeField]
    [Range(1, 15)]
    protected int listLimit = 5;

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
                if (go.tag.Equals("index"))
                {
                    _index = go;
                    Debug.Log("Interactable gameObject Added to List!: " + go.name);
                }
                if (go.tag.Equals("thumb"))
                {
                    _thumb = go;
                    Debug.Log("Interactable gameObject Added to List!: " + go.name);
                }
                if (go.tag.Equals("middle"))
                {
                    _middle = go;
                    Debug.Log("Interactable gameObject Added to List!: " + go.name);
                }
                if (go.tag.Equals("canvasLeft"))
                {
                    canvasLeft = go;
                    Debug.Log("Interactable gameObject Added to List!: " + go.name);
                }
                if (go.tag.Equals("canvasRight"))
                {
                    canvasRight = go;
                    Debug.Log("Interactable gameObject Added to List!: " + go.name);
                }
                if (go.tag.Equals("petamig"))
                {
                    objectsInScene.Add(go);
                    Debug.Log("Interactable gameObject Added to List!: " + go.name);
                }

            }

        }


        fingerBallsInScene.Add(_middle);
        fingerBallsInScene.Add(_index);
        fingerBallsInScene.Add(_thumb);
    }


    void Update()
    {
        update = MonoUpdate();
        Marshal.Copy(ImageUpdate(), FINGER_POINTS, 0, FINGER_POINTS.Length);

        // Pekfinger
        dll_xyz.x = FINGER_POINTS[0];
        dll_xyz.y = FINGER_POINTS[1];
        dll_xyz.z = FINGER_POINTS[2];
        dll_xyz.x *= -1;
        dll_xyz_copy = dll_xyz;
        //print("X:" + dll_xyz.x + "| Y:" + dll_xyz.y + "| Z:" + dll_xyz.z);

        // Tumme
        dll_xyz_tumme.x = FINGER_POINTS[3];
        dll_xyz_tumme.y = FINGER_POINTS[4];
        dll_xyz_tumme.z = FINGER_POINTS[5];
        dll_xyz_tumme.x *= -1;


        // ====[][] Part of the TriPlane Magic [][]========================================== v (6/7)
        dll_xyz_mid.x = FINGER_POINTS[6];
        dll_xyz_mid.y = FINGER_POINTS[7];
        dll_xyz_mid.z = FINGER_POINTS[8];
        dll_xyz_mid.x *= -1;

        /*       
               Vector3 p0, p1, p2;
               p0.x = FINGER_POINTS[6];
               p0.y = FINGER_POINTS[7];
               p0.z = FINGER_POINTS[8];

               p1.x = FINGER_POINTS[9];
               p1.y = FINGER_POINTS[10];
               p1.z = FINGER_POINTS[11];

               p2.x = FINGER_POINTS[12];
               p2.y = FINGER_POINTS[13];
               p2.z = FINGER_POINTS[14];

               print("P..0-> [ X:" + p0.x + "| Y:" + p0.y + "| Z:" + p0.z + "]");
               print("P__1-> [ X:" + p1.x + "| Y:" + p1.y + "| Z:" + p1.z + "]");
               print("P**2-> [ X:" + p2.x + "| Y:" + p2.y + "| Z:" + p2.z + "]");
       */

        //################ BROKEN TRACKING OR NOT?? ##########################
        if (FINGER_POINTS[15] == 1.0f)
        {
            counter++;
            if (counter > 5)
            {
                if (!canvasLeft.GetComponent<Canvas>().enabled)
                {
                    canvasLeft.GetComponent<Canvas>().enabled = true;
                    canvasRight.GetComponent<Canvas>().enabled = true;

                    foreach (GameObject GO in objectsInScene)
                    {
                        GO.GetComponent<MeshRenderer>().enabled = false;
                    }
                }
                counter = 0;
            }



        }
        else
        {
            if (canvasLeft.GetComponent<Canvas>().enabled)
            {
                counter = 0;
                foreach (GameObject GO in objectsInScene)
                {
                    GO.GetComponent<MeshRenderer>().enabled = true;
                }
                canvasLeft.GetComponent<Canvas>().enabled = false;
                canvasRight.GetComponent<Canvas>().enabled = false;
            }
        }
        /*
        // ============[ Get Mean Position ]
        // PEK
        if (vector3List_PEK.Count < listLimit)
            vector3List_PEK.Add(dll_xyz);
        else
        {
            vector3List_PEK.RemoveAt(0);
            vector3List_PEK.Add(dll_xyz);
        }
        var vec_sum = Vector3.zero;
        foreach (Vector3 vec in vector3List_PEK)
        {
            vec_sum += vec;
        }
        dll_xyz = vec_sum / vector3List_PEK.Count;
        // _____TUM
        if (vector3List_TUM.Count < listLimit)
            vector3List_TUM.Add(dll_xyz_tumme);
        else
        {
            vector3List_TUM.RemoveAt(0);
            vector3List_TUM.Add(dll_xyz_tumme);
        }
        vec_sum = Vector3.zero;
        foreach (Vector3 vec in vector3List_TUM)
        {
            vec_sum += vec;
        }
        dll_xyz_tumme = vec_sum / vector3List_TUM.Count;
        // ____ M I D _____
        if (vector3List_MID.Count < listLimit)
            vector3List_MID.Add(dll_xyz_mid);
        else
        {
            vector3List_MID.RemoveAt(0);
            vector3List_MID.Add(dll_xyz_mid);
        }
        vec_sum = Vector3.zero;
        foreach (Vector3 vec in vector3List_MID)
        {
            vec_sum += vec;
        }
        dll_xyz_mid = vec_sum / vector3List_MID.Count;

    */

        // The new way to get Mean position.
        Add_Pos_To_List(vector3List_PEK, dll_xyz, listLimit);
        Add_Pos_To_List(vector3List_TUM, dll_xyz_tumme, listLimit);
        Add_Pos_To_List(vector3List_MID, dll_xyz_mid, listLimit);
        dll_xyz = Get_Weighted_Mean(vector3List_PEK);
        dll_xyz_tumme = Get_Weighted_Mean(vector3List_TUM);
        dll_xyz_mid = Get_Weighted_Mean(vector3List_MID);

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
        CF_pos = dll_xyz_copy;

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
        Debug.DrawLine(P_centroid, P_centroid + myTriangle.normal.normalized * 0.1f, Color.cyan);

        var theNormVector = CalcNormalVector(fingerBallsInScene[1].transform.position, fingerBallsInScene[2].transform.position,
                                                fingerBallsInScene[0].transform.position);
        Vector3 CF_relativePos = fingerBallsInScene[1].transform.position - P_centroid;

        Vector3 baseNormal = LF_normal;
        Vector3 activeNormal = CalcNormalVector(LF_StartEnd[0], LF_StartEnd[1], fingerBallsInScene[1].transform.position);

        var LeftSide_or_RightSide = Vector3.Dot(activeNormal, baseNormal);//If Ans >= 0 -> Left | Else -> Right
        var Better_CF_relativePos = Vector3.ProjectOnPlane(CF_relativePos, LF_normal);

        var angleAroundNormal = Vector3.Angle(LF_relativePos, Better_CF_relativePos);

        if (LeftSide_or_RightSide < 0)
        {
            angleAroundNormal *= -1f;
        }

        LF_StartEnd[0] = P_centroid;


        CF_normal = myTriangle.normal.normalized;

        Quaternion rot1 = Quaternion.FromToRotation(LF_normal, CF_normal);
        Quaternion rot2 = Quaternion.AngleAxis(angleAroundNormal, myTriangle.normal.normalized);


        Vector3 MIDtoPEK = fingerBallsInScene[1].transform.position - fingerBallsInScene[0].transform.position;
        Vector3 towardsPalm = Vector3.Cross(CF_normal, MIDtoPEK);
        towardsPalm = -towardsPalm.normalized * (Vector3.Distance(fingerBallsInScene[1].transform.position, fingerBallsInScene[0].transform.position) * 0.45f);

        ///[ Transform the Object of Interest ]
        if (collidedObj)
        {
            //[Translate The Object]-------------
            ///collidedObj.transform.position = CF_pos + (fromFingerToObject);
            collidedObj.transform.position = P_centroid + myTriangle.normal.normalized * 0.06f + towardsPalm;

            //[Rotate The Object]-------------
            if (collidedObj || fingerBallsInScene[1].transform.position != LF_StartEnd[1])
            {
                collidedObj.transform.Rotate(rot1.eulerAngles, Space.World);
                var orthoOrNot = Vector3.Dot(LF_normal, (fingerBallsInScene[1].transform.position - P_centroid));
                //print("DOT: " + orthoOrNot);
                collidedObj.transform.Rotate(rot2.eulerAngles, Space.World);
            }
        }

        ///[ Update what's OLD ]
        LF_normal = CF_normal;
        LF_relativePos = CF_relativePos;
        LF_StartEnd[1] = fingerBallsInScene[1].transform.position;
        P_sum = Vector3.zero;
        LF_pointSet = CF_pointSet;
    }

    void OnCollisionEnter(Collision _col)
    {
        if (!collidedObj)
        {
            collidedObj = _col.gameObject;
            collidedObj.GetComponent<Rigidbody>().isKinematic = true;
            fromFingerToObject = collidedObj.transform.position - CF_pos;
        }

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


    private Vector3 Get_Weighted_Mean(List<Vector3> pList)
    {
        if (pList.Count == 0)
        {
            return Vector3.zero;
        }

        float w_old = 0.25f;
        float w_new = 1.75f;
        Vector3 _sum = Vector3.zero;

        foreach (Vector3 pos in pList)
        {
            _sum += pos;
        }
        _sum -= pList[pList.Count - 1];
        _sum -= pList[0];
        _sum += w_new * pList[pList.Count - 1];
        _sum += w_old * pList[0];

        return (_sum / pList.Count);
    }

    private void Add_Pos_To_List(List<Vector3> pastPositions, Vector3 newPos, int desired_size)
    {
        if (pastPositions.Count < desired_size)
            pastPositions.Add(newPos);
        else
        {
            pastPositions.RemoveAt(0);
            pastPositions.Add(newPos);
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