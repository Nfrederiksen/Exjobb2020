using System;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Animations;
using UnityEditor;
using System.Threading.Tasks;
/// <summary>
///  Btw, rs2::alignment reduces the framerate. ex. We put 60fps but got 29fps.
/// </summary>

public class OPRSHandTrackerAndPlaneInteraction16fps : MonoBehaviour
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

    ///          .------------------------------.
    /// #########|       GLOBAL VARIABLES       | ###################################################
    ///          `-----------------------------'
    
    //For threading tasks
    Task<float> OPTask;
    Vector3 dll_xyz_copy;

    protected List<GameObject> fingerBallsInScene = new List<GameObject>();
    protected List<GameObject> objectsInScene = new List<GameObject>();
    protected List<Vector3> vector3List_PEK = new List<Vector3>();
    protected List<Vector3> vector3List_TUM = new List<Vector3>();
    protected List<Vector3> vector3List_MID = new List<Vector3>();
    protected Vector3 dll_xyz, dll_xyz_tumme; //-new (2/7)
    protected Vector3 dll_xyz_mid;
    float update, YAW, PITCH;
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
    bool enableOffset = false;

    Vector3 LF_relativePos, fromFingerToObject;
    Vector3[] LF_StartEnd = new Vector3[2];

    [SerializeField]
    [Range(0.001f, 0.4f)]
    protected float UnstickThreashold = 0.066f;

    [SerializeField]
    [Range(1, 15)]
    protected int listLimit = 8;

    public bool released = true;

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

        // ===[ Find all relavant GameObjects in scene and assign them to variables. ]
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
                if (go.tag.Equals("reset_button"))
                {
                    objectsInScene.Add(go);
                    Debug.Log("Interactable gameObject Added to List!: " + go.name);
                }
            }
        }
        
        fingerBallsInScene.Add(_middle);
        fingerBallsInScene.Add(_index);
        fingerBallsInScene.Add(_thumb);
        
        update = MonoUpdate();
        OPTask = Task<float>.Factory.StartNew(() => FingerUpdate());
        print("OPTask started: " + OPTask.Result);
        
    }

    
    void Update()
    {
        // ===[ START USING THE DLLs FUNCTIONS ]
        update = MonoUpdate();
        
        if (OPTask.IsCompleted)
        {
            OPTask.Dispose();
            OPTask = Task<float>.Factory.StartNew(() => FingerUpdate());
        }
        
        // ===[ READ the estimated finger positions from dll array ]
        // (Pekfinger)
        dll_xyz.x = FINGER_POINTS[0];
        dll_xyz.y = FINGER_POINTS[1];
        dll_xyz.z = FINGER_POINTS[2];
        dll_xyz.x *= -1;
        dll_xyz_copy = dll_xyz;
        //print("X:" + dll_xyz.x + "| Y:" + dll_xyz.y + "| Z:" + dll_xyz.z);

        // (Tumme)
        dll_xyz_tumme.x = FINGER_POINTS[3];
        dll_xyz_tumme.y = FINGER_POINTS[4];
        dll_xyz_tumme.z = FINGER_POINTS[5];
        dll_xyz_tumme.x *= -1;


        // (Mid/ Under Tumme)
        dll_xyz_mid.x = FINGER_POINTS[6];
        dll_xyz_mid.y = FINGER_POINTS[7];
        dll_xyz_mid.z = FINGER_POINTS[8];
        dll_xyz_mid.x *= -1;

        /*       
         // [ USED FOR: T matrix calibration process. ]
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

        // ===[ FEEDBACK FOR LOST HAND TRACKING! ]
        if (FINGER_POINTS[15] == 1.0f) // if the tracking is lost 
        {
            // release the object
            NoMoreParent();
            // Pekfinger
            fingerBallsInScene[1].transform.position = new Vector3(-0.025f, 0.169f, 0.15f);
            // Tumme 1 & 2
            fingerBallsInScene[2].transform.position = new Vector3(-0.020f, 0.150f, 0.15f); 
            fingerBallsInScene[0].transform.position = new Vector3(-0.020f, 0.150f, 0.15f);

            counter++;
            if(counter > 5)
            {

                if (!canvasLeft.GetComponent<Canvas>().enabled)
                {
                    canvasLeft.GetComponent<Canvas>().enabled = true;
                    canvasRight.GetComponent<Canvas>().enabled = true;

                    foreach (GameObject GO in objectsInScene)
                    {
                        GO.GetComponent<MeshRenderer>().enabled = false;
                        //GO.GetComponent<Renderer>().material.color = new Color(0f, 0f, 0f, 0.5f);
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
        
        // ======[ Get Mean Position ]
        // The new way to get Mean position.
        Add_Pos_To_List(vector3List_PEK, dll_xyz, listLimit); // medel pekfinger
        Add_Pos_To_List(vector3List_TUM, dll_xyz_tumme, listLimit); // medel tumme
        Add_Pos_To_List(vector3List_MID, dll_xyz_mid, listLimit); // medel tumme 2

        // if no object don't use mean
        if (collidedObj)
        {
            dll_xyz = Get_Weighted_Mean(vector3List_PEK);
            dll_xyz_tumme = Get_Weighted_Mean(vector3List_TUM);
            dll_xyz_mid = Get_Weighted_Mean(vector3List_MID);
        }
        
        if (FINGER_POINTS[15] == 0.0f)
        {
            // ====[ Where should the Finger spheres go according to the DLL? ]
            /// ---------------------------------------------------------------------
            // Pekfinger
            fingerBallsInScene[1].transform.position = dll_xyz;
            // Tumme 1 & 2
            fingerBallsInScene[2].transform.position = dll_xyz_tumme;
            fingerBallsInScene[0].transform.position = dll_xyz_mid;
            /// ---------------------------------------------------------------------
            ///
        }

        if (!_index.transform.GetComponent<Collider>().enabled)
            NoMoreParent();


        //##################################################### PLANE INTERACTION SECTION #################################
        // Initialize this variable.
        CF_pos = dll_xyz_copy;

        // ====[ DO WE UNSTICK? ]
        if (Vector3.Distance(CF_pos, LF_pos) > UnstickThreashold)
            NoMoreParent();
        LF_pos = CF_pos;
        // ===[ Calc The Triangle Centroid ]
        foreach (GameObject fb in fingerBallsInScene)
            P_sum += fb.transform.position;
        P_centroid = P_sum / 3;

        // ====[ DEFINE TRIANGLE & GET NORMAL OF TRIANGLE! ]
        Plane myTriangle = new Plane(fingerBallsInScene[0].transform.position, fingerBallsInScene[1].transform.position, fingerBallsInScene[2].transform.position);
        CF_normal = myTriangle.normal.normalized;

        // # For Debugging <--_--> ___________________________________________________________________________________________
        Debug.DrawLine(fingerBallsInScene[0].transform.position, fingerBallsInScene[1].transform.position, Color.yellow);
        Debug.DrawLine(fingerBallsInScene[1].transform.position, fingerBallsInScene[2].transform.position, Color.yellow);
        Debug.DrawLine(fingerBallsInScene[2].transform.position, fingerBallsInScene[0].transform.position, Color.yellow);
        Debug.DrawLine(P_centroid, P_centroid + myTriangle.normal.normalized * 0.1f, Color.blue);
        // ___________________________________________________________________________________________________________________

        // ===[ CLOCK-WISE OR ANTICLOCK-WISE? ]
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
        // =====[ Anti-Occlusion Offset? ]
        var doesOcclude = Vector3.Dot(CF_normal, Vector3.back);
        if (doesOcclude < 0)
        {
            if (!enableOffset)
            {
                //enableOffset = true;
                enableOffset = false;

            }
        }
        else
        {
            if (enableOffset)
            {
                enableOffset = false;
            }
        }

        LF_StartEnd[0] = P_centroid;
        CF_normal = myTriangle.normal.normalized;
        Quaternion rot1 = Quaternion.FromToRotation(LF_normal, CF_normal);
        Quaternion rot2 = Quaternion.AngleAxis(angleAroundNormal, myTriangle.normal.normalized);
        //Quaternion rot2 = Quaternion.AngleAxis(angleAroundNormal, Vector3.back);

        // ==[ Calculate where & how far it is to palm center ]
        Vector3 MIDtoPEK = fingerBallsInScene[1].transform.position - fingerBallsInScene[0].transform.position;
        Vector3 towardsPalm = Vector3.Cross(CF_normal, MIDtoPEK);
        towardsPalm = -towardsPalm.normalized * (Vector3.Distance(fingerBallsInScene[1].transform.position, fingerBallsInScene[0].transform.position) * 0.45f);

        // ===[ Transform the Object of Interest ]
        if (collidedObj)
        {
            //[Translate The Object]-------------
            ///collidedObj.transform.position = CF_pos + (fromFingerToObject);
            if (enableOffset)
            {
                collidedObj.transform.position = P_centroid + myTriangle.normal.normalized * 0.02f + 
                                                 (MIDtoPEK.normalized * (Vector3.Distance(fingerBallsInScene[1].transform.position, fingerBallsInScene[0].transform.position) * 1.75f));
                //[Rotate The Object]-------------
                collidedObj.transform.Rotate(rot1.eulerAngles, Space.World);
                var orthoOrNot = Vector3.Dot(LF_normal, (fingerBallsInScene[1].transform.position - P_centroid));
                collidedObj.transform.Rotate(rot2.eulerAngles, Space.World);
            }
            else
            {
                //collidedObj.transform.position = P_centroid + myTriangle.normal.normalized * 0.06f + towardsPalm;

                float one_cm_from_hand;

                if (collidedObj.name == "Blue Dice")
                    one_cm_from_hand= (collidedObj.transform.GetComponent<BoxCollider>().size.x / 2) + 0.01f;
                else
                    one_cm_from_hand = (collidedObj.transform.localScale.x / 2) + 0.01f;

                collidedObj.transform.position = P_centroid + myTriangle.normal.normalized * one_cm_from_hand + towardsPalm;

                //[Rotate The Object]-------------
                collidedObj.transform.Rotate(rot1.eulerAngles, Space.World);
                var orthoOrNot = Vector3.Dot(LF_normal, (fingerBallsInScene[1].transform.position - P_centroid));
                collidedObj.transform.Rotate(rot2.eulerAngles, Space.World);
            }

            YAW = GetYawAngle(CF_normal);
            PITCH = GetPitchAngle(CF_normal);
            //print("Yaw: " + yaw + " | Pitch: " + pitch);
        }
        // ===[ Update what's OLD ]
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

            released = false;
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
        //Debug.Log("NoMoreParent HAS BEEN CALLED!");
        if (collidedObj != null)
        {
            collidedObj.GetComponent<Rigidbody>().isKinematic = false;
            collidedObj = null;

            released = true;
        }
    }

    private float FingerUpdate()
    {
        Marshal.Copy(ImageUpdate(), FINGER_POINTS, 0, FINGER_POINTS.Length);

        return 0.0f;
    }

    private Vector3 Get_Weighted_Mean(List<Vector3> pList)
    {
        if (pList.Count == 0)
        {
            return Vector3.zero;
        }

        float w_old = 0.05f;
        float w_new = 1.95f;
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

    private float GetYawAngle(Vector3 plane_normal)
    {
        var vec = Vector3.ProjectOnPlane(plane_normal,Vector3.up);
        var angle = Vector3.Angle(Vector3.back, vec);
        if(Vector3.Dot(vec, Vector3.right) < 0)
        {
            angle *= -1;
        }
        return angle;
    }
    private float GetPitchAngle(Vector3 plane_normal)
    {
        var vec = Vector3.ProjectOnPlane(plane_normal, Vector3.right);
        var angle = Vector3.Angle(Vector3.back, vec);
        if (Vector3.Dot(vec, Vector3.down) < 0)
        {
            angle *= -1;
        }
        return angle;
    }
    // Show the number of calls to both messages.
    void OnGUI()
    {
        GUIStyle fontSize = new GUIStyle(GUI.skin.GetStyle("label"));
        GUI.contentColor = Color.red;
        fontSize.fontSize = 24;
        GUI.Label(new Rect(800, 900, 200, 50), "Yaw: " + Mathf.RoundToInt(YAW) + "°", fontSize);
        GUI.Label(new Rect(1000, 900, 200, 50), "Pitch: " + Mathf.RoundToInt(PITCH) + "°", fontSize);
    }

    void OnApplicationQuit()
    {
#if UNITY_EDITOR
        
        if(!OPTask.IsCompleted)
        {
            OPTask.Wait();
            OPTask.Dispose();
        }
        
        Shutdown();
        CloseLibrary(libraryHandle);
        libraryHandle = IntPtr.Zero;
  
#endif

    }



}