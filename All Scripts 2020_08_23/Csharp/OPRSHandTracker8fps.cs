using System;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using UnityEngine;
using System.Collections;




//#####################################################################
//  _______ ______  _____ _______    ____  _   _ _  __     __       ###
// |__   __|  ____|/ ____|__   __|  / __ \| \ | | | \ \   / /       ###
//    | |  | |__  | (___    | |    | |  | |  \| | |  \ \_/ /        ###
//    | |  |  __|  \___ \   | |    | |  | | . ` | |   \   /         ###    
//    | |  | |____ ____) |  | |    | |__| | |\  | |____| |          ###
//    |_|  |______|_____/   |_|     \____/|_| \_|______|_|          ###
//                                                                  ###
//#####################################################################                                                           

/// <summary>
///  Btw, rs2::alignment reduces the framerate. ex. We put 60fps but got 29fps.
/// </summary>



public class OPRSHandTracker8fps : CollisionTester
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
    const string DLL_NAME = "beta_CAM2_old";
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

    /// <summary>********************************************
    /// ORDINARY C# UNITY STUFF******************************
    /// </summary>*******************************************
    private GameObject GO, thumbGO, middleGO, canvasLeftGO, canvasRightGO;
    private GameObject roteramig;
    int LOG_OUTPUT_DLL, LOG_OUTPUT_DLL_F3;
    float update;
    Vector3 dll_xyz,dll_xyz_copy, dll_xyz_tumme, dll_xyz_mid;
    int counter = 0;
    private float updateCount = 0;
    private float updateUpdateCountPerSecond;

    float[] FINGER_POINTS = new float[16];

    protected List<GameObject> objectsInScene;
    [SerializeField]
    [Range(1, 15)]
    protected int meanListSize = 5;

    void Awake()
    {
        // Uncommenting this will cause framerate to drop to 10 frames per second.
        // This will mean that FixedUpdate is called more often than Update.
        //Application.targetFrameRate = 10;
        StartCoroutine(Loop());
    }


    //################ UNITY START() ################################
    void Start()
    {
        objectsInScene = new List<GameObject>();
        objectsInScene = GetAllObjectsOnlyInScene();
        Debug.Log("Alla GameObejcts i Scenen boi: " + objectsInScene.Count);

        GO = new GameObject();
        thumbGO = GiveMeThumbGO();
        middleGO = GiveMeMiddleGO();
        canvasLeftGO = GiveMeLeftCanvasGO();
        canvasRightGO = GiveMeRightCanvasGO();

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
    }

    protected List<Vector3> vector3List_PEK = new List<Vector3>();
    protected List<Vector3> vector3List_TUM = new List<Vector3>();
    protected List<Vector3> vector3List_MID = new List<Vector3>();

    void Update()
    {
        update = MonoUpdate();
        updateCount += 1;
        Marshal.Copy(ImageUpdate(), FINGER_POINTS, 0, FINGER_POINTS.Length);

        // Pekfinger
        dll_xyz.x = FINGER_POINTS[0];
        dll_xyz.y = FINGER_POINTS[1];
        dll_xyz.z = FINGER_POINTS[2];
        dll_xyz.x *= -1;
        dll_xyz_copy = dll_xyz;
         // Tumme
        dll_xyz_tumme.x = FINGER_POINTS[3];
        dll_xyz_tumme.y = FINGER_POINTS[4];
        dll_xyz_tumme.z = FINGER_POINTS[5];
        dll_xyz_tumme.x *= -1;
        // Mid a.k.a Tumme 2
        dll_xyz_mid.x = FINGER_POINTS[6];
        dll_xyz_mid.y = FINGER_POINTS[7];
        dll_xyz_mid.z = FINGER_POINTS[8];
        dll_xyz_mid.x *= -1;

        //################################# BROKEN TRACKING OR NOT?? ########################################
        if (FINGER_POINTS[15] == 1.0f)
        {
            counter++;
            if (counter > 5)
            {
                if (!canvasLeftGO.GetComponent<Canvas>().enabled)
                {
                    canvasLeftGO.GetComponent<Canvas>().enabled = true;
                    canvasRightGO.GetComponent<Canvas>().enabled = true;

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
            if (canvasLeftGO.GetComponent<Canvas>().enabled)
            {
                counter = 0;
                foreach (GameObject GO in objectsInScene)
                {
                    GO.GetComponent<MeshRenderer>().enabled = true;
                }
                canvasLeftGO.GetComponent<Canvas>().enabled = false;
                canvasRightGO.GetComponent<Canvas>().enabled = false;
            }
        }
        // ===[ Calc Mean Position ]
        Add_Pos_To_List(vector3List_PEK, dll_xyz, meanListSize);
        Add_Pos_To_List(vector3List_TUM, dll_xyz_tumme, meanListSize);
        Add_Pos_To_List(vector3List_MID, dll_xyz_mid, meanListSize);
        dll_xyz = Get_Weighted_Mean(vector3List_PEK);
        dll_xyz_tumme = Get_Weighted_Mean(vector3List_TUM);
        dll_xyz_mid = Get_Weighted_Mean(vector3List_MID);

        //############################################## PERFORM OBJECT TRANSFORMATION #####################
        middleGO.transform.position = dll_xyz_mid;

        thumbGO.transform.position = dll_xyz_tumme;

        transform.position = dll_xyz;


        PositionTextScript.posValue = dll_xyz;
        DistTextScript.DistValue = Vector3.Distance(dll_xyz, new Vector3(-0.025f, 0.169f, -0.58f));
    }

    // Show the number of calls to both messages.
    void OnGUI()
    {
        GUIStyle fontSize = new GUIStyle(GUI.skin.GetStyle("label"));
        fontSize.fontSize = 24;
        GUI.Label(new Rect(100, 100, 200, 50), "Update FPS: " + updateUpdateCountPerSecond.ToString(), fontSize);
    }

    // Update both CountsPerSecond values every second.
    IEnumerator Loop()
    {
        while (true)
        {
            yield return new WaitForSeconds(1);
            updateUpdateCountPerSecond = updateCount;

            updateCount = 0;
        }
    }


    // FUNCTION-VILLE  _________TT_    ----------------------------------
    //                /____________\    
    //                 |   |-|  0 |   Here we put our helpful functions!
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