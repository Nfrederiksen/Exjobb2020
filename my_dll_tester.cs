using System;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using UnityEngine;



//#####################################################################
//  _______ ______  _____ _______    ____  _   _ _  __     __       ###
// |__   __|  ____|/ ____|__   __|  / __ \| \ | | | \ \   / /       ###
//    | |  | |__  | (___    | |    | |  | |  \| | |  \ \_/ /        ###
//    | |  |  __|  \___ \   | |    | |  | | . ` | |   \   /         ###    
//    | |  | |____ ____) |  | |    | |__| | |\  | |____| |          ###
//    |_|  |______|_____/   |_|     \____/|_| \_|______|_|          ###
//                                                                  ###
//#####################################################################                                                           




public class my_dll_tester : CollisionTester
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

    /// <summary>********************************************
    /// ORDINARY C# UNITY STUFF******************************
    /// </summary>*******************************************
    private GameObject GO;
    private GameObject roteramig;
    private Vector3 mOffset;
    int LOG_OUTPUT_DLL, LOG_OUTPUT_DLL_F3;
    float speed, update, mZCoord;
    Vector3 dll_xyz, old_dll_xyz, dll_xyz_tumme;
    
    private Vector3 GetFingerAsWorldPoint()
    {

        Marshal.Copy(ImageUpdate(), FINGER_POINTS, 0, FINGER_POINTS.Length);
        // Pixel coordinates of mouse (x,y)
        Vector3 fingerPoint;
        fingerPoint.x = -2*FINGER_POINTS[0];
        fingerPoint.y = -2 * FINGER_POINTS[1];
        fingerPoint.z = FINGER_POINTS[2];
        // z coordinate of game object on screen
        speed = fingerPoint.z*0.01f;
        fingerPoint.z = mZCoord;
        // Convert it to world points
        return Camera.main.ScreenToWorldPoint(fingerPoint);
    }

    float[] FINGER_POINTS = new float[15];//[6];
    protected float grabDist=10f;
    protected float getGrabDist()
    {
        return grabDist;
    }


    protected List<GameObject> objectsInSceneboi;

    //################ UNITY START() ################################
    void Start()
    {
        objectsInSceneboi = new List<GameObject>();

        objectsInSceneboi = GetAllObjectsOnlyInScene();

        Debug.Log("Alla GameObejcts i Scenen boi: " + objectsInSceneboi.Count);

        GO = new GameObject();
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

        mZCoord = Camera.main.WorldToScreenPoint(gameObject.transform.position).z;
        // Store offset = gameobject world pos - mouse world pos
        //mOffset = gameObject.transform.position - GetFingerAsWorldPoint();
        speed = 0.6f;
    }

    float grab_tresh = 0.04f;
    float release_thresh = 0.07f;

    float dist = 0f;
    bool checkDist = false;
    bool dll_grabb = false;
    bool dll_released = true;

    void Update()
    {
        update = MonoUpdate();


        Marshal.Copy(ImageUpdate(), FINGER_POINTS, 0, FINGER_POINTS.Length);
        // Pekfinger
        dll_xyz.x = FINGER_POINTS[0];
        dll_xyz.y = FINGER_POINTS[1];
        dll_xyz.z = FINGER_POINTS[2];
        dll_xyz.x *= -1;

        //print("X:" + dll_xyz.x + "| Y:" + dll_xyz.y + "| Z:" + dll_xyz.z);

        // Tumme
        dll_xyz_tumme.x = FINGER_POINTS[3];
        dll_xyz_tumme.y = FINGER_POINTS[4];
        dll_xyz_tumme.z = FINGER_POINTS[5];
        dll_xyz_tumme.x *= -1;


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

        //print("p0 ->X:" + p0.x + "| Y:" +p0.y + "| Z:" + p0.z);

        //print("p1 ->X:" + p1.x + "| Y:" + p1.y + "| Z:" + p1.z);

        //print("p2 ->X:" + p2.x + "| Y:" + p2.y + "| Z:" + p2.z);


        // Chill No twitching
        if (checkDist)
        {
            dist = Vector3.Distance(dll_xyz, old_dll_xyz);
            if (dist < 0.005f) //0.005f
                dll_xyz = old_dll_xyz;
            else
                old_dll_xyz = dll_xyz;
        }
        else
        {
            old_dll_xyz = dll_xyz;
            checkDist = true;
        }
        // You grab? Me grab? Grabb?
        grabDist = Vector3.Distance(dll_xyz, dll_xyz_tumme);
        //Debug.Log("Grab Check Distance->:" + grabDist);

        int Ican = CanISelect(objectsInSceneboi);
        // %%% TODO %%%%% Om två objekt är nära varan, vilken ska man prioritera? 
        //                En kanske är röd och den andra är gul, Men man styr endast 
        //                den senaste funna.

        // rotate
        if (grabDist < grab_tresh && dll_released == true && dll_grabb == false && Ican == 3)
        {
            Debug.Log("Grabbed now rotate!");
            //ObjectStickToFinger(dll_grabb, dll_released, GO);

            roteramig = WhoAmI();

            if (roteramig != null)
            {       
              dll_grabb = true;
            }
            dll_released = false;
        }

        if (roteramig && dll_grabb)
        {
            Vector3 rotate = roteramig.transform.position - transform.position;

            if (rotate.x > 1) rotate.x = -1;
            else if (rotate.x < -1) rotate.x = 1;
            else rotate.x = 0;

            if (rotate.y > 1) rotate.y = -1;
            else if (rotate.y < -1) rotate.y = 1;
            else rotate.y = 0;

            if (rotate.z > 1) rotate.z = -1;
            else if (rotate.z < -1) rotate.z = 1;
            else rotate.z = 0;

            float rot_x = 0.5f*Time.deltaTime * rotate.x;
            float rot_y = 0.5f * Time.deltaTime * rotate.y;
            float rot_z = 0.5f * Time.deltaTime * rotate.z;
            roteramig.transform.RotateAround(Vector3.up, rot_x);
            roteramig.transform.RotateAround(Vector3.right, -rot_y);
            roteramig.transform.RotateAround(Vector3.forward, rot_z);
        }

        // garbbed
        if (grabDist < grab_tresh && dll_released == true && dll_grabb == false && Ican == 2)
        {
            Debug.Log("Grabbed!");
            ObjectStickToFinger(dll_grabb, dll_released, GO);

            if (CubeTransform != null)
            {
                if (CubeTransform.parent != null)
                {
                    dll_grabb = true;           
                }
            }
            dll_released = false;
        }

        // open fist
        if (grabDist > release_thresh && dll_released == false)
        {
            dll_released = true;

            if(roteramig)
            {
                roteramig = null;
                StopRotating();
            }
        }


        // released
        if (CubeTransform != null && roteramig == null)
        {
            if (grabDist > release_thresh && dll_grabb == true && dll_released == true)
            {
                Debug.Log("Released!");

                ObjectStickToFinger(dll_grabb, dll_released, GO);
                dll_grabb = false;
                dll_released = false;
            }
        }    
        //dll_xyz.z += 0.5f;
        //Debug.Log("dll_xyz ->[ " + dll_xyz + " ]");

        //############################################## PERFORM OBJECT TRANSFORMATION #####################
        transform.position = dll_xyz * 10;
        //=------------------------------------------------------------------------------------------------=
        // x10 to make it in dm (i.e. Unity would see 0.561m->0.581m, when moving 2.0cm/0.2dm to the left. |
        // But would now see 5.610m -> 5.810m, when moving 2cm to the left. So basically 2cm in real world |
        // becomes 0.2m in Unity instead of 0.02m)                                                         |
        //=------------------------------------------------------------------------------------------------=
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