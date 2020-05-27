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
    public GameObject plane;// = GameObject.CreatePrimitive(PrimitiveType.Plane); and -new (1/7)
    private Vector3 mOffset;
    int LOG_OUTPUT_DLL, LOG_OUTPUT_DLL_F3;
    float speed, update, mZCoord;
    Vector3 dll_xyz, old_dll_xyz, dll_xyz_tumme, old_dll_xyz_tumme, old_p0; //-new (2/7)
    
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
        //GameObject createdplane = GameObject.CreatePrimitive(PrimitiveType.Plane);



        objectsInSceneboi = new List<GameObject>();

        objectsInSceneboi = GetAllObjectsOnlyInScene();

        Debug.Log("Alla GameObejcts i Scenen boi: " + objectsInSceneboi.Count);

        GO = new GameObject();

        plane = WhoIsPlane();//-new (3/7)
        if(plane == null)
        {
            print("Aj Aj Senjor");
        }


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
    bool checkDist2 = false;//-new 
    bool checkDist3 = false;//-new (4/7)
    bool dll_grabb = false;
    bool dll_released = true;

    int transformation_mode = 0;
    // For Scaling.
    float d0 = 0f;
    float prevScale = 0;
    // For Rotating.
    Vector3 d0xyz;
    Vector3 prevRot_xyz;
    float grow_LF = 0;
    float grow_CF = 0;
    float do_grow = 0;
    // new test stuff (5/7)
    Vector3 pl_norm_LF = Vector3.zero;
    Vector3 pl_norm_CF = Vector3.zero;
    float vec_angle = 0;
    Vector3 LF_dllxyz = Vector3.zero;
    Vector3 CF_dllxyz = Vector3.zero;


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
        // Coord.System. Cali. points.
        Vector3 p0;//, p1, p2;
        p0.x = FINGER_POINTS[6];
        p0.y = FINGER_POINTS[7];
        p0.z = FINGER_POINTS[8];
        p0.z *= -1;

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
        if (checkDist2)
        {
            dist = Vector3.Distance(dll_xyz_tumme, old_dll_xyz_tumme);
            if (dist < 0.005f) //0.005f
                dll_xyz_tumme = old_dll_xyz_tumme;
            else
                old_dll_xyz_tumme = dll_xyz_tumme;
        }
        else
        {
            old_dll_xyz_tumme = dll_xyz_tumme;
            checkDist2 = true;
        }
        if (checkDist3)
        {
            dist = Vector3.Distance(p0, old_p0);
            if (dist < 0.005f) //0.005f
                p0 = old_p0;
            else
                old_p0 = p0;
        }
        else
        {
            old_p0 = p0;
            checkDist3 = true;
        }

        if (plane != null)
        {
            // ()Make plane?----
            Plane myPlane = new Plane(dll_xyz_tumme * 10, p0*10, dll_xyz * 10);
            

            //[# Draw a Debug Triangle with normal #]
            Vector3 pl_centroid = ((dll_xyz_tumme + p0 + dll_xyz) * 10 / 3);
            // ()()() Lets try this ?? O.O
         
            plane.transform.localPosition = pl_centroid;
            plane.transform.localScale = new Vector3(.1f, .1f, .1f);
            plane.transform.rotation = Quaternion.FromToRotation(Vector3.up, myPlane.normal.normalized);
            //plane.transform.rotation = Quaternion.FromToRotation(dll_xyz, myPlane.normal.normalized);

            pl_norm_CF = pl_centroid;
            CF_dllxyz = dll_xyz;

            if(pl_norm_LF != Vector3.zero && LF_dllxyz != Vector3.zero)
            {
                //vec_angle = Vector3.Angle((CF_dllxyz - pl_norm_LF), (LF_dllxyz - pl_norm_LF));
                //plane.transform.RotateAround(pl_centroid + myPlane.normal.normalized, 45);
                //plane.transform.RotateAround(Vector3.forward, 0);
                //print("Vec ANGLE->:" + vec_angle);
            }
                
            //Vector3 pl_normal = CalcNormalVector(dll_xyz_tumme, dll_xyz, p0);
            Debug.DrawLine(dll_xyz_tumme * 10, p0*10, Color.yellow);
            Debug.DrawLine(p0*10, dll_xyz * 10, Color.yellow);
            Debug.DrawLine(dll_xyz * 10, dll_xyz_tumme * 10, Color.yellow);
            Debug.DrawLine(pl_centroid, pl_centroid + myPlane.normal.normalized,  Color.blue);

            pl_norm_LF = pl_norm_CF;
            LF_dllxyz = CF_dllxyz;

        }
        else
            print("[]()[]() HAHAHAHAHA");
        // ==============================================[][] Part of the TriPlane Magic [][]==== ^
        

        //p1.x = FINGER_POINTS[9];
        //p1.y = FINGER_POINTS[10];
        //p1.z = FINGER_POINTS[11];

        //p2.x = FINGER_POINTS[12];
        //p2.y = FINGER_POINTS[13];
        //p2.z = FINGER_POINTS[14];

        //  print("p0 ->X:" + p0.x + "| Y:" +p0.y + "| Z:" + p0.z);
        //print("p1 ->X:" + p1.x + "| Y:" + p1.y + "| Z:" + p1.z);
        // print("p2 ->X:" + p2.x + "| Y:" + p2.y + "| Z:" + p2.z);

        // #### Is the User Pinching???
        grabDist = Vector3.Distance(dll_xyz, dll_xyz_tumme);
        //Debug.Log("Grab Check Distance->:" + grabDist);

        // Is the object selectable AND what color state is the object in?
        int Ican = CanISelect(objectsInSceneboi);

        //_______________________________________________________________________________
        // %%% TODO %%%%% Om två objekt är nära varan, vilken ska man prioritera?       |
        //                En kanske är röd och den andra är gul, Men man styr endast    |
        //                den senaste funna.                                            |
        //______________________________________________________________________________|

        // [Do We Scale? Then scale]-  Grabbed || Ican = 4 
        DoWeScale(Ican);

        // [Do We Rotate? Then rotate]-  Grabbed || Ican = 3 
        DoWeRotate(Ican);

        // [Do We Translate? Then Translate] -  Grabbed || Ican = 2 
        DoWeTranslate(Ican);

        

        // open fist | RELEASE PART (1/2)
        if (grabDist > release_thresh && dll_released == false)
        {
            dll_released = true;

            if(roteramig)
            {
                roteramig = null;
                StopRotating();
                transformation_mode = 0;
            }
        }
        // released | RELEASE PART (2/2)
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
        
        //############################################## PERFORM CURSOR-OBJECT TRANSLATION #################
        transform.position = dll_xyz * 10;
        //=------------------------------------------------------------------------------------------------=
        // x10 to make it in dm (i.e. Unity would see 0.561m->0.581m, when moving 2.0cm/0.2dm to the left. |
        // But would now see 5.610m -> 5.810m, when moving 2cm to the left. So basically 2cm in real world |
        // becomes 0.2m in Unity instead of 0.02m)                                                         |
        //=------------------------------------------------------------------------------------------------=
    }


// ^=^=^=^=^=^=^=^= Group of the Void Functions ^^=^=^=^=^=^=^=^=
    void DoWeScale( int colorState)
    {
        // #### Decide and give clearance for object to transformatioon
        if (grabDist < grab_tresh && dll_released == true && dll_grabb == false && colorState == 4)
        {   
            // Just What Object are we Talking about here :o ?
            roteramig = WhoAmI();
            // Change Color State.
            roteramig.transform.GetComponent<Renderer>().material.color = Color.cyan;
            //---------------------------------------We are for Scaling ----------------
            d0 = Vector3.Distance(roteramig.transform.position, transform.position);
            prevScale = roteramig.transform.localScale.x;
            //--------------------------------------------------------------------------
            if (roteramig != null)
            {
                dll_grabb = true;
            }
            dll_released = false;

            transformation_mode = 4;
        }
        // ObjectSelected = True, -> Tmode now always = 1;
        //############################################## PERFORM OBJECT SCALING #####################
        if (roteramig && dll_grabb && transformation_mode == 4) //TODO Gatekeeper
        {
            // [x]Test Code for Scaling!----------------
            float up_change_rate_factor = 0.2f;
            float down_change_rate_factor = 0.8f;
            float growth = 0f;
            float dn = Vector3.Distance(roteramig.transform.position, transform.position);

            var _DDP = ((dn / d0) - 1); //(Distance-Distparity-Percentage)
            // Put Slower growth-rate on upscaling.
            if (_DDP < 0)
            {
                growth = _DDP * down_change_rate_factor;
            }
            else
            {
                growth = _DDP * up_change_rate_factor;
            }

            var become_this_big = prevScale + growth;
            // Scale Step 1: the Object of Interest! 
            roteramig.transform.localScale = new Vector3(become_this_big, become_this_big, become_this_big);
            // Scale Step 2: the Collider accoringly depending on type. (Limited to types: Box & Sphere)
            if (roteramig.transform.GetComponent<SphereCollider>() != null)
            {
                roteramig.transform.GetComponent<SphereCollider>().radius = become_this_big * 0.8f;
            }
            else
            {
                roteramig.transform.GetComponent<BoxCollider>().size = new Vector3(become_this_big * 0.8f, become_this_big * 0.8f, become_this_big * 0.8f);
            }
            //prevScale = roteramig.transform.localScale.x;
            //print("scaleChange->:" + become_this_big + " | dn->:" + dn + " | d0->:" + d0+" | and also (dn/d0)->:"+(dn/d0));
        }
    }

    void DoWeRotate(int colorState)
    {
        // #### Decide and give clearance for object to transformatioon
        if (grabDist < grab_tresh && dll_released == true && dll_grabb == false && colorState == 3)
        {
            Debug.Log("Grabbed now rotate!");
            //ObjectStickToFinger(dll_grabb, dll_released, GO);

            // Just What Object are we Talking about here :o ?
            roteramig = WhoAmI();
            // Change Color State.
            roteramig.transform.GetComponent<Renderer>().material.color = Color.green;
            //---------------------------------------We are for Rotate ----------------
            d0xyz = roteramig.transform.position - transform.position;
            prevRot_xyz = roteramig.transform.localEulerAngles;
            grow_LF = 0; // Reset Grow Last Frame
            //--------------------------------------------------------------------------
            if (roteramig != null)
            {
                dll_grabb = true;
            }
            dll_released = false;

            transformation_mode = 3;
        }
        //############################################## PERFORM OBJECT ROTATION #####################
        if (roteramig && dll_grabb && transformation_mode == 3)
        {
            float hub_radius = 0.75f;
            bool inside_hub = false;
            bool in_a_tube = false;
            int WeOnlyDo = 0; // 1 = Pitch, 2 = Yaw, 3 = Roll.

            Vector3 rotate = roteramig.transform.position - transform.position;
            Vector3 dn = rotate;
            float change_rate_factor = 100f;
            Vector3 growth; // (The Angle Growth in x,y,z)
            var _DDP = dn - d0xyz; //(Distance-Distparity-Percentage)
            growth = _DDP * change_rate_factor;


            // []Test if are inside or outside the hub! 
            if (Mathf.Abs(rotate.x) < hub_radius && Mathf.Abs(rotate.y) < hub_radius && Mathf.Abs(rotate.z) < hub_radius)
            {
                inside_hub = true;
                roteramig.transform.GetComponent<Renderer>().material.color = Color.green;
                d0xyz = roteramig.transform.position - transform.position;
                prevRot_xyz = roteramig.transform.localEulerAngles;
                grow_LF = 0;
            }
            else
            {
                roteramig.transform.GetComponent<Renderer>().material.color = new Color(0.1f, 0.7f, 0.2f);
                inside_hub = false;

            }

            // Do things depending on where we are! 
            if (inside_hub)
            {
                in_a_tube = false;
            }

            if(!inside_hub && !in_a_tube) // Then you're in the hub no?
            {
                var _Y = Mathf.Abs(rotate.y);
                var _X = Mathf.Abs(rotate.x);
                var _Z = Mathf.Abs(rotate.z);

                if (_X > _Y && _X > _Z) // If X is largest-> user trying to YAW
                {
                    in_a_tube = true;
                    WeOnlyDo = 2;
                }
                else if (_Y > _X && _Y > _Z)// If Y is largest-> user trying to PITCH
                {
                    in_a_tube = true;
                    WeOnlyDo = 1;
                }
                else // Then Z is largest-> user trying to ROLL
                {
                    in_a_tube = true;
                    WeOnlyDo = 3;
                }
            }
            // This is where we rotate along 1 axis, or tube.
            if(!inside_hub && in_a_tube)
            {
                
                if (WeOnlyDo == 1)
                {   // FOR PITCH WE NEED LOCALROTATION!! !FFS
                    grow_CF = growth.y; // Update Grow Current Frame.
                    dist = Mathf.Abs(grow_CF - grow_LF);
                    if (dist < 0.15f) // If not diff. enough stay as Last Frame.
                        grow_CF = grow_LF;
                    do_grow = Mathf.Round(grow_CF) - Mathf.Round(grow_LF); // Calc diff degree.
                    grow_LF = grow_CF; // Update Grow Last Frame.
                    print("[] pitch []Do Grow ->:" + do_grow);
                    roteramig.transform.Rotate(new Vector3(-do_grow * 0.5f, 0, 0), Space.World);
                    roteramig.transform.Rotate(new Vector3(0, 0, 0), Space.World);
                }
                else if(WeOnlyDo == 2)
                {
                    grow_CF = growth.x; // Update Grow Current Frame.
                    dist = Mathf.Abs(grow_CF - grow_LF);
                    if (dist < 0.15f) // If not diff. enough stay as Last Frame.
                        grow_CF = grow_LF;
                    do_grow = Mathf.Round(grow_CF) - Mathf.Round(grow_LF); // Calc diff degree.
                    grow_LF = grow_CF; // Update Grow Last Frame.
                    print("[] yaw []Do Grow ->:" + do_grow);
                    roteramig.transform.Rotate(new Vector3(0, do_grow * 0.5f, 0), Space.World);
                    roteramig.transform.Rotate(new Vector3(0, 0, 0), Space.World);
                }
                else
                {
                    grow_CF = growth.z; // Update Grow Current Frame.
                    dist = Mathf.Abs(grow_CF - grow_LF);
                    if (dist < 0.15f) // If not diff. enough stay as Last Frame.
                        grow_CF = grow_LF;
                    do_grow = Mathf.Round(grow_CF) - Mathf.Round(grow_LF); // Calc diff degree.
                    grow_LF = grow_CF; // Update Grow Last Frame.
                    print("[] roll []Do Grow ->:" + do_grow);
                    roteramig.transform.Rotate(new Vector3(0, 0, do_grow * 0.5f), Space.World);
                    roteramig.transform.Rotate(new Vector3(0, 0, 0), Space.World);
                }
            }
        }
    }

    void DoWeTranslate(int colorState)
    {
        if (grabDist < grab_tresh && dll_released == true && dll_grabb == false && colorState == 2)
        {
            Debug.Log("Grabbed!");
            ObjectStickToFinger(dll_grabb, dll_released, GO); // Assign parent-child relationship.

            if (CubeTransform != null)
            {
                if (CubeTransform.parent != null)
                {
                    dll_grabb = true;
                }
            }
            dll_released = false;

            transformation_mode = 2;

            release_thresh = 0.17f; // - new (7/7)
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