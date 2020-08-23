using System;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using UnityEngine;
using System.Collections;
using UnityEditor;
using System.Threading.Tasks;



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



public class OPRSHandTrackerWithGestures : CollisionTester
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

    /// <summary>********************************************
    /// ORDINARY C# UNITY STUFF******************************
    /// </summary>*******************************************
    private GameObject GO, thumbGO, middleGO;
    private GameObject roteramig;
    private GameObject plane;// = 
    private GameObject puzzleP;
    private Vector3 mOffset;
    int LOG_OUTPUT_DLL, LOG_OUTPUT_DLL_F3;
    float speed, update, mZCoord;
    Vector3 dll_xyz, old_dll_xyz, dll_xyz_tumme, old_dll_xyz_tumme, old_p0; //-new (2/7)
    Task<float> OPTask;

    float[] FINGER_POINTS = new float[16];//[6];
    protected float grabDist = 10f;
    protected float getGrabDist()
    {
        return grabDist;
    }


    protected List<GameObject> objectsInSceneboi;
    public bool isReleased = true;


    //################ UNITY START() ################################
    void Start()
    {
        objectsInSceneboi = new List<GameObject>();
        old_dll_xyz = Vector3.one;
        objectsInSceneboi = GetAllObjectsOnlyInScene();

        Destroy(gameObject.transform.GetComponent<Rigidbody>());

        Debug.Log("Alla GameObejcts i Scenen boi: " + objectsInSceneboi.Count);

        puzzleP = GameObject.Find("puzzle_piece");
        puzzleP.GetComponent<Rigidbody>().isKinematic = true;
        GO = new GameObject();
        thumbGO = GiveMeThumbGO();
        middleGO = GiveMeMiddleGO();

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

        update = MonoUpdate();
        OPTask = Task<float>.Factory.StartNew(() => FingerUpdate());
        print("OPTask started: " + OPTask.Result);
    }

    float grab_tresh = 0.04f;
    float release_thresh = 0.06f;

    float dist = 0f;
    bool checkDist = false;
    bool checkDist2 = false;//-new 
    bool checkDist3 = false;//-new (4/7)
    bool dll_grabb = false;
    bool dll_released = true;
    bool not_first_time = true;
    bool in_a_tube = false;
    int WeOnlyDo;

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

    protected List<Vector3> vector3List_PEK = new List<Vector3>();
    protected List<Vector3> vector3List_TUM = new List<Vector3>();
    GameObject canvasLeft, canvasRight, resetButtonGO;
    int counter;

    void Update()
    {
        if (!transform.GetComponent<Collider>().enabled)
        {
            ObjectStickToFinger(true, true, GO);
            dll_grabb = false;
            dll_released = false;
        }

        if (!puzzleP.GetComponent<Rigidbody>().isKinematic)
            puzzleP.GetComponent<Rigidbody>().isKinematic = true;

        // ===[ START USING THE DLLs FUNCTIONS ]
        update = MonoUpdate();

        if (OPTask.IsCompleted)
        {
            OPTask.Dispose();
            OPTask = Task<float>.Factory.StartNew(() => FingerUpdate());
        }

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



        // ====[][] Part of the TriPlane Magic [][]========================================== v (6/7)
        // Coord.System. Cali. points.
        Vector3 dll_xyz_mid;//, p1, p2;
        dll_xyz_mid.x = FINGER_POINTS[6];
        dll_xyz_mid.y = FINGER_POINTS[7];
        dll_xyz_mid.z = FINGER_POINTS[8];
        dll_xyz_mid.x *= -1;


        // ============[ Get Mean Position ]
        // PEK
        if (vector3List_PEK.Count < 1)
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
        if (vector3List_TUM.Count < 1)
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
        // You grab? Me grab? Grabb?
        grabDist = Vector3.Distance(dll_xyz, dll_xyz_tumme);
        //Debug.Log("Grab Check Distance->:" + grabDist);


        canvasLeft = GiveMeLeftCanvasGO();
        canvasRight = GiveMeRightCanvasGO();
        resetButtonGO = GiveMeResetButtonGO();
        // ====[ FEEDBACK FOR LOST HAND TRACKING! ]
        if (FINGER_POINTS[15] == 1.0f) // if the tracking is lost 
        {
            // FOR ROTATION FUNCTION
            ObjectStickToFinger(true, true, GO);
            dll_grabb = false;
            dll_released = false;
            not_first_time = true;
            in_a_tube = false;


            // Pekfinger
            transform.position = new Vector3(-0.025f, 0.169f, 0.15f);
            // Tumme 1 & 2
            thumbGO.transform.position = new Vector3(-0.020f, 0.150f, 0.15f);
            middleGO.transform.position = new Vector3(-0.020f, 0.150f, 0.15f);

            counter++;
            if (counter > 5)
            {

                if (!canvasLeft.GetComponent<Canvas>().enabled)
                {
                    canvasLeft.GetComponent<Canvas>().enabled = true;
                    canvasRight.GetComponent<Canvas>().enabled = true;

                    foreach (GameObject GO in objectsInSceneboi)
                    {
                        GO.GetComponent<MeshRenderer>().enabled = false;
                        //GO.GetComponent<Renderer>().material.color = new Color(0f, 0f, 0f, 0.5f);
                    }
                    resetButtonGO.GetComponent<MeshRenderer>().enabled = false;
                }
                counter = 0;
            }
        }
        else
        {
            if (canvasLeft.GetComponent<Canvas>().enabled)
            {
                counter = 0;
                foreach (GameObject GO in objectsInSceneboi)
                {
                    GO.GetComponent<MeshRenderer>().enabled = true;
                }
                resetButtonGO.GetComponent<MeshRenderer>().enabled = true;
                canvasLeft.GetComponent<Canvas>().enabled = false;
                canvasRight.GetComponent<Canvas>().enabled = false;
            }
        }



        // Is the object selectable AND what color state is the object in? ex. 1 or 2 or 3 or 4.
        int Ican = CanISelect(objectsInSceneboi);

        //_______________________________________________________________________________
        // %%% TODO %%%%% Om två objekt är nära varan, vilken ska man prioritera?       |
        //                En kanske är röd och den andra är gul, Men man styr endast    |
        //                den senaste funna.                                            |
        //______________________________________________________________________________|

        /* [Do We Scale? Then scale]-  Grabbed || Ican = 4 */
        //DoWeScale(Ican);

        /* [Do We Rotate? Then rotate]-  Grabbed || Ican = 3 */
        //DoWeRotate_A(Ican); // Rotate in degrees per seconds.

        DoWeRotate_B(Ican); // Rotate in degrees per distance unit.

        /* [Do We Translate? Then Translate] -  Grabbed || Ican = 2 */
        DoWeTranslate(Ican);


        // open fist | RELEASE PART (1/2)
        if (grabDist > release_thresh && dll_released == false)
        {
            dll_released = true;

            if (roteramig)
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
                //Debug.Log("Released! ][][][][][]][][]] Nu är dll_grabb = false");

                ObjectStickToFinger(dll_grabb, dll_released, GO);
                dll_grabb = false;
                dll_released = false;
                not_first_time = true;
                in_a_tube = false;
            }
        }
        //dll_xyz.z += 0.5f;
        //Debug.Log("dll_xyz ->[ " + dll_xyz + " ]");



        isReleased = !IsReleased();

        //############################################## PERFORM OBJECT TRANSFORMATION #####################
        middleGO.transform.position = dll_xyz_mid;

        thumbGO.transform.position = dll_xyz_tumme;


        transform.position = dll_xyz;// * 10;

    }

    // ^=^=^=^=^=^=^=^= Group of the Void Functions ^^=^=^=^=^=^=^=^=
    void DoWeScale(int colorState)
    {
        // #### Decide and give clearance for object to transformatioon
        if (grabDist < grab_tresh && dll_released == true && dll_grabb == false && colorState == 4)
        {
            // Just What Object are we Talking about here :o ?
            roteramig = WhoAmI();
            // Change Color State.
            roteramig.transform.GetComponent<Renderer>().material.color = Color.blue;
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
            float up_change_rate_factor = 0.5f;
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
            // Scale the Object of Interest!
            roteramig.transform.localScale = new Vector3(become_this_big, become_this_big, become_this_big);
            // Scale the Collider accoringly depending on type. (Limited to types: Box & Sphere)
            if (roteramig.transform.GetComponent<SphereCollider>() != null)
            {
                roteramig.transform.GetComponent<SphereCollider>().radius = become_this_big * 0.5f;
            }
            else
            {
                roteramig.transform.GetComponent<BoxCollider>().size = new Vector3(become_this_big * 0.5f, become_this_big * 0.5f, become_this_big * 0.5f);
            }
            //prevScale = roteramig.transform.localScale.x;
            //print("scaleChange->:" + become_this_big + " | dn->:" + dn + " | d0->:" + d0+" | and also (dn/d0)->:"+(dn/d0));
        }
    }

    void DoWeRotate_A(int colorState)
    {
        // #### Decide and give clearance for object to transformatioon
        if (grabDist < grab_tresh && dll_released == true && dll_grabb == false && colorState == 3)
        {
            //Debug.Log("Grabbed now rotate!");
            //ObjectStickToFinger(dll_grabb, dll_released, GO);

            // Just What Object are we Talking about here :o ?
            roteramig = WhoAmI();
            // Change Color State.
            roteramig.transform.GetComponent<Renderer>().material.color = Color.green;
            //---------------------------------------We are for Scaling ----------------
            d0 = Vector3.Distance(roteramig.transform.position, transform.position);
            prevScale = roteramig.transform.localScale.x;
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

            float rot_x = 0.5f * Time.deltaTime * rotate.x;
            float rot_y = 0.5f * Time.deltaTime * rotate.y;
            float rot_z = 0.5f * Time.deltaTime * rotate.z;
            roteramig.transform.RotateAround(Vector3.up, rot_x);
            roteramig.transform.RotateAround(Vector3.right, -rot_y);
            roteramig.transform.RotateAround(Vector3.forward, rot_z);
        }

    }

    void DoWeRotate_B(int colorState)
    {
       // Debug.Log("grabDist = " + grabDist + "| Cond.2 =" + dll_released + "| Cond.3 =" + dll_grabb);
        // #### Decide and give clearance for object to transformatioon
        if (grabDist < grab_tresh && dll_released == true && dll_grabb == false && colorState == 3)
        {
            //Debug.Log("Grabbed now rotate!");
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
            float hub_radius = 1f * 0.1f;
            bool inside_hub = false;
            Vector3 rotate = roteramig.transform.position - transform.position;
            Vector3 dn = rotate;
            float change_rate_factor = 1000f;
            Vector3 growth; // (The Angle Growth in x,y,z)
            var _DDP = dn - d0xyz; //(Distance-Distparity-Percentage)
            growth = _DDP * change_rate_factor;


            // []Test if are inside or outside the hub! 
            if (Mathf.Abs(rotate.x) < hub_radius && Mathf.Abs(rotate.y) < hub_radius && Mathf.Abs(rotate.z) < hub_radius*2)
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
                if(not_first_time)
                    in_a_tube = false; // LETA EFTER PORTAL
                
            }

            if (!inside_hub && !in_a_tube) // Then you're in the hub no?
            {
                var _Y = Mathf.Abs(rotate.y);
                var _X = Mathf.Abs(rotate.x);
                var _Z = Mathf.Abs(rotate.z);

                if (_X > _Y && _X > _Z) // If X is largest-> user trying to YAW
                {
                    in_a_tube = true;
                    WeOnlyDo = 2;
                    not_first_time = false;
                }
                else if (_Y > _X && _Y > _Z)// If Y is largest-> user trying to PITCH
                {
                    in_a_tube = true;
                    WeOnlyDo = 1;
                    not_first_time = false;
                }
                else // Then Z is largest-> user trying to ROLL
                {
                    in_a_tube = true;
                    WeOnlyDo = 3;
                    not_first_time = false;
                }

            }

            //print("We use portal:" + WeOnlyDo);
            // This is where we rotate along 1 axis, or tube.
            if (!inside_hub && in_a_tube)
            {

                if (WeOnlyDo == 1)
                {   // FOR PITCH WE NEED LOCALROTATION!! !FFS
                    grow_CF = growth.y; // Update Grow Current Frame.
                    dist = Mathf.Abs(grow_CF - grow_LF);
                    if (dist < 0.15f) // If not diff. enough stay as Last Frame.
                        grow_CF = grow_LF;
                    do_grow = Mathf.Round(grow_CF) - Mathf.Round(grow_LF); // Calc diff degree.
                    grow_LF = grow_CF; // Update Grow Last Frame.
                   // print("[] pitch []Do Grow ->:" + do_grow);
                    roteramig.transform.Rotate(new Vector3(do_grow * 0.5f, 0, 0), Space.World);
                    roteramig.transform.Rotate(new Vector3(0, 0, 0), Space.World);
                }
                else if (WeOnlyDo == 2)
                {
                    grow_CF = growth.x; // Update Grow Current Frame.
                    dist = Mathf.Abs(grow_CF - grow_LF);
                    if (dist < 0.15f) // If not diff. enough stay as Last Frame.
                        grow_CF = grow_LF;
                    do_grow = Mathf.Round(grow_CF) - Mathf.Round(grow_LF); // Calc diff degree.
                    grow_LF = grow_CF; // Update Grow Last Frame.
                  //  print("[] yaw []Do Grow ->:" + do_grow);
                    roteramig.transform.Rotate(new Vector3(0, -do_grow * 0.5f, 0), Space.World);
                    roteramig.transform.Rotate(new Vector3(0, 0, 0), Space.World);
                }
                else if (WeOnlyDo == 3)
                {
                    grow_CF = growth.z; // Update Grow Current Frame.
                    dist = Mathf.Abs(grow_CF - grow_LF);
                    if (dist < 0.15f) // If not diff. enough stay as Last Frame.
                        grow_CF = grow_LF;
                    do_grow = Mathf.Round(grow_CF) - Mathf.Round(grow_LF); // Calc diff degree.
                    grow_LF = grow_CF; // Update Grow Last Frame.
                   // print("[] roll []Do Grow ->:" + do_grow);
                    roteramig.transform.Rotate(new Vector3(0, 0, -do_grow * 0.5f), Space.World);
                    roteramig.transform.Rotate(new Vector3(0, 0, 0), Space.World);
                }
            }
        }
    }

    void DoWeTranslate(int colorState)
    {
        if (grabDist < grab_tresh && dll_released == true && dll_grabb == false && colorState == 2)
        {
            //Debug.Log("Grabbed!");
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
        }
    }

    private float FingerUpdate()
    {
        Marshal.Copy(ImageUpdate(), FINGER_POINTS, 0, FINGER_POINTS.Length);

        return 0.0f;
    }

    void OnApplicationQuit()
    {
#if UNITY_EDITOR
        if (!OPTask.IsCompleted)
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