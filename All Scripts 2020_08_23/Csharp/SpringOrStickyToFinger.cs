using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Animations;

public class SpringOrStickyToFinger : MonoBehaviour
{
    //Global
    protected SpringJoint springJoint;
    protected GameObject collidedObj;
    ConstraintSource constraintSource;
    protected Vector3 CF_pos, LF_pos;

    int COUNTER = 0;
    [SerializeField]
    [Range(0.001f, 0.4f)]
    private float UnstickThreashold = 0.03f;
    bool reset = false;
    bool firstTime = true;
    //More Global
    protected Gradient gradient;
    protected LineRenderer line;
    Vector3 _avg, VEC_F2O;
    float DIST_F2O;
    //list of positions to average
    Vector3[] posArray = new Vector3[2];

    private bool SPRING_MODE;
    private string I_MODE = "SPRING";

    // Start is called before the first frame update
    void Start()
    {   // For "Sticky...".
        CF_pos = Vector3.zero;
        LF_pos = CF_pos;

        
        SPRING_MODE = true;
        // Create a Line.
        line = GetComponent<LineRenderer>();
        // A simple 2 color gradient with a fixed alpha of 1.0f.
        gradient = new Gradient();
        gradient.SetKeys(
            new GradientColorKey[] { new GradientColorKey(Color.green, 0.0f), new GradientColorKey(Color.green, 1.0f) },
            new GradientAlphaKey[] { new GradientAlphaKey(0f, 0.88f), new GradientAlphaKey(1.0f, 0.99f) }
        );

    }

    // Update is called once per frame
    void Update()
    {


        if (reset)
            COUNTER++;

        if (Input.GetKey(KeyCode.PageUp))
        {
            I_MODE = "SPRING";
            SPRING_MODE = true;
            if (gameObject.transform.childCount == 1)
                NoMoreParent();
            //Debug.Log("SPRING-MODE ACTIVATED [O.o]");
        }
        else if (Input.GetKey(KeyCode.PageDown))
        {

            I_MODE = "STICKY";
            SPRING_MODE = false;
            // Release from current Spring grasp.
            if (collidedObj)
            {
                Destroy(gameObject.GetComponent<SpringJoint>());
                Destroy(gameObject.GetComponent<LineRenderer>());
                collidedObj.GetComponent<Rigidbody>().isKinematic = true;
                collidedObj.GetComponent<Rigidbody>().isKinematic = false;
                NoMoreParent();

            }
            //Debug.Log("STICKY-MODE ACTIVATED [^_-]");
        }

        if (SPRING_MODE)
        {
            if (GetComponent<SpringJoint>())
            {
                posArray[0] = transform.position;
                posArray[1] = collidedObj.gameObject.transform.position;
                line.colorGradient = gradient;
                line.SetPositions(posArray);
                /*
                float dist = Vector3.Distance(posArray[0],posArray[1]);
                line.startWidth = Mathf.Clamp(0.0001f * (1 / (dist*dist)) , 0.00002f, 0.02f);
                line.endWidth = Mathf.Clamp(0.0001f * (1 / (dist * dist)), 0.00002f, 0.02f);
            */
                var forceVec = springJoint.currentForce;
                var thiccness = forceVec.magnitude / 1.5f;
                line.startWidth = Mathf.Max(0.0001f, (0.015f - (thiccness * 0.015f) ));
                line.endWidth = Mathf.Max(0.0001f, (0.015f - (thiccness * 0.015f)));
            }
        }// For Spring Mode.
        else
        {
            //CF_pos = transform.position;
            CF_pos = transform.GetComponent<OPRSHandTracker16fps>().GetRawPos();
            if (Vector3.Distance(CF_pos, LF_pos) > UnstickThreashold)
            {
                print("I TRY TO LET GO");
                NoMoreParent();
            }
            LF_pos = CF_pos;
        }// For Sticky Mode.
    }//END OF UPDATE

    void OnCollisionEnter(Collision collision)
    {

        print("I COLLIDE");
        if (SPRING_MODE)
        {
            // Means NOT NULL
            if (!GetComponent<SpringJoint>())//if SpringJoint exist DONT do anything. But is SprigJoins is Missing THEN go to town.
            {
                // #################################################
                // [*] Find the mean position of the contact points!
                //list of positions to average
                List<Vector3> contact_points_positions = new List<Vector3>();
                foreach (ContactPoint contact in collision.contacts)
                {
                    contact_points_positions.Add(contact.point);
                }
                //get the average of the objects in positions
                Vector3 avg = GetMeanVector(contact_points_positions);
                _avg = avg;

                // ###########################################
                // [*] Create and Configure the SpringJoint!!! 
                collidedObj = collision.gameObject;
                SpringJoint _SJ = gameObject.AddComponent<SpringJoint>();
                _SJ.connectedBody = collision.rigidbody;
                _SJ.anchor = new Vector3(0, 0, 0);
                _SJ.autoConfigureConnectedAnchor = false;
                _SJ.connectedAnchor = _avg - collidedObj.transform.position;
                _SJ.spring = 22;
                _SJ.damper = 2f;
                _SJ.minDistance = 0.1f;
                _SJ.maxDistance = 0;
                _SJ.tolerance = 0.05f;
                _SJ.breakForce = 1.5f;
                _SJ.enableCollision = true;
                _SJ.massScale = 1f;
                _SJ.connectedMassScale = 0.5f;

                springJoint = _SJ;
            }
            if (!GetComponent<LineRenderer>())
            {
                // ###################################################
                // [*] Fill up the position Array for Line Rendering!!
                line = gameObject.AddComponent<LineRenderer>();
                line.startWidth = 0.0075f;
                line.endWidth = 0.0075f;

                Material defaultMat = new Material(Shader.Find("Sprites/Default"));
                line.material = defaultMat;
            }
        }
        else // (STICKY MODE)
        {
            if (COUNTER > 8 || firstTime)
            {

                if (!collidedObj)
                {

                    collidedObj = collision.gameObject;
                    collision.gameObject.transform.parent = gameObject.transform;
                    collision.gameObject.AddComponent<ParentConstraint>();
                    var parentConstraint = collision.gameObject.GetComponent<ParentConstraint>();
                    if (parentConstraint.sourceCount > 0)
                    {
                        parentConstraint.RemoveSource(0);
                    }
                    constraintSource.sourceTransform = gameObject.transform;
                    constraintSource.weight = 1;
                    if (parentConstraint.sourceCount == 0)
                    {
                        parentConstraint.AddSource(constraintSource);
                        //parentConstraint.locked = true;
                        parentConstraint.constraintActive = true;
                        parentConstraint.SetTranslationOffset(0, (collision.gameObject.transform.position - transform.position));
                        parentConstraint.SetRotationOffset(0, collision.gameObject.transform.rotation.eulerAngles);
                    }


                }

            COUNTER = 0;
            reset = false;
            firstTime = false;
            }
        }


        

    }

    void OnJointBreak(float breakForce)
    {
        if (collidedObj)
        {
            springJoint = null;
            Destroy(gameObject.GetComponent<LineRenderer>());
            collidedObj.GetComponent<Rigidbody>().isKinematic = true;
            collidedObj.GetComponent<Rigidbody>().isKinematic = false;
            collidedObj = null;
            //Debug.Log("A joint has just been broken!, force: " + breakForce);
        }
    }
    /*
    private void OnGUI()
    {
        GUIStyle fontSize = new GUIStyle(GUI.skin.GetStyle("label"));
        fontSize.fontSize = 24;
        GUI.Label(new Rect(100, 200, 200, 100), "Interaction Mode: " + I_MODE, fontSize);
    }
    */




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

    private void NoMoreParent()
    {
        //Debug.Log("NoMoreParent HAS BEEN CALLED!");
        
        // If selected object exist and it is stuck on finger then let go!
        if (collidedObj)
        {
            if (collidedObj.transform.parent)
            {
                print("NO MORE PARENT");

                var parentConstraint = collidedObj.transform.GetComponent<ParentConstraint>();
                parentConstraint.constraintActive = false;
                Destroy(parentConstraint);

                collidedObj.GetComponent<Rigidbody>().isKinematic = true;
                collidedObj.GetComponent<Rigidbody>().isKinematic = false;
                collidedObj.transform.parent = null;
                collidedObj = null;
            }

            reset = true;
        }
    }
}