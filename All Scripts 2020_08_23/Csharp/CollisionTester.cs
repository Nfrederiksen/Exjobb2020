using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Animations;
using UnityEditor;
public class CollisionTester : MonoBehaviour
{



    protected Transform CubeTransform = null;
    ConstraintSource constraintSource;
    bool objectselected = false;
    bool iLetGo = false;
    private GameObject globalis, cam1, cam2, thumb, middle, canvasL, canvasR, resetButton, pp;
    private Vector3 BaseNormal, ActiveNormal;

    protected List<GameObject> GetAllObjectsOnlyInScene()
    {
        List<GameObject> objectsInScene = new List<GameObject>();

        foreach (GameObject go in Resources.FindObjectsOfTypeAll(typeof(GameObject)) as GameObject[])
        {
            if (!EditorUtility.IsPersistent(go.transform.root.gameObject) && !(go.hideFlags == HideFlags.NotEditable || go.hideFlags == HideFlags.HideAndDontSave))
            {
                if (go.tag.Equals("petamig"))
                {
                    objectsInScene.Add(go);
                    Debug.Log("Jag finns!: " + go.name);
                }
                if (go.tag.Equals("MainCamera"))
                {
                    Debug.Log("Jag hitta Cam1: " + go.name);
                    cam1 = go;
                }
                if (go.tag.Equals("cam 2"))
                {
                    Debug.Log("Jag hitta Cam2: " + go.name);
                    cam2 = go;
                }
                if (go.tag.Equals("thumb"))
                {
                    Debug.Log("Jag hitta tumme: " + go.name);
                    thumb = go;
                }
                if (go.tag.Equals("middle"))
                {
                    Debug.Log("Jag hitta middle finger: " + go.name);
                    middle = go;
                }
                if (go.tag.Equals("canvasLeft"))
                {
                    canvasL = go;
                    Debug.Log("Interactable gameObject Added to List!: " + go.name);
                }
                if (go.tag.Equals("canvasRight"))
                {
                    canvasR = go;
                    Debug.Log("Interactable gameObject Added to List!: " + go.name);
                }
                if (go.tag.Equals("reset_button"))
                {
                    resetButton = go;
                    Debug.Log("Interactable gameObject Added to List!: " + go.name);
                }
            }
        }

        return objectsInScene;
    }

    void Start()
    {
        pp = GameObject.Find("puzzle_piece");
        pp.GetComponent<Rigidbody>().isKinematic = true;
    }

    protected int CanISelect(List<GameObject> objectsInSceneboi2)
    {
        var i = GameObject.Find("puzzle_piece");
        var finger_coll = GetComponent<Collider>(); 
        var collider = i.GetComponent<Collider>();

        Vector3 finger_pos = transform.position;
        Vector3 box_pos = i.transform.position;

        Vector3 object_collider_pos = collider.ClosestPoint(finger_pos);
        Vector3 finger_collider_pos = finger_coll.ClosestPoint(object_collider_pos);

        float dist = Vector3.Distance(finger_collider_pos, object_collider_pos);
        BaseNormal = CalcNormalVector(cam2.transform.position, cam1.transform.position, finger_pos);
        ActiveNormal = CalcNormalVector(cam2.transform.position, box_pos, finger_pos);

        var LeftSide_or_RightSide = Vector3.Dot(ActiveNormal, BaseNormal); //If Ans >= 0 -> Left | Else -> Right


        // ## Decide Color State of Object ##003
        if (dist < 0.4*0.1 && dist > 0.01  && objectselected == false && LeftSide_or_RightSide == 0.03131f)// right or left side??
        {
            i.transform.GetComponent<Renderer>().material.color = Color.cyan;
            globalis = i;
            return 4;
        }
        // ===[Condition for turning yellow, region between 1mm and 4 cm above collider.]
        if (dist < 0.06 && dist > 0.001 && objectselected == false )
        {
            i.transform.GetComponent<Renderer>().material.color = Color.yellow;
            globalis = i;
            return 3;
        }

        if (dist <= 0.001 && objectselected == false)
        {
            i.transform.GetComponent<Renderer>().material.color = Color.magenta;
            CubeTransform = null;
            if(!i.gameObject.GetComponent<ParentConstraint>())
                i.gameObject.AddComponent<ParentConstraint>();
            //  Debug.Log(gameObject.name + " has collided with " + i.gameObject.name);
            CubeTransform = i.gameObject.transform;
            return 2;
        }

        if (dist > 0.001 && objectselected == false)
        {
            i.transform.GetComponent<Renderer>().material.color = Color.white;
        }

    return 1;
    }

    protected void ObjectStickToFinger(bool grabbed, bool released, GameObject empty)
    {

        if (CubeTransform != null)
        {
            // Debug.Log("grabbed and released is->" + grabbed + "|" + released);
            // released
            if (grabbed == true && released == true)
            {
                if (CubeTransform.parent != null)
                {
                   // print("NO MORE PARENT");
                    //Debug.Log("ingen parent!");

                    var parentConstraint = CubeTransform.GetComponent<ParentConstraint>();
                    parentConstraint.constraintActive = false;
                    Destroy(parentConstraint);



                    CubeTransform.parent = null;
                    //CubeTransform.GetComponent<Rigidbody>().isKinematic = true;
                    //CubeTransform.GetComponent<Rigidbody>().isKinematic = false;
                    CubeTransform = null;

                    objectselected = false;
                }

            }
            // grabbed
            else if (grabbed == false && released == true)
            {
                if (CubeTransform.parent == null)
                {

                    objectselected = true;

                  //  Debug.Log("jag ä med barn!");
                   // print("GIVE PARENT");

                    empty.transform.parent = gameObject.transform;
                    CubeTransform.parent = empty.transform;
                    CubeTransform.GetComponent<Renderer>().material.color = Color.red;
                    //CubeTransform.gameObject.AddComponent<ParentConstraint>();
                    var parentConstraint = CubeTransform.GetComponent<ParentConstraint>();
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
                        parentConstraint.SetTranslationOffset(0, new Vector3(0, 0, -0.06f));
                        parentConstraint.SetRotationOffset(0, CubeTransform.rotation.eulerAngles);
                    }

                    
                }

            }
        }
    }


    protected bool IsReleased()
    {
        return objectselected;
    }
    protected GameObject WhoAmI()
    {
        objectselected = true;
        //globalis.transform.GetComponent<Renderer>().material.color = Color.green;

        return globalis;
    }
    protected GameObject GiveMeThumbGO()
    {
        return thumb;
    }
    protected GameObject GiveMeMiddleGO()
    {
        return middle;
    }
    protected GameObject GiveMeLeftCanvasGO()
    {
        return canvasL;
    }
    protected GameObject GiveMeRightCanvasGO()
    {
        return canvasR;
    }

    protected GameObject GiveMeResetButtonGO()
    {
        return resetButton;
    }

    protected void StopRotating()
    {
        objectselected = false;
    }

    protected Vector3 CalcNormalVector(Vector3 a, Vector3 b, Vector3 c)
    {
        var side1 = b - a;
        var side2 = c - a;

        var perp = Vector3.Cross(side1, side2);
        var perpLength = perp.magnitude;
        perp /= perpLength;

        return perp;
    }

}




