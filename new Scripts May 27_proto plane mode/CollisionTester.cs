﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Animations;
using UnityEditor;
public class CollisionTester : MonoBehaviour
{

    

    public Transform CubeTransform = null;
    ConstraintSource constraintSource;
    bool objectselected = false;
    private GameObject globalis,cam1,cam2,lePlane; //-new (1/6)
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
                    Debug.Log("Interactable gameObject Added to List!: " + go.name);
                }
                if(go.tag.Equals("MainCamera"))
                {
                    //Debug.Log("Jag hitta Cam1: " + go.name);
                    cam1 = go;
                }
                if (go.tag.Equals("cam 2"))
                {
                    //Debug.Log("Jag hitta Cam2: " + go.name);
                    cam2 = go;
                }
                if (go.tag.Equals("aPlane")) //-new (2/6)
                {
                    Debug.Log("Jag hitta a Plane: " + go.name);
                    lePlane = GameObject.CreatePrimitive(PrimitiveType.Plane);
                }
            }
        }

        return objectsInScene;
    }

    void Start()
    {
        

    }

    protected int CanISelect(List<GameObject> objectsInSceneboi2)
    {
        foreach (GameObject i in objectsInSceneboi2)
        {
            var finger_coll = GetComponent<Collider>();
            var collider = i.GetComponent<Collider>();

            Vector3 finger_pos = transform.position;
            Vector3 box_pos = i.transform.position;
            // Debug.Log("box_pos: " + box_pos + " finger_pos: " + finger_pos);

            //Vector3 mrTest = new Vector3(0.0f, 0.0f, 0.0f);

            Vector3 object_collider_pos = collider.ClosestPoint(finger_pos);
            Vector3 finger_collider_pos = finger_coll.ClosestPoint(object_collider_pos);

            //Debug.Log("##### box_collider_pos: " + object_collider_pos + " finger_collider_pos: " + finger_collider_pos);

            float dist = Vector3.Distance(finger_collider_pos, object_collider_pos);
            //Debug.Log(" ----------------dist: " + dist);

            BaseNormal = CalcNormalVector(cam2.transform.position, cam1.transform.position, finger_pos);
            ActiveNormal = CalcNormalVector(cam2.transform.position, box_pos, finger_pos);

            var LeftSide_or_RightSide = Vector3.Dot(ActiveNormal, BaseNormal); //If Ans >= 0 -> Left | Else -> Right


            // ## Decide Color State of Object ##
            if (dist < 0.4 && dist > 0.01 && objectselected == false && LeftSide_or_RightSide < 0)// right or left side??
            {
                i.transform.GetComponent<Renderer>().material.color = Color.blue;
                globalis = i;
                return 4;
            }

            if (dist < 0.4 && dist > 0.01 && objectselected == false && LeftSide_or_RightSide >= 0)
            {
                i.transform.GetComponent<Renderer>().material.color = Color.yellow;
                //CubeTransform = null;
                //i.gameObject.AddComponent<ParentConstraint>();
                //  Debug.Log(gameObject.name + " has collided with " + i.gameObject.name);
                globalis = i;
                return 3;
            }

            if (dist <= 0.01 && objectselected == false)
            {
                i.transform.GetComponent<Renderer>().material.color = Color.magenta;
                CubeTransform = null;

                i.gameObject.AddComponent<ParentConstraint>();
                //  Debug.Log(gameObject.name + " has collided with " + i.gameObject.name);
                CubeTransform = i.gameObject.transform;
                return 2;
            }

            if (dist > 0.01 && objectselected == false)
            {
                i.transform.GetComponent<Renderer>().material.color = Color.gray;
            }

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
                    print("NO MORE PARENT");
                    Debug.Log("ingen parent!");

                    var parentConstraint = CubeTransform.GetComponent<ParentConstraint>();
                    parentConstraint.constraintActive = false;
                    Destroy(parentConstraint);

                    CubeTransform.parent = null;
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

                    Debug.Log("jag ä med barn!");
                    print("GIVE PARENT");

                    empty.transform.parent = lePlane.transform;//gameObject.transform; (3/6)
                    CubeTransform.parent = empty.transform;
                    CubeTransform.GetComponent<Renderer>().material.color = Color.red;
                    //CubeTransform.gameObject.AddComponent<ParentConstraint>();
                    var parentConstraint = CubeTransform.GetComponent<ParentConstraint>();
                    if (parentConstraint.sourceCount > 0)
                    {
                        parentConstraint.RemoveSource(0);
                    }
                    constraintSource.sourceTransform = lePlane.transform;//gameObject.transform; (4/6)
                    constraintSource.weight = 1;
                    if (parentConstraint.sourceCount == 0)
                    {
                        parentConstraint.AddSource(constraintSource);
                        //parentConstraint.locked = true;
                        parentConstraint.constraintActive = true;
                        parentConstraint.SetTranslationOffset(0, new Vector3(0, 0.85f, 0.07f)); //-new (5/6)
                        parentConstraint.SetRotationOffset(0, CubeTransform.rotation.eulerAngles);
                    }
                }



            }
        }
    }

    protected GameObject WhoAmI()
    {
        objectselected = true;
        //globalis.transform.GetComponent<Renderer>().material.color = Color.green;

        return globalis;
    }

    protected GameObject WhoIsPlane() //-new (6/6)
    {
        return lePlane;
    }

    protected void StopRotating()
    {
        objectselected = false;
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

}

