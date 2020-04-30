using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Animations;
using UnityEditor;
public class CollisionTester : MonoBehaviour
{

    

    public Transform CubeTransform = null;
    ConstraintSource constraintSource;
    bool objectselected = false;
    private GameObject globalis;

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
            
            if (dist < 0.4 && dist > 0.01 && objectselected == false)
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

                    CubeTransform.GetComponent<Renderer>().material.color = Color.red;

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

                    Debug.Log("jag ä med barn!");
                    print("GIVE PARENT");

                    empty.transform.parent = gameObject.transform;
                    CubeTransform.parent = empty.transform;

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
                        parentConstraint.SetTranslationOffset(0, new Vector3(0, 0, 0.07f));
                        parentConstraint.SetRotationOffset(0, CubeTransform.rotation.eulerAngles);
                    }
                }
                else if (Input.GetKey(KeyCode.Mouse3))
                {
                    Vector3 scale = new Vector3(1, 1, 1);
                    CubeTransform.localScale += (scale * 0.25f * Time.deltaTime);
                }
                else if (Input.GetKey(KeyCode.Mouse4))
                {
                    Vector3 scale = new Vector3(1, 1, 1);
                    CubeTransform.localScale -= (scale * 0.25f * Time.deltaTime);
                }


            }
        }
    }

    protected GameObject WhoAmI()
    {
        objectselected = true;
        globalis.transform.GetComponent<Renderer>().material.color = Color.green;

        return globalis;
    }

    protected void StopRotating()
    {
        objectselected = false;
    }

}



/*
    // void OnCollisionEnter(Collision col)
    void OnTriggerEnter(Collider col)
    {
        Debug.Log("Trigger On : " +  col.isTrigger);
        CubeTransform = null;
        col.gameObject.AddComponent<ParentConstraint>();
        Debug.Log(gameObject.name + " has collided with " + col.gameObject.name);
        CubeTransform = col.gameObject.transform;
        CubeTransform.GetComponent<Renderer>().material.color = Color.yellow;
        //CubeTransform.GetComponent<Rigidbody>().isKinematic = false;
    }

    void OnCollisionExit(Collision col)
    {

        if(col.transform.parent == null) //If orphan, destroy parent constraints.
        {

            var pcon = col.gameObject.GetComponent<ParentConstraint>();
            Destroy(pcon);
            if(CubeTransform != null)
            {
                CubeTransform.GetComponent<Renderer>().material.color = Color.black;
                CubeTransform = null;
            }
        }
    }
}
*/