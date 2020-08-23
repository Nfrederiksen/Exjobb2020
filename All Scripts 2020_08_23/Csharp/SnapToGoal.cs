using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SnapToGoal : MonoBehaviour
{
    GameObject pp, finger;
    public Vector3 GOAL = new Vector3(-0.0735f, (0.1f + 0.07f), -0.025f);
    bool victory_royale = false;
    bool dropped;
    // Start is called before the first frame update
    void Start()
    {
        pp = GameObject.Find("puzzle_piece");
        finger = GameObject.FindGameObjectWithTag("index");

        if (finger)
        {
            //print("finger!");
        }
    }

    // Update is called once per frame
    void Update()
    {
        float distance = Vector3.Distance(pp.transform.position, GOAL);

        Vector3 correctAngles_copy = pp.transform.GetComponent<PuzzleResetScript>().correctAngles;

        if (finger.transform.GetComponent<OPRSHandTrackerAndPlaneInteraction16fps>())
            dropped = finger.transform.GetComponent<OPRSHandTrackerAndPlaneInteraction16fps>().released;
        else if(finger.transform.GetComponent<OPRSHandTrackerWithGestures>())
            dropped = finger.transform.GetComponent<OPRSHandTrackerWithGestures>().isReleased;

        //print("dist:" + distance);
        if (!dropped)
        {
            victory_royale = false;
        }

        if (!victory_royale)
        {
            if (distance < 0.01)
            {
                //print("jadå" + pp.transform.parent);
                var ROTxyz = pp.transform.rotation.eulerAngles;
                var ROT90 = ROTxyz;
                var d = (ROT90.x / 90f);
                ROT90.x = Mathf.Round(d) * 90f;
                d = (ROT90.y / 90f);
                ROT90.y = Mathf.Round(d) * 90f;
                d = (ROT90.z / 90f);
                ROT90.z = Mathf.Round(d) * 90f;

                if (Mathf.Abs(ROT90.x - ROTxyz.x) < 15f && Mathf.Abs(ROT90.y - ROTxyz.y) < 15f && Mathf.Abs(ROT90.z - ROTxyz.z) < 15f)
                {

                    print("Inneeeee!");
                    ROT90.x = (int)ROT90.x;
                    ROT90.y = (int)ROT90.y;
                    ROT90.z = (int)ROT90.z;
                    print("ROT90.y: " + ROT90.y);

                    if (ROT90.x.Equals(360)) ROT90.x = 0;
                    if (ROT90.y.Equals(360)) ROT90.y = 0;
                    if (ROT90.z.Equals(360)) ROT90.z = 0;


                    if (ROT90.x.Equals(correctAngles_copy.x) && ROT90.y.Equals(correctAngles_copy.y))
                    {
                        print("Inne del 2");
                        finger.transform.GetComponent<Collider>().enabled = false;

                        pp.transform.eulerAngles = ROT90;
                        pp.transform.position = GOAL;
                        //victory_royale = true;

                    }
                }
                //print("x:" + ROTxyz.x + "|y: " + ROTxyz.y + "|z:" + ROTxyz.z);
        
            }
        }

    }

    public Vector3 GetGOAL()
    {
        return GOAL;
    }
}
