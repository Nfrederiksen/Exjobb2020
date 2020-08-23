using System;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using UnityEngine;
using System.Collections;
using UnityEngine.Animations;
using UnityEditor;

public class PuzzleResetScript : MonoBehaviour
{
    // Start is called before the first frame update
    int counter;
    Vector3 StartPos, StartRot, _goal;
    GameObject _index, resetButton, puzzlePiece, puzzlePart1, puzzlePart2, puzzlePart3, canvasL,canvasR;
    public Vector3 correctAngles;
    List<string> LEVEL_MATS_NAMES = new List<string>();
    int[] angles;
    List<Vector3> StartAngles;
    public bool useMaterials2 = false;
    private string folder;

    void Start()
    {
        if (useMaterials2)
            folder = "Materials2";
        else
            folder = "Materials";

        counter = 0;

        angles = new int[] { 0, 90, 180, 270 };
        StartAngles = new List<Vector3> { new Vector3(90f, 90f, 0f), new Vector3(0f, 180f, 0f), new Vector3(0f, 90f, 90f), new Vector3(0f, 0f, 180f) };
        StartPos = transform.position;
        StartRot = transform.rotation.eulerAngles;
        _index = GameObject.FindGameObjectWithTag("index");
        resetButton = GameObject.FindGameObjectWithTag("reset_button");
        puzzlePiece = GameObject.Find("puzzle_piece");
        puzzlePart1 = GameObject.Find("Puzzle Part 1");
        puzzlePart2 = GameObject.Find("Puzzle Part 2");
        puzzlePart3 = GameObject.Find("Puzzle Part 3");
        canvasL = GameObject.Find("Canvas TEXT L");
        canvasR = GameObject.Find("Canvas TEXT R");
        _goal = puzzlePiece.transform.GetComponent<SnapToGoal>().GetGOAL();
        correctAngles = new Vector3(0.0f, 0.0f, 0.0f);

        var loadedObjects = Resources.LoadAll(folder, typeof(Material));
        foreach (var mat in loadedObjects)
        {
            LEVEL_MATS_NAMES.Add(mat.name);
        }
        // ===[ Load Next Level ]
        Material mm = Resources.Load<Material>(folder+"/level1");
        puzzlePiece.transform.GetComponent<MeshRenderer>().material = mm;
        puzzlePart1.transform.GetComponent<MeshRenderer>().material = mm;
        puzzlePart2.transform.GetComponent<MeshRenderer>().material = mm;
        puzzlePart3.transform.GetComponent<MeshRenderer>().material = mm;
        counter++;
    }



    // Update is called once per frame
    void Update()
    {


        // ===[Case 1: We press the reset button]
        if (Vector3.Distance(_index.transform.position, resetButton.transform.position) < 0.05)
        {
            if (transform.position != StartPos)
            {

                transform.GetComponent<Rigidbody>().isKinematic = true;
                transform.GetComponent<Rigidbody>().isKinematic = false;
                transform.eulerAngles = StartRot;
                transform.position = StartPos;

            }

            resetButton.GetComponent<Renderer>().material.color = new Color(0f, 0f, 0f, 0.5f);
        }
        else
            resetButton.GetComponent<Renderer>().material.color = Color.white;


        var ea = puzzlePiece.transform.rotation.eulerAngles;
        //print("ea" + ea);
        //print("ca" + correctAngles);
      

        ea.x = (int)ea.x;
        ea.y = (int)ea.y;
        ea.z = (int)ea.z;


        // ===[Case 2: We completed the levels 1 or 2]
        if (counter < 3 && ea.x.Equals(correctAngles.x) && ea.y.Equals(correctAngles.y) && puzzlePiece.transform.position == _goal && !canvasL.GetComponent<Canvas>().enabled)
        {
            StartCoroutine(LoadNextLevel());
        }
        // ===[Case 3: We completed the levels 2 or 3]
        else if (ea.Equals(correctAngles) && puzzlePiece.transform.position == _goal && !canvasL.GetComponent<Canvas>().enabled)
        {
            StartCoroutine(LoadNextLevel());
        }
 
    }

    Vector3 RandomizedStartRotation()
    {
        //bool loop = true;
        //int x = 0, y = 0, z = 0;

        /*while (loop)
        {
            System.Random rnd = new System.Random();

            int xIndex = rnd.Next(angles.Length);
            int yIndex = rnd.Next(angles.Length);
            int zIndex = rnd.Next(angles.Length);

            x = angles[xIndex];
            y = angles[yIndex];
            z = angles[zIndex];

            if (counter < 2)
            {
                if (x.Equals(0) && y.Equals(0) && z.Equals(0))
                    loop = true;
                else
                    loop = false;
            }
                
            if (counter > 1)
            {

                if (x.Equals(0) && y.Equals(270) && z.Equals(0))
                    loop = true;
                else
                    loop = false;
            }

        }*/


        StartRot = StartAngles[counter];
        return StartRot;
    }

    IEnumerator LoadNextLevel()
    {
        // FEEDBACK FOR WINNING
        canvasL.GetComponent<Canvas>().enabled = true;
        canvasR.GetComponent<Canvas>().enabled = true;

        yield return new WaitForSecondsRealtime(2); 

        canvasL.GetComponent<Canvas>().enabled = false;
        canvasR.GetComponent<Canvas>().enabled = false;

        // ===[ Reset all pieces ]
        if (transform.position != StartPos)
        {

            transform.GetComponent<Rigidbody>().isKinematic = true;
            transform.GetComponent<Rigidbody>().isKinematic = false;
            transform.eulerAngles = RandomizedStartRotation();
            transform.position = StartPos;

        }

        if(counter < 2)
        {
            // ===[ Load Next Level ]
            Material mm = Resources.Load<Material>(folder+"/" + LEVEL_MATS_NAMES[counter]);
            puzzlePiece.transform.GetComponent<MeshRenderer>().material = mm;
            puzzlePart1.transform.GetComponent<MeshRenderer>().material = mm;
            puzzlePart2.transform.GetComponent<MeshRenderer>().material = mm;
            puzzlePart3.transform.GetComponent<MeshRenderer>().material = mm;

            puzzlePart1.transform.rotation = new Quaternion(0f, 0f, 0f, 1f);
            puzzlePart2.transform.rotation = new Quaternion(0f, 0f, 0f, 1f);
            puzzlePart3.transform.rotation = new Quaternion(0f, 0f, 0f, 1f);
        }
        else
        {
            if(counter == 2)
            {
                correctAngles = new Vector3(0.0f, 270.0f, 0.0f);
                puzzlePart1.transform.eulerAngles = new Vector3(0.0f, 180.0f, 0.0f);
                puzzlePart2.transform.eulerAngles = new Vector3(90.0f, 0.0f, 0.0f);
                puzzlePart3.transform.eulerAngles = new Vector3(0.0f, 90.0f, 0.0f);

            }

            // ===[ Load Next Level ]
            Material mm = Resources.Load<Material>(folder + "/" + LEVEL_MATS_NAMES[counter]);
            puzzlePiece.transform.GetComponent<MeshRenderer>().material = mm;
            puzzlePart1.transform.GetComponent<MeshRenderer>().material = mm;
            puzzlePart2.transform.GetComponent<MeshRenderer>().material = mm;
            puzzlePart3.transform.GetComponent<MeshRenderer>().material = mm;
        }


        if (counter < LEVEL_MATS_NAMES.Count-1)
            counter++;

        _index.transform.GetComponent<Collider>().enabled = true;
    }
}
