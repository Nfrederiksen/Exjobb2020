using System;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using UnityEngine;
using System.Collections;
using UnityEngine.Animations;
using UnityEditor;


/// <summary>
///  OU T DA T E D
/// </summary>
public class jump_to_here : MonoBehaviour
{
    // Start is called before the first frame update
    Vector3 StartPos;
   Vector3 StartRot;
    GameObject _index, resetButton;
    void Start()
    {
        StartPos = transform.position;
        StartRot = transform.rotation.eulerAngles;
        _index = GameObject.FindGameObjectWithTag("index");
        resetButton = GameObject.FindGameObjectWithTag("reset_button");

    }

   

    // Update is called once per frame
    void Update()
    {
        if (Vector3.Distance(_index.transform.position,resetButton.transform.position) < 0.05 )
        {
            if(transform.position != StartPos)
            {

                transform.GetComponent<Rigidbody>().isKinematic = true;
                transform.eulerAngles = StartRot;
                transform.position = StartPos;
                transform.GetComponent<Rigidbody>().isKinematic = false;
            }

            resetButton.GetComponent<Renderer>().material.color = new Color(0f,0f,0f,0.5f);
        }
        else
            resetButton.GetComponent<Renderer>().material.color = Color.white;
    }
}
