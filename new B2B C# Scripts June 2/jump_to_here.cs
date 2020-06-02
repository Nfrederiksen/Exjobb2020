using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class jump_to_here : MonoBehaviour
{
    // Start is called before the first frame update
    Vector3 StartPos;
    void Start()
    {
        StartPos = transform.position;
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKey(KeyCode.Space))
        {
            transform.GetComponent<Rigidbody>().isKinematic = true;
            transform.GetComponent<Rigidbody>().isKinematic = false;
            transform.rotation = new Quaternion(0f,0f,0f,1f);
            transform.position = StartPos;
        }
    }
}
