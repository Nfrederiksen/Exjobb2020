using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class PositionTextScript : MonoBehaviour
{
    public static Vector3 posValue = Vector3.zero;
    Text pos;

    // Start is called before the first frame update
    void Start()
    {
        pos = GetComponent<Text>();    
    }

    // Update is called once per frame
    void Update()
    {
        pos.text = "X: " + posValue.x + "\nY: " + posValue.y + "\nZ: " + posValue.z;
    }
}
