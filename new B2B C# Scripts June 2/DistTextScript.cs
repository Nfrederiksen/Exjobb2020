using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class DistTextScript : MonoBehaviour
{
    public static float DistValue = 0f;
    Text dist;

    // Start is called before the first frame update
    void Start()
    {
        dist = GetComponent<Text>();
    }

    // Update is called once per frame
    void Update()
    {
        dist.text = "Dist. to Origin Cube: " + DistValue;
    }
}
